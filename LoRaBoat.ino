// PYC Race boat postion sending unit.
// When started, sends boat position every 4 seconds based on assigned offset.
// Say boat is in time slot 1, then send at 1, 5, 9, 13, 17, 21, 25, 29, 33, 37, 41, 45, 49, 53, 57 seconds.
// If boat is in time slot 2, then send at 2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 50, 54, 58 seconds.
// If boat is in time slot 3, then send at 3, 7, 11, 15, 19, 23, 27, 31, 35, 39, 43, 47, 51, 55, 59 seconds.
// Boats are also assigned a 10ths of a second slot, so that they can be distinguished if they are in the same second slot.
// See database for a listing of boat ids and their assigned time slots.

// Boats all take there times on even seconds, 0, 2, 4 etc..

// Modules are manually started up by pressing the reset button.
// The boat module will go to sleep after 5 hours of operation.

// Scott Nichols
// 2/23/2025

// the GPS module used is GPS.
#include "Arduino.h"
#include "GPS_Air530Z.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"

SSD1306Wire displayBd(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , resolution , rst
Air530ZClass GPS;

#define DEVICE_ID 10   // Device ID in database
#define TXSECONDSLOT 1 // Tx slot for secondx - see database for assigned time slot i.e 1.2 means 1 second slot and 2 tenths of a second slot.
#define TXTENTHSSLOT 1 // Tx slot for tenths of second - see database for assigned time slot

#define TXSECTIMETRIG 4      // Time triggerred sampling - transmit every 4 seconds for boats.
#define GPSREADSECTIMETRIG 2 // time triggered reading of GPS at 2 seconds (0, 2, 4, 6, 8, 10, etc.)

// LORA SETUP
#define RF_FREQUENCY 915000000 // Hz

#define TX_OUTPUT_POWER 14 // dBm

#define LORA_BANDWIDTH 2        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 8 // [SF7..SF12]
#define LORA_CODINGRATE 2       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// TRANSMIT SETTINGS
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 200 // room for 4 messages

#define BOATAUTOSLEETIME 18000000 // 5 hours then go to sleep. Reset needed to wake up. 3600000 = 1 hour

const int MessageType = 1;     // Boat Message type - used for parsing algorithm on server side.
int WaitingGPSCounter = 0;     // Incrument while waiting for GPS to be picked up.
char transmitStr[BUFFER_SIZE]; // Used for holding the Tx message
char gpsMsgBuffer[BUFFER_SIZE];
int secondsCounter = -99; // Used for counting seconds for Tx slot

int startTran = 0; // Used for timing the Tx message
int gpsLat = 0;    // hold Lat and long from GPS
int gpsLong = 0;
bool successfulGPSMessage = false;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

void OnNavLightOn(void);
void OnNavLightOff(void);
int getDayOfWeek(int year, int month, int day);
bool buildGPSTXMessage(void);

uint16_t voltage = 0; // Reading battery voltage level
typedef enum
{
  stWaitGPSBoot,
  stWaitGPS1PPS,
  st1SecPulseGrapGPSData,
  stTxString,
  stReadGPSSave2BufferAndTransmit,
  stReadGPSSave2Buffer
} States_t;

States_t StateMachine = stWaitGPSBoot;
uint32_t starttime = 0;

int fracPart(double val, int n) // Cuts apart GPS lat/long
{
  val = abs(val);
  return (int)((val - (int)(val)) * pow(10, n));
}

void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) // Vext default OFF
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void GPS1SecPulse(void)
{

  StateMachine = st1SecPulseGrapGPSData;
}

void setup()
{
  VextON();
  delay(10);

  displayBd.init();
  displayBd.clear();
  displayBd.display();

  displayBd.setTextAlignment(TEXT_ALIGN_CENTER);
  displayBd.setFont(ArialMT_Plain_16);
  displayBd.drawString(64, 32 - 16 / 2, "GPS initing...");
  displayBd.display();

  displayBd.setTextAlignment(TEXT_ALIGN_LEFT);
  displayBd.setFont(ArialMT_Plain_10);

  Serial.begin(115200);
  Serial.print("Looking for GPS......");
  GPS.setmode(MODE_GPS_GLONASS); // US and Europen satalites.
  GPS.setNMEA(0);                // No preprogramed sentances being sent out.
  GPS.begin(57600);

  // Setup Pins usage for the board
  pinMode(GPIO12, INPUT_PULLUP);   // GPS 1 second pulse
  pinMode(GPIO6, OUTPUT_PULLDOWN); // Nav light
  pinMode(ADC2, INPUT);            // Solar panel voltage

  // Setup LoRa
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  StateMachine = stWaitGPSBoot; // intial state
}

void loop()
{
  // Check if time to go to sleep, need reset to wake up.
  if (millis() > BOATAUTOSLEETIME)
  {
    // Go to sleep
    Serial.println("Going to sleep" + String(millis()));
    VextOFF();
    delay(100);
    VextOFF();
    delay(100);
    // Put Arduino into deep sleep indefinitely
    detachInterrupt(digitalPinToInterrupt(GPIO12)); // Remove GPS 1 second pulse
    GPS.end();
    Radio.Sleep();

    // displayBd.clear();
    turnOffRGB();

    lowPowerHandler();
  }

  switch (StateMachine)
  {
  case stWaitGPSBoot:
    if (GPS.available() > 0)
    {
      attachInterrupt(digitalPinToInterrupt(GPIO12), GPS1SecPulse, RISING);
    }
    WaitingGPSCounter = WaitingGPSCounter + 1;
    displayBd.clear();
    displayBd.drawString(0, 0, String(WaitingGPSCounter));
    displayBd.display();
    Serial.println("Wait GPS Boot.." + String(WaitingGPSCounter));
    delay(1000);
    break;

  case st1SecPulseGrapGPSData:
    // Check if time to send or just wait.
    starttime = millis(); // time of 1 second pulse

    while (GPS.available() > 0)
    {
      GPS.encode(GPS.read());
      secondsCounter = GPS.time.second();
      // Serial.println("GPS seconds=" + String(secondsCounter));
    }

    if ((secondsCounter - TXSECONDSLOT) % TXSECTIMETRIG == 0) // time to send
    {

      if (secondsCounter % GPSREADSECTIMETRIG == 0) // but could also be time to read GPS
      {
        StateMachine = stReadGPSSave2BufferAndTransmit;
      }
      else
      {
        StateMachine = stTxString;
      }
    }
    else if (secondsCounter % GPSREADSECTIMETRIG == 0)
    {
      StateMachine = stReadGPSSave2Buffer;
    }
    else
    {
      // wait for next 1 second pulse
      StateMachine = stWaitGPS1PPS;
    }

    break;

  case stTxString:
    displayBd.clear();
    while ((millis() - starttime) < 100 * TXTENTHSSLOT)
    { // wait till time send.
    }
    turnOnRGB(COLOR_SEND, 0);
    OnNavLightOn();
    displayBd.drawStringMaxWidth(20, 0, 128, transmitStr);
    displayBd.display();

    startTran = millis();

    Radio.Send((uint8_t *)transmitStr, strlen(transmitStr));

    StateMachine = stWaitGPS1PPS; // Wait for 1 second pulse.

    break;

  case stReadGPSSave2BufferAndTransmit:
    // read GPS and save to buffer, then transmit if  GPS is good
    successfulGPSMessage = buildGPSTXMessage();

    if (successfulGPSMessage)
    {
      StateMachine = stTxString;
    }
    else
    {
      StateMachine = stWaitGPS1PPS;
    }

    break;
  case stReadGPSSave2Buffer:
    // read GPS and save to buffer, then wait for next 1 second pulse
    successfulGPSMessage = buildGPSTXMessage();
    StateMachine = stWaitGPS1PPS;
    break;

  } // end of switch
} // end of loop

bool buildGPSTXMessage(void)
{
  // Start building up transmit string

  gpsLat = GPS.location.lat();
  gpsLong = GPS.location.lng();

  if (gpsLat != 0 && gpsLong != 0)
  {
    // Serial.println("buildGPSTXMessage");

    // GPS is good, build up transmit string
    sprintf(gpsMsgBuffer,
            "|%d,%d,"       // Device/msg
            "%02d%02d%02d," // gps time
            "%d.%d,"        // Lat
            "%d.%d,"        // Long
            "%d,%d",        // Heading, speed
            MessageType, DEVICE_ID,
            GPS.time.hour(), GPS.time.minute(), GPS.time.second(),
            gpsLat, fracPart(GPS.location.lat(), 6), // (int)GPS.location.lat()
            gpsLong, fracPart(GPS.location.lng(), 6),
            (int)GPS.course.deg(),
            (int)(GPS.speed.knots() * 10));

    strcat(transmitStr, gpsMsgBuffer);
    gpsMsgBuffer[0] = 0; // clear gpsMsgBuffer
    return true;         // good read
  }
  else
  {
    // GPS is not good, wait for next 1 second pulse
    // Serial.println("bad lat or long");
    return false; // bad read
  }
}
void OnTxDone(void)
{
  // Serial.println("TX done!");
  int endTran = millis();
  turnOffRGB();
  OnNavLightOff();
  Serial.println("Tx time=" + String(endTran - startTran));
  Serial.println(transmitStr);
  transmitStr[0] = 0; // clear transmitStr
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.println("TX Timeout......");
  StateMachine = stWaitGPS1PPS;
}

void OnNavLightOn(void)
{
  digitalWrite(GPIO6, HIGH); // Set GPIO for NAV_LIGHT On
                             // Futre set 1/2 second timer to turn off light.
}
void OnNavLightOff(void)
{
  digitalWrite(GPIO6, LOW); // Set GPIO for NAV_LIGHT Off
}

int getDayOfWeek(int year, int month, int day)
{
  // Zeller's Congruence Algorithm to calculate the day of the week
  if (month % 3)
  {
    month += 12;
    year -= 1;
  }
  int k = year % 100;
  int j = year / 100;
  int dayOfWeek = (day + (13 * (month + 1)) / 5 + k + (k / 4) + (j / 4) - (2 * j)) % 7;
  return (dayOfWeek + 6) % 7; // Adjust to 0=Sunday, 1=Monday, etc.
}