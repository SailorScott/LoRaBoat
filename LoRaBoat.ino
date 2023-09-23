// the GPS module used is GPS.
#include "Arduino.h"
#include "GPS_Air530Z.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "LoRaWan_APP.h"

SSD1306Wire displayBd(0x3c, 500000, SDA, SCL, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , resolution , rst
Air530ZClass GPS;

#define DEVICE_ID 2 // Boat  ID, also used for which Tx Slot

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

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 100

const int MessageType = 1;     // Boat Message
int TxCounter = 0;             // Incrument with each send, used for checking for dropped messages.
char transmitStr[BUFFER_SIZE]; // Used for holding the Tx message

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

typedef enum
{
  stWaitGPSBoot,
  stWaitGPS1PPS,
  stDelayToGPSReadTime,
  stTxString,
  stReadGPS
} States_t;

States_t StateMachine = stWaitGPSBoot;
uint32_t starttime = 0;

int fracPart(double val, int n)
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

  StateMachine = stDelayToGPSReadTime;
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
  GPS.begin();
  pinMode(GPIO12, INPUT_PULLUP);

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

  switch (StateMachine)
  {
  case stWaitGPSBoot:
    if (GPS.available() > 0)
    {
      attachInterrupt(digitalPinToInterrupt(GPIO12), GPS1SecPulse, RISING);
    }
    TxCounter = TxCounter + 1;
    displayBd.clear();
    displayBd.drawString(0, 0, String(TxCounter));
    displayBd.display();
    delay(1000);
    break;

  case stDelayToGPSReadTime:
    // delay(75*DEVICE_ID);
    starttime = millis();
    while (GPS.available() > 0)
    {
      GPS.encode(GPS.read());
    }
    StateMachine = stReadGPS;
    break;

  case stTxString:
    displayBd.clear();
    while ((millis() - starttime) < 75 * DEVICE_ID)
    { // wait till time send.
    }
    turnOnRGB(COLOR_SEND, 0);
    displayBd.drawStringMaxWidth(20, 0, 128, transmitStr);
    displayBd.display();
    Radio.Send((uint8_t *)transmitStr, strlen(transmitStr));
    // turnOffRGB();
    StateMachine = stWaitGPS1PPS; // Wait for 1 second pulse.
    break;

  case stReadGPS:
    // Start building up transmit string
   // TxCounter = TxCounter + 1;
    // if (GPS.available() > 0)
    //   {
    //     GPS.encode(GPS.read());

    sprintf(transmitStr,
            "%d,%d,"            // Device/msg
            "%02d%02d%02d%02d," // date time
            "%d.%d,"           // Lat
            "%d.%d,"           // Long
            "%d,%d",            // Heading, speed
            DEVICE_ID, MessageType,
            GPS.date.day(), GPS.time.hour(), GPS.time.minute(), GPS.time.second(),
            (int)GPS.location.lat(), fracPart(GPS.location.lat(), 6),
            (int)GPS.location.lng(), fracPart(GPS.location.lng(), 6),
            // TxCounter, 
            (int)GPS.course.deg(),
            (int)(GPS.speed.knots() * 10));
    // }
    StateMachine = stTxString;
    break;

  default:
    break;
  }
}

void OnTxDone(void)
{
  Serial.print("TX done!");
  turnOffRGB();
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  Serial.print("TX Timeout......");
  StateMachine = stWaitGPS1PPS;
}

// if( GPS.location.age() < 1000 )
// {
//   displayBd.drawString(120, 0, "A");
// }
// else
// {
//   displayBd.drawString(120, 0, "V");
// }

// index = sprintf(str,"CT: %d",TxCounter);
// str[index] = 0;
// displayBd.drawString(0, 48, str);

// void OrigScreendisplayBd(void)
// {
//     char str[30];
//   int index = sprintf(str,"%02d-%02d-%02d",GPS.date.year(),GPS.date.month(),GPS.date.day());
//   str[index] = 0;
//   displayBd.setTextAlignment(TEXT_ALIGN_LEFT);
//   displayBd.drawString(0, 0, str);

//   index = sprintf(str,"%02d:%02d:%02d.%02d",GPS.time.hour(),GPS.time.minute(),GPS.time.second(),GPS.time.centisecond());
//   str[index] = 0;
//   displayBd.drawString(60, 0, str);

//   index = sprintf(str,"lat :  %d.%d",(int)GPS.location.lat(),fracPart(GPS.location.lat(),7));
//   str[index] = 0;
//   displayBd.drawString(0, 12, str);

//   index = sprintf(str,"lon:%d.%d",(int)GPS.location.lng(),fracPart(GPS.location.lng(),6));
//   str[index] = 0;
//   displayBd.drawString(0, 24, str);

//   index = sprintf(str,"spd: %d.%d mph",(int)GPS.speed.mph(),fracPart(GPS.speed.mph(),1));
//   str[index] = 0;
//   displayBd.drawString(0, 36, str);

//   index = sprintf(str,"Hd: %d",(int)GPS.course.deg());
//   str[index] = 0;
//   displayBd.drawString(70, 36, str);
// }
