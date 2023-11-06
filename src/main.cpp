/*
pico - cc1101 module
GP16 - MISO (GDO1, SO)
GP17 - CSn
GP18 - SCK
GP19 - MOSI (SI)
GP20 - GDO0
GP21 - GDO2

pico - gps module
GP0(Tx) - Rx
GP1(Rx) - Tx

*/
#define ID "5678"
#define ID_LEN 4

#include <Arduino.h>
#include <SPI.h>
#include <CC1101_RF.h>

CC1101 lora;
#define GDO0 20
bool sendGPSpacket(byte *packet, int len);

#define gps Serial1
int readGPS(byte *_data);

#define LED_G 14
#define LED_B 15
int b_state = 0;
void blink_led();

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(GDO0, INPUT);

  SPI.begin();         // mandatory
  lora.begin(433.4e6); // 433.4 MHz

  lora.setRXstate();

  Serial.begin(9600);

  gps.begin(9600);

  digitalWrite(LED_G, HIGH);
}

void loop()
{
  digitalWrite(LED_G, HIGH);
  static byte _data[64] = {0};
  static int _data_len = 0;
  byte data[64];
  int len = readGPS(data);
  if (len)
  {
    strcpy((char *)_data, (const char *)data);
    _data_len = len;
    Serial.write(data, len);
    Serial.println();
    blink_led();
  }
  if (digitalRead(GDO0))
  {
    len = lora.getPacket(data);
    if (strncmp((const char *)data, ID, ID_LEN) == 0)
    {
      if (sendGPSpacket(_data, _data_len))
      {
        Serial.println("packet sent");
        digitalWrite(LED_B, b_state ^= 1);
      }
    }
  }
}

int readGPS(byte *_data)
{
  if (gps.available())
  {
    int len = gps.readBytesUntil('\n', _data, 61);
    if (strncmp("$GPGLL", (const char *)_data, 6) == 0)
    {
      return len;
    }
  }
  return 0;
}

void blink_led()
{
  static unsigned long t = 0;
  static int s = 0;
  if (millis() - t < 250)
    return;
  s ^= true;
  digitalWrite(LED_BUILTIN, s);
  t = millis();
  return;
}

bool sendGPSpacket(byte *packet, int len)
{
  static unsigned long t = 0;
  static unsigned int cnt = 0;
  static byte buf[64] = {0};
  if (millis() - t < 10)
    return false;

  if (len == 0)
    return false;

  cnt++;
  packet[len] = '\0';
  len = sprintf((char *)buf, "%04u::%s", cnt, (char *)packet);
  Serial.write(buf, len);
  Serial.println();
  bool rtn = lora.sendPacket(buf, len);

  t = millis();
  return rtn;
}