#include <Wire.h>
#include <Adafruit_DRV2605.h>

#define TCA9548A_ADDR 0x70
#define NUM_HAPTICS 5

const uint8_t HAPTIC_CHANNELS[NUM_HAPTICS] = {2, 3, 4, 5, 6};

Adafruit_DRV2605 drv; 

struct __attribute__((__packed__)) HapticCommand
{
  uint8_t driver;
  uint8_t effect; 
};

const byte START_BIT = 0xAA;
const byte END_BIT = 0x55;

uint8_t state = 0;

unsigned long haptic_wait_start = 0;
const unsigned long HAPTIC_WAIT_TIMEOUT_MS = 50; 

void tcaSelect(uint8_t channel)
{
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10); 

  Wire.begin(); // TCA9548A -> DRV2605 drivers

  
  for (uint8_t i = 0; i < NUM_HAPTICS; i++)
  {
    tcaSelect(HAPTIC_CHANNELS[i]);
    if (!drv.begin())
    {
      Serial.print("DRV2605 not detected on channel ");
      Serial.println(HAPTIC_CHANNELS[i]);
    }
    else
    {
      drv.selectLibrary(1);               
      drv.setMode(DRV2605_MODE_INTTRIG);  
    }
  }
}

void loop(void)
{
  switch (state) {
    case 0:
      {
        if (Serial.available() > 0)
        {
          int incomingByte = Serial.read();
          if (incomingByte == 2)
          {
            state = 4; // haptic trigger request
            haptic_wait_start = millis();
          }
        }
        break;
      }

    case 4:
      {
       
        if (Serial.available() >= (int)sizeof(HapticCommand))
        {
          HapticCommand cmd;
          Serial.readBytes((char*)&cmd, sizeof(cmd));

          if (cmd.driver < NUM_HAPTICS)
          {
            tcaSelect(HAPTIC_CHANNELS[cmd.driver]);
            drv.setWaveform(0, cmd.effect); 
            drv.setWaveform(1, 0);          
            drv.go();

            Serial.write(START_BIT);
            Serial.write((byte*)&cmd, sizeof(cmd));
            Serial.write(END_BIT);
          }

          state = 0;
        }
        else if (millis() - haptic_wait_start > HAPTIC_WAIT_TIMEOUT_MS)
        {
          
          state = 0;
        }
        break;
      }
  }
}
