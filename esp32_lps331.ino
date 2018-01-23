#include <Wire.h>

#define LPS331_SLABE_ADDR 0x5C //スレーブアドレス
 
#define LPS331_PRESS_OUT_XL   0x28
#define LPS331_PRESS_OUT_L    0x29
#define LPS331_PRESS_OUT_H    0x2A
#define LPS331_CTRL_REG1      0x20

float readPressureMillibars(void);
float readPressureInchesHg(void);
int32_t readPressureRaw(void);
void enableDefault(void);
void writeReg(byte reg, byte value);
byte readReg(byte reg);

void setup() {
    byte who;
  // put your setup code here, to run once:
    Wire.begin(21,22);//sda,scl
    Serial.begin(115200);
    delay(1000);
    enableDefault();
    who = readReg(0x0F);
    Serial.print((who));
    delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  float ap;
  ap = readPressureMillibars();
  Serial.print("大気圧:"); 
   Serial.print((int)(ap));
   Serial.print("\n");
   delay(2000);
   
}

// turns on sensor and enables continuous output
void enableDefault(void)
{
  // active mode, 12.5 Hz output data rate
  writeReg(LPS331_CTRL_REG1, 0b11100000);
}
byte readReg(byte reg)
{
  byte value;

  Wire.beginTransmission(LPS331_SLABE_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // restart
  Wire.requestFrom(LPS331_SLABE_ADDR, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}
void writeReg(byte reg, byte value)
{
  Wire.beginTransmission(LPS331_SLABE_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


// reads pressure in millibars (mbar)/hectopascals (hPa)
float readPressureMillibars(void)
{
  return (float)readPressureRaw() / 4096;
}

// reads pressure in inches of mercury (inHg)
float readPressureInchesHg(void)
{
  return (float)readPressureRaw() / 138706.5;
}

// reads pressure and returns raw 24-bit sensor output
int32_t readPressureRaw(void)
{
  Wire.beginTransmission(LPS331_SLABE_ADDR);
  // assert MSB to enable register address auto-increment
  Wire.write(LPS331_PRESS_OUT_XL | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(LPS331_SLABE_ADDR, (byte)3);

  while (Wire.available() < 3);

  uint8_t pxl = Wire.read();
  uint8_t pl = Wire.read();
  uint8_t ph = Wire.read();
   
  // combine bytes
  return (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}