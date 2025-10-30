
#include <Wire.h>
#include <math.h>

extern int  baseFreq;          
extern void playNote(int freq);  
extern void stopVibrato();
extern void setVibrato(float vibRateHz);
extern int  curFreq;


static const uint8_t MPU_ADDR    = 0x68;   
static const uint8_t REG_PWR1    = 0x6B;
static const uint8_t REG_SMPLRT  = 0x19;
static const uint8_t REG_CONFIG  = 0x1A;
static const uint8_t REG_GYROCFG = 0x1B;
static const uint8_t REG_ACCCFG  = 0x1C;
static const uint8_t REG_INT_EN  = 0x38;
static const uint8_t REG_WHOAMI  = 0x75;
static const uint8_t REG_ACCEL   = 0x3B;

static uint8_t   last_i2c_err = 0;
unsigned long    lastIMUms    = 0;
const unsigned long IMU_DT_MS = 10;   


float pitch_deg_filt = 0.0f;
const float ALPHA_ANGLE = 0.12f;   


const float PITCH_MIN_DEG = -30.0f;  
const float PITCH_MAX_DEG =  60.0f;  


const float TILT_RANGE_SEMITONES = 24.0f;


static int   lastVibLevel = -1;       
static const float vibRates[4] = {0.0f, 2.0f, 4.0f, 6.0f};




static void i2cWrite2(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  last_i2c_err = Wire.endTransmission();
}

static uint8_t i2cRead1(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if ((last_i2c_err = Wire.endTransmission(false)) != 0) return 0xFF;
  if (Wire.requestFrom((int)MPU_ADDR, 1) != 1) { last_i2c_err = 0xFE; return 0xFF; }
  return Wire.read();
}

void i2cScan() {
  Serial.println(F("I2C scan:"));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t e = Wire.endTransmission();
    if (e == 0) {
      Serial.print(F("  found 0x"));
      Serial.println(addr, HEX);
    }
  }
}


void mpuInit() {
  Wire.begin();
  Wire.setClock(400000); 
  delay(30);

  i2cScan();


  i2cWrite2(REG_PWR1, 0x01);  
  delay(10);
  i2cWrite2(REG_SMPLRT, 0x07);  
  i2cWrite2(REG_CONFIG,  0x03);  
  i2cWrite2(REG_GYROCFG, 0x00);  
  i2cWrite2(REG_ACCCFG,  0x00);  
  i2cWrite2(REG_INT_EN,  0x00); 

  uint8_t who = i2cRead1(REG_WHOAMI);
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);
  if (who != 0x68) {
    Serial.println(F("ERROR: MPU not responding at 0x68 (check AD0, wiring, power)."));
  }
}


bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL);
  if ((last_i2c_err = Wire.endTransmission(false)) != 0) {
    Serial.print(F("I2C endTransmission err ")); Serial.println(last_i2c_err);
    return false;
  }
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) {
    Serial.println(F("I2C requestFrom failed (wanted 14 bytes)."));
    return false;
  }
  auto rd16 = []() -> int16_t {
    int hi = Wire.read();
    int lo = Wire.read();
    return (int16_t)((hi << 8) | lo);
  };
  ax = rd16(); ay = rd16(); az = rd16();
  (void)rd16();            
  gx = rd16(); gy = rd16(); gz = rd16();
  return true;
}


void pollIMUAndUpdatePitch() {

  if (millis() - lastIMUms < IMU_DT_MS) return;
  lastIMUms = millis();

  int16_t axr, ayr, azr, gxr, gyr, gzr;
  if (!mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr)) return;


  float ax_g = (float)axr / 16384.0f;
  float ay_g = (float)ayr / 16384.0f;
  float az_g = (float)azr / 16384.0f;

  float pitch_deg = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / PI;

  pitch_deg_filt = pitch_deg_filt + ALPHA_ANGLE * (pitch_deg - pitch_deg_filt);

  if (baseFreq > 0) {

    float norm = (pitch_deg_filt - PITCH_MIN_DEG) / (PITCH_MAX_DEG - PITCH_MIN_DEG);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;


    float semi_cont = norm * TILT_RANGE_SEMITONES;  

    int semi_disc = (int)roundf(semi_cont);
    static int lastSemi = 9999;
    if (semi_disc != lastSemi) {
      lastSemi = semi_disc;

      float newF = (float)baseFreq * powf(2.0f, semi_disc / 12.0f);
      if (newF < 50.0f)   newF = 50.0f;
      if (newF > 4000.0f) newF = 4000.0f;
      curFreq = (int)newF;   
      playNote(curFreq);
    }
  }

  float roll_deg = atan2f(ay_g, az_g) * 180.0f / PI;

  if (roll_deg < 0.0f)  roll_deg = 0.0f;
  if (roll_deg > 90.0f) roll_deg = 90.0f;

  int vibLevel;
  if (roll_deg < 15.0f)        vibLevel = 0;
  else if (roll_deg < 35.0f)   vibLevel = 1; 
  else if (roll_deg < 60.0f)   vibLevel = 2; 
  else                         vibLevel = 3; 

  if (vibLevel != lastVibLevel) {
    lastVibLevel = vibLevel;
    float vibRate = vibRates[vibLevel];
    if (vibRate < 0.1f) {
      stopVibrato();
    } else {
      setVibrato(vibRate);
    }
  }
}
