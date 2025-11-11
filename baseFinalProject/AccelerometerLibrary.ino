#include <Wire.h>
#include <math.h>

extern int  baseFreq;               
extern void playNote(int freq);     
extern void stopPlay();              
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

const float PITCH_MIN_DEG = -30.0f;
const float PITCH_MAX_DEG =  60.0f;
const float TILT_RANGE_SEMITONES = 24.0f;

static int   lastVibLevel = -1;
static const float vibRates[4] = {0.0f, 2.0f, 4.0f, 6.0f};

const float AX_BIAS=0.0f, AY_BIAS=0.0f, AZ_BIAS=0.0f;
const float AX_SF  =1.0f, AY_SF  =1.0f, AZ_SF  =1.0f;

const float Rm[3][3] = {
  { 1.0f,  0.0f,  0.0f},
  { 0.0f,  1.0f,  0.0f},
  { 0.0f,  0.0f,  1.0f}
};

static inline void applyMount(float ax, float ay, float az,
                              float &ux, float &uy, float &uz) {
  float x = (ax - AX_BIAS) * AX_SF;
  float y = (ay - AY_BIAS) * AY_SF;
  float z = (az - AZ_BIAS) * AZ_SF;
  ux = Rm[0][0]*x + Rm[0][1]*y + Rm[0][2]*z;
  uy = Rm[1][0]*x + Rm[1][1]*y + Rm[1][2]*z;
  uz = Rm[2][0]*x + Rm[2][1]*y + Rm[2][2]*z;
}

static float pitch_est = 0.0f;  
static float roll_est  = 0.0f;  
const  float CF_ALPHA  = 0.98f;
static unsigned long last_t_ms = 0;

static float yaw_deg = 0.0f;            
static float yaw_bias_dps = 0.0f;      
const  float YAW_BIAS_ALPHA = 0.002f;   
const  float CALM_GZ_DPS    = 10.0f;   

const  float PLAY_BAND_DEG  = 15.0f;   

static bool  notePlaying  = false;
static int   targetFreqHz = 0;         

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
    if (e == 0) { Serial.print(F("  found 0x")); Serial.println(addr, HEX); }
  }
}

void mpuInit() {
  Wire.begin();
  Wire.setClock(400000);
  delay(30);

  i2cScan();

  i2cWrite2(REG_PWR1,    0x01);
  delay(10);
  i2cWrite2(REG_SMPLRT,  0x07);
  i2cWrite2(REG_CONFIG,  0x03);
  i2cWrite2(REG_GYROCFG, 0x00); 
  i2cWrite2(REG_ACCCFG,  0x00); 
  i2cWrite2(REG_INT_EN,  0x00);

  uint8_t who = i2cRead1(REG_WHOAMI);
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);
  if (who != 0x68) {
    Serial.println(F("ERROR: MPU not responding at 0x68 (check AD0, wiring, power)."));
  }

  yaw_deg = 0.0f;
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


static int   lastSemi = 9999;
static float semiEdgeLow  = -1e9f;
static float semiEdgeHigh =  1e9f;
static inline int quantizeWithHys(float semi_cont) {
  if (lastSemi == 9999) {
    lastSemi = (int)roundf(semi_cont);
    semiEdgeLow  = lastSemi - 0.4f;
    semiEdgeHigh = lastSemi + 0.4f;
  }
  if (semi_cont < semiEdgeLow) {
    lastSemi--; semiEdgeLow  = lastSemi - 0.4f; semiEdgeHigh = lastSemi + 0.4f;
  } else if (semi_cont > semiEdgeHigh) {
    lastSemi++; semiEdgeLow  = lastSemi - 0.4f; semiEdgeHigh = lastSemi + 0.4f;
  }
  return lastSemi;
}

void pollIMUAndUpdatePitch() {
  unsigned long now = millis();
  if (now - lastIMUms < IMU_DT_MS) return;
  lastIMUms = now;

  int16_t axr, ayr, azr, gxr, gyr, gzr;
  if (!mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr)) return;

  float ax_g = (float)axr / 16384.0f;
  float ay_g = (float)ayr / 16384.0f;
  float az_g = (float)azr / 16384.0f;
  float gx_dps = (float)gxr / 131.0f;  
  float gy_dps = (float)gyr / 131.0f;
  float gz_dps = (float)gzr / 131.0f;

  float ux, uy, uz;
  applyMount(ax_g, ay_g, az_g, ux, uy, uz);


  float pitch_acc = atan2f(-ux, sqrtf(uy*uy + uz*uz)) * 180.0f / PI;
  float roll_acc  = atan2f( uy, uz ) * 180.0f / PI;                  

  float dt = (last_t_ms == 0) ? (IMU_DT_MS / 1000.0f) : (now - last_t_ms) / 1000.0f;
  last_t_ms = now;
  pitch_est = CF_ALPHA * (pitch_est + gy_dps * dt) + (1.0f - CF_ALPHA) * pitch_acc;
  roll_est  = CF_ALPHA * (roll_est  + gx_dps * dt) + (1.0f - CF_ALPHA) * roll_acc;

  if (fabsf(gz_dps) < CALM_GZ_DPS) {
    yaw_bias_dps = (1.0f - YAW_BIAS_ALPHA) * yaw_bias_dps + YAW_BIAS_ALPHA * gz_dps;
  }
  yaw_deg += (gz_dps - yaw_bias_dps) * dt;

  if (baseFreq > 0) {
    float norm = (pitch_est - PITCH_MIN_DEG) / (PITCH_MAX_DEG - PITCH_MIN_DEG);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    float semi_cont = norm * TILT_RANGE_SEMITONES;
    int   semi_disc = quantizeWithHys(semi_cont);

    float f = (float)baseFreq * powf(2.0f, semi_disc / 12.0f);
    if (f < 50.0f)   f = 50.0f;
    if (f > 4000.0f) f = 4000.0f;
    targetFreqHz = (int)f;
  } else {
    targetFreqHz = 0;
  }

  float roll_deg = roll_est;
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
    if (vibRate < 0.1f) stopVibrato(); else setVibrato(vibRate);
  }

  bool shouldPlay = (fabsf(yaw_deg) <= PLAY_BAND_DEG) && (targetFreqHz > 0);

  if (shouldPlay) {
    if (!notePlaying || curFreq != targetFreqHz) {
      curFreq = targetFreqHz;
      playNote(curFreq);
      notePlaying = true;
    }
  } else {
    if (notePlaying) {
      stopPlay();
      notePlaying = false;
    }
  }
}
