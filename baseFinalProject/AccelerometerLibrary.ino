
#include <Wire.h>
#include <math.h>

// ===== External audio interface (implemented elsewhere in your project) =====
extern int  baseFreq;
extern void playNote(int freq);
extern void stopPlay();
extern void stopVibrato();
extern void setVibrato(float vibRateHz);
extern int  curFreq;
static int lastAnnouncedHz = -1; 
// ===== MPU-6050 register map and constants =====
static const uint8_t MPU_ADDR    = 0x68;
static const uint8_t REG_PWR1    = 0x6B;
static const uint8_t REG_SMPLRT  = 0x19;
static const uint8_t REG_CONFIG  = 0x1A;
static const uint8_t REG_GYROCFG = 0x1B;
static const uint8_t REG_ACCCFG  = 0x1C;
static const uint8_t REG_INT_EN  = 0x38;
static const uint8_t REG_WHOAMI  = 0x75;
static const uint8_t REG_ACCEL   = 0x3B;

// ===== I2C / polling cadence =====
static uint8_t   last_i2c_err = 0;
unsigned long    lastIMUms    = 0;
const unsigned long IMU_DT_MS = 10;  // IMU update ~100 Hz

// ===== Mapping & ranges =====
// Map pitch angle to a ± semitone span around baseFreq
const float PITCH_MIN_DEG = -30.0f;          // Tilt back limit (maps to 0 semitones)
const float PITCH_MAX_DEG =  60.0f;          // Tilt forward limit (maps to TILT_RANGE_SEMITONES)
const float TILT_RANGE_SEMITONES = 24.0f;    // 2 octaves total range

// Vibrato buckets by ROLL angle (0=off, then ~2,4,6 Hz)
static int   lastVibLevel = -1;
static const float vibRates[4] = {0.0f, 2.0f, 4.0f, 6.0f};

// Optional accel calibration & mounting transform (identity defaults)
const float AX_BIAS=0.0f, AY_BIAS=0.0f, AZ_BIAS=0.0f;
const float AX_SF  =1.0f, AY_SF  =1.0f, AZ_SF  =1.0f;

// Rm rotates raw accel to "instrument" axes if your board is mounted oddly.
// Default is identity (no rotation). Fill this with your mount calibration if needed.
const float Rm[3][3] = {
  { 1.0f,  0.0f,  0.0f},
  { 0.0f,  1.0f,  0.0f},
  { 0.0f,  0.0f,  1.0f}
};

// Apply bias/scale and rotate by Rm to get unified axes (ux,uy,uz).
static inline void applyMount(float ax, float ay, float az,
                              float &ux, float &uy, float &uz) {
  float x = (ax - AX_BIAS) * AX_SF;
  float y = (ay - AY_BIAS) * AY_SF;
  float z = (az - AZ_BIAS) * AZ_SF;
  ux = Rm[0][0]*x + Rm[0][1]*y + Rm[0][2]*z;
  uy = Rm[1][0]*x + Rm[1][1]*y + Rm[1][2]*z;
  uz = Rm[2][0]*x + Rm[2][1]*y + Rm[2][2]*z;
}

// ===== Orientation estimation (complementary filter) =====
// We fuse accel (for long-term reference) and gyro (for short-term dynamics).
static float pitch_est = 0.0f;  // deg, forward/back
static float roll_est  = 0.0f;  // deg, left/right
const  float CF_ALPHA  = 0.98f; // 98% gyro, 2% accel (typical)
static unsigned long last_t_ms = 0;

// Yaw is integrated gyro Z with a slow bias adaptation when the device is "calm".
static float yaw_deg = 0.0f;              // deg, 0 points "forward"
static float yaw_bias_dps = 0.0f;         // learned gyro Z bias (deg/s)
const  float YAW_BIAS_ALPHA = 0.002f;     // small adaptation step
const  float CALM_GZ_DPS    = 10.0f;      // consider calm if |gz| < this

// We only allow playing when yaw is near forward to act as a "gate".
const  float PLAY_BAND_DEG  = 15.0f;      // ± yaw window

// ===== Audio state =====
static bool  notePlaying  = false;        // whether oscillator is running
static int   targetFreqHz = 0;            // latest desired frequency

// ===== Vibrato state machine (fix for "stuck vibrato") =====
// We gate vibrato by note state, and stop it before re-tuning to a new freq.
static bool  vibActive        = false;
static float vibCurrentRateHz = 0.0f;

// Apply or stop vibrato based on requested rate; avoids repeated reconfigure spam.
static inline void vibApply(float rateHz) {
  if (rateHz < 0.1f) {
    if (vibActive) {
      stopVibrato();
      vibActive = false;
      vibCurrentRateHz = 0.0f;
    }
  } else {
    if (!vibActive || fabsf(rateHz - vibCurrentRateHz) > 0.05f) {
      setVibrato(rateHz);
      vibActive = true;
      vibCurrentRateHz = rateHz;
    }
  }
}

// Hard stop vibrato right now (used before re-tuning or when silencing).
static inline void vibForceStop() {
  if (vibActive) {
    stopVibrato();
    vibActive = false;
    vibCurrentRateHz = 0.0f;
  }
}

// ===== I2C helpers =====
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

// Simple I2C bus scan (handy during bring-up; can be commented out later).
void i2cScan() {
  Serial.println(F("I2C scan:"));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t e = Wire.endTransmission();
    if (e == 0) { Serial.print(F("  found 0x")); Serial.println(addr, HEX); }
  }
}

// ===== IMU initialization =====
void mpuInit() {
  Wire.begin();
  Wire.setClock(400000);     // Fast-mode I2C
  delay(30);

  i2cScan();                 // Optional debug

  // Wake up & basic configuration
  i2cWrite2(REG_PWR1,    0x01); // Clock = PLL X, sleep off
  delay(10);
  i2cWrite2(REG_SMPLRT,  0x07); // Sample rate divider
  i2cWrite2(REG_CONFIG,  0x03); // DLPF ~44 Hz
  i2cWrite2(REG_GYROCFG, 0x00); // ±250 dps
  i2cWrite2(REG_ACCCFG,  0x00); // ±2 g
  i2cWrite2(REG_INT_EN,  0x00); // No interrupts

  // Verify device presence
  uint8_t who = i2cRead1(REG_WHOAMI);
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);
  if (who != 0x68) {
    Serial.println(F("ERROR: MPU not responding at 0x68 (check AD0, wiring, power)."));
  }

  yaw_deg = 0.0f;           // Reset yaw on init
}

// ===== Raw IMU read (14 bytes: accel(6), temp(2), gyro(6)) =====
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
  (void)rd16();             // temperature (ignored)
  gx = rd16(); gy = rd16(); gz = rd16();
  return true;
}

// ===== Semitone quantization with hysteresis =====
// This avoids “chatter” when the continuous semitone value hovers near a boundary.
static int   lastSemi = 9999;
static float semiEdgeLow  = -1e9f;
static float semiEdgeHigh =  1e9f;

static inline int quantizeWithHys(float semi_cont) {
  if (lastSemi == 9999) { // first run: seed with current value
    lastSemi = (int)roundf(semi_cont);
    semiEdgeLow  = lastSemi - 0.6f;   // hysteresis band ~0.8 semitone wide
    semiEdgeHigh = lastSemi + 0.6f;
  }
  if (semi_cont < semiEdgeLow) {
    lastSemi--;
    semiEdgeLow  = lastSemi - 0.6f;
    semiEdgeHigh = lastSemi + 0.6f;
  } else if (semi_cont > semiEdgeHigh) {
    lastSemi++;
    semiEdgeLow  = lastSemi - 0.6f;
    semiEdgeHigh = lastSemi + 0.6f;
  }
  return lastSemi;
}

// ===== Main polling function: read IMU, update orientation, decide audio =====
void pollIMUAndUpdatePitch() {
  unsigned long now = millis();
  if (now - lastIMUms < IMU_DT_MS) return; // Run at ~100 Hz
  lastIMUms = now;

  // 1) Read raw IMU
  int16_t axr, ayr, azr, gxr, gyr, gzr;
  if (!mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr)) return;

  // Convert raw to physical units (LSB scale from datasheet)
  float ax_g = (float)axr / 16384.0f;
  float ay_g = (float)ayr / 16384.0f;
  float az_g = (float)azr / 16384.0f;
  float gx_dps = (float)gxr / 131.0f;   // deg/s
  float gy_dps = (float)gyr / 131.0f;
  float gz_dps = (float)gzr / 131.0f;

  // 2) Apply mounting transform for consistent axes
  float ux, uy, uz;
  applyMount(ax_g, ay_g, az_g, ux, uy, uz);

  // 3) Compute accel-only angles (reference)
  float pitch_acc = atan2f(-ux, sqrtf(uy*uy + uz*uz)) * 180.0f / PI;
  float roll_acc  = atan2f( uy, uz ) * 180.0f / PI;

  // 4) Complementary filter fuse gyro+accel
  float dt = (last_t_ms == 0) ? (IMU_DT_MS / 1000.0f) : (now - last_t_ms) / 1000.0f;
  last_t_ms = now;

  pitch_est = CF_ALPHA * (pitch_est + gy_dps * dt) + (1.0f - CF_ALPHA) * pitch_acc;
  roll_est  = CF_ALPHA * (roll_est  + gx_dps * dt) + (1.0f - CF_ALPHA) * roll_acc;

  // 5) Yaw integrate with slow bias adaptation when calm
  if (fabsf(gz_dps) < CALM_GZ_DPS) {
    yaw_bias_dps = (1.0f - YAW_BIAS_ALPHA) * yaw_bias_dps + YAW_BIAS_ALPHA * gz_dps;
  }
  yaw_deg += (gz_dps - yaw_bias_dps) * dt;

  // 6) Map pitch to a target frequency (via semitone mapping) if baseFreq > 0
  if (baseFreq > 0) {
    float norm = (pitch_est - PITCH_MIN_DEG) / (PITCH_MAX_DEG - PITCH_MIN_DEG);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    float semi_cont = norm * TILT_RANGE_SEMITONES;
    int   semi_disc = quantizeWithHys(semi_cont);

    float f = (float)baseFreq * powf(2.0f, semi_disc / 12.0f);
    if (f < 50.0f)   f = 50.0f;       // Safety clamp for audio backends
    if (f > 4000.0f) f = 4000.0f;
    targetFreqHz = (int)f;
  } else {
    targetFreqHz = 0; // base muted: do not play
  }

  // 7) Compute vibrato bucket from roll
  float roll_deg = roll_est;
  if (roll_deg < 0.0f)  roll_deg = 0.0f;
  if (roll_deg > 90.0f) roll_deg = 90.0f;

  int vibLevel;
  if (roll_deg < 15.0f)        vibLevel = 0;  // off
  else if (roll_deg < 35.0f)   vibLevel = 1;  // ~2 Hz
  else if (roll_deg < 60.0f)   vibLevel = 2;  // ~4 Hz
  else                         vibLevel = 3;  // ~6 Hz

  // Only remember desired level; actual apply happens after gate decision
  if (vibLevel != lastVibLevel) {
    lastVibLevel = vibLevel;
  }

  // 8) Gate note by yaw and availability of a valid target frequency
  bool shouldPlay = (fabsf(yaw_deg) <= PLAY_BAND_DEG) && (targetFreqHz > 0);

  // Pick desired vibrato rate based on latest bucket
  float desiredVibRate = vibRates[lastVibLevel < 0 ? 0 : lastVibLevel];

  if (shouldPlay) {
    // If starting a note or changing pitch, kill any running vibrato first.
    // This avoids the vibrato LFO fighting the re-tune and getting “stuck”.
    if (!notePlaying || curFreq != targetFreqHz) {
      vibForceStop();              // <— key line: stop LFO before re-tune
      curFreq = targetFreqHz;
      
      playNote(curFreq);
      notePlaying = true;
    }
    // With note running, (re)apply appropriate vibrato
    vibApply(desiredVibRate);
  } else {
    // Silence path: stop oscillator and ensure vibrato is off
    if (notePlaying) {
      stopPlay();
      notePlaying = false;
    }
    vibForceStop();
  }
if (targetFreqHz != lastAnnouncedHz) {
  Serial.print(F("[note] -> "));
  printHzAndNote(targetFreqHz);
  Serial.println();
  lastAnnouncedHz = targetFreqHz;
}
}
// ----- Hz -> musical note name (C, C#, ..., B) with octave -----
static const char* NOTE12[12] =
  {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

static inline int hzToMidi(int hz) {
  if (hz <= 0) return 0;
  float n = 69.0f + 12.0f * (log((float)hz / 440.0f) / log(2.0f)); // A4=440 -> 69
  return (int)lroundf(n);
}

static inline void printHzAndNote(int hz) {
  int midi = hzToMidi(hz);
  const char* name = NOTE12[(midi % 12 + 12) % 12];
  int octave = midi / 12 - 1;
  Serial.print(hz);
  Serial.print(F(" Hz ("));
  Serial.print(name);
  Serial.print(octave);
  Serial.print(')');
}
