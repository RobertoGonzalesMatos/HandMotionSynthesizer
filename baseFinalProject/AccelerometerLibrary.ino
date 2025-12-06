#include "DrumLibrary.h"
#include "SoundEngine.h"
#include <Wire.h>
#include <math.h>
#include "notes.h"

// ===== GLOBAL FSM INSTANCE =====
static full_state FS = { 0, 0, false, 0, s_INIT };

// ===== External audio interfaces =====
extern int baseFreq;
extern void playNote(int freq);
extern void initGPT();
extern void initDrumPWM();
extern void teardownDrumPWM();
extern void stopPlay();
extern void stopVibrato();
extern void setVibrato(float vibRateHz);
extern int curFreq;
extern bool drumMode;

static int lastAnnouncedHz = -1;

// ===== MPU CONFIG =====
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t REG_PWR1 = 0x6B;
static const uint8_t REG_SMPLRT = 0x19;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYROCFG = 0x1B;
static const uint8_t REG_ACCCFG = 0x1C;
static const uint8_t REG_INT_EN = 0x38;
static const uint8_t REG_WHOAMI = 0x75;
static const uint8_t REG_ACCEL = 0x3B;

unsigned long lastIMUms = 0;
static uint8_t last_i2c_err = 0;

const unsigned long IMU_DT_MS = 10;

// ===== tilt → semitone mapping =====
const float PITCH_MIN_DEG = -60.0f;
const float PITCH_MAX_DEG = 60.0f;
const float TILT_RANGE_SEMITONES = 24.0f;

static int lastVibLevel = -1;
static const float vibRates[4] = { 0.0f, 2.0f, 4.0f, 6.0f };

// Calibration & transform
const float AX_BIAS = 0, AY_BIAS = 0, AZ_BIAS = 0;
const float AX_SF = 1, AY_SF = 1, AZ_SF = 1;

const float Rm[3][3] = {{ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }};

static inline void applyMount(float ax, float ay, float az, float &ux, float &uy, float &uz) {
  float x = (ax - AX_BIAS) * AX_SF;
  float y = (ay - AY_BIAS) * AY_SF;
  float z = (az - AZ_BIAS) * AZ_SF;

  ux = Rm[0][0] * x + Rm[0][1] * y + Rm[0][2] * z;
  uy = Rm[1][0] * x + Rm[1][1] * y + Rm[1][2] * z;
  uz = Rm[2][0] * x + Rm[2][1] * y + Rm[2][2] * z;
}

// ===== Complementary filter state =====
static float pitch_est = 0.0f;
static float roll_est = 0.0f;
static float yaw_deg = 0.0f;
static float yaw_bias_dps = 0.0f;
static unsigned long last_t_ms = 0;

const float CF_ALPHA = 0.98f;
const float YAW_BIAS_ALPHA = 0.002f;
const float CALM_GZ_DPS = 10.0f;

const float PLAY_BAND_DEG = 15.0f;

// ===== Audio tracking =====
static bool notePlaying = false;
static int targetFreqHz = 0;

// ===== Vibrato Helpers =====
static bool vibActive = false;
static float vibCurrentRateHz = 0;

static inline void vibApply(float rateHz) {
  if (rateHz < 0.1f) {
    if (vibActive) {
      stopVibrato();
      vibActive = false;
      vibCurrentRateHz = 0;
    }
    return;
  }

  float scaled = rateHz * 2.0f;

  if (!vibActive || fabsf(scaled - vibCurrentRateHz) > 0.05f) {
    setVibrato(scaled);
    vibActive = true;
    vibCurrentRateHz = scaled;
  }
}

static inline void vibForceStop() {
  if (vibActive) {
    stopVibrato();
    vibActive = false;
    vibCurrentRateHz = 0;
  }
}

static inline void doStop() {
  if (notePlaying) {
    stopPlay();
    notePlaying = false;
  }
  vibForceStop();
}

// ===== Stubs you can wire to hardware =====
#ifndef BUTTON_PIN
#define BUTTON_PIN 2
#endif

static inline void pet_watchdog() {}
static inline bool readButton() {  // active HIGH by default; tweak if needed
  pinMode(BUTTON_PIN, INPUT);
  return digitalRead(BUTTON_PIN) == HIGH;
}

// ===== IMU helpers =====
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
  if (Wire.requestFrom((int)MPU_ADDR, 1) != 1) {
    last_i2c_err = 0xFE;
    return 0xFF;
  }
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
  i2cWrite2(REG_CONFIG, 0x03);
  i2cWrite2(REG_GYROCFG, 0x00);
  i2cWrite2(REG_ACCCFG, 0x00);
  i2cWrite2(REG_INT_EN, 0x00);

  Serial.println("PmnPFS_b.PSEL");
  Serial.println((int)R_PFS->PORT[1].PIN[5].PmnPFS_b.PSEL);

  uint8_t who = i2cRead1(REG_WHOAMI);
  Serial.print(F("WHO_AM_I = 0x"));
  Serial.println(who, HEX);
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
    Serial.print(F("I2C endTransmission err "));
    Serial.println(last_i2c_err);
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
  ax = rd16();
  ay = rd16();
  az = rd16();
  (void)rd16();  // temperature (ignored)
  gx = rd16();
  gy = rd16();
  gz = rd16();
  return true;
}

// ==========================================
// QUANTIZATION w/ HYSTERESIS
// ==========================================
static int lastSemi = 9999;
static float semiEdgeLow = -1e9f;
static float semiEdgeHigh = 1e9f;

static inline int quantizeWithHys(float semi) {
  if (lastSemi == 9999) {
    lastSemi = roundf(semi);
    semiEdgeLow = lastSemi - 0.6f;
    semiEdgeHigh = lastSemi + 0.6f;
  }

  if (semi < semiEdgeLow) {
    lastSemi--;
    semiEdgeLow = lastSemi - 0.6f;
    semiEdgeHigh = lastSemi + 0.6f;
  } else if (semi > semiEdgeHigh) {
    lastSemi++;
    semiEdgeLow = lastSemi - 0.6f;
    semiEdgeHigh = lastSemi + 0.6f;
  }

  return lastSemi;
}

full_state updateFSM(full_state curr,
                     float xRead, float yRead, float zRead,
                     bool drumModeOn,
                     unsigned long clock) {
  full_state ret = curr;
  bool fiveMs = (clock - curr.savedClock) >= 5;

  switch (curr.state) {
    case s_INIT:
      if (!drumModeOn && xRead > 0 && fabsf(zRead) <= PLAY_BAND_DEG) {
        ret.noteFrequency = xRead;
        vibForceStop();

        curFreq = ret.noteFrequency;
        playNote(curFreq);
        notePlaying = true;

        ret.savedClock = clock;
        ret.state = s_REG_CALC;
        break;
      }

      // DRUM MODE ENTER
      if (drumModeOn && !curr.gestureModeOn) {
        ret.gestureModeOn = true;
        ret.state = s_GESTURE_WAIT;
        ret.savedClock = clock;
        initDrumPWM();
      }
      break;

    // ----------------------------------------------------------
    case s_REG_CALC:
      // DRUM MODE ENTER
      if (drumModeOn && !curr.gestureModeOn) {
        ret.gestureModeOn = true;
        ret.state = s_GESTURE_WAIT;
        ret.savedClock = clock;
        initDrumPWM();
        break;
      }

      if (fiveMs) {
        ret.savedClock = clock;
        ret.state = s_REG_WAIT;
      }
      break;

    // ----------------------------------------------------------
    case s_REG_WAIT:
      if (drumModeOn && !curr.gestureModeOn) {
        ret.gestureModeOn = true;
        ret.state = s_GESTURE_WAIT;
        ret.savedClock = clock;
        initDrumPWM();
        break;
      }

      // silence
      if (fiveMs && (fabsf(zRead) > PLAY_BAND_DEG || xRead <= 0)) {
        doStop();
        ret.state = s_REG_CALC;
        ret.savedClock = clock;
        break;
      }

      // start playing
      if (fiveMs && !notePlaying && xRead > 0 && fabsf(zRead) <= PLAY_BAND_DEG) {
        vibForceStop();
        ret.noteFrequency = xRead;

        curFreq = xRead;
        playNote(curFreq);
        notePlaying = true;

        ret.state = s_REG_CALC;
        ret.savedClock = clock;
        break;
      }

      // vibrato
      if (fiveMs && notePlaying && yRead > 0) {
        vibApply(yRead);
        ret.vibratoLevel = yRead;
        ret.savedClock = clock;
        ret.state = s_REG_CALC;
        break;
      }

      // retune
      if (fiveMs && fabsf(xRead - (float)curr.noteFrequency) > 1.0f) {
        vibForceStop();
        ret.noteFrequency = xRead;

        if (xRead > 0 && fabsf(zRead) <= PLAY_BAND_DEG) {
          curFreq = xRead;
          playNote(curFreq);
          notePlaying = true;
        } else {
          doStop();
        }

        ret.state = s_REG_CALC;
        ret.savedClock = clock;
      }
      break;

    // ----------------------------------------------------------
    case s_GESTURE_WAIT:
      // exit drum mode → regular
      if (!drumModeOn && curr.gestureModeOn) {
        ret.gestureModeOn = false;
        ret.state = s_REG_WAIT;
        ret.savedClock = clock;
        teardownDrumPWM();
        initGPT();
        break;
      }
      Serial.print("XYZ");
      Serial.print(xRead);
      Serial.print(yRead);
      Serial.print(zRead);
      if (fiveMs && drumModeOn) {
        // These are DRUM TRIGGERS ONLY when drumMode is ON
        if (xRead < -25) {
          Kick(clock);
          ret.state = s_GESTURE_CALC;
          ret.savedClock = clock;
          break;
        } else if (xRead > 25) {
          Snare(clock);
          ret.state = s_GESTURE_CALC;
          ret.savedClock = clock;
          break;
        } else if (yRead > 25) {
          Tom(clock);
          ret.state = s_GESTURE_CALC;
          ret.savedClock = clock;
          break;
        } else if (yRead < -25) {
          Hat(clock);
          ret.state = s_GESTURE_CALC;
          ret.savedClock = clock;
          break;
        } else if (zRead > PLAY_BAND_DEG) {
          Ride(clock);
          ret.state = s_GESTURE_CALC;
          ret.savedClock = clock;
          break;
        } else if (zRead < -PLAY_BAND_DEG) {
          Cymbal(clock);
          ret.state = s_GESTURE_CALC;
          ret.savedClock = clock;
          break;
        }
      }
      break;

    // ----------------------------------------------------------
    case s_GESTURE_CALC:
      if (!drumModeOn && curr.gestureModeOn) {
        ret.gestureModeOn = false;
        ret.state = s_REG_WAIT;
        ret.savedClock = clock;
        teardownDrumPWM();
        initGPT();
        break;
      }

      if (fiveMs && drumModeOn) {
        ret.state = s_GESTURE_WAIT;
        ret.savedClock = clock;
      }
      break;
  }

  return ret;
}


// ===== Main polling function: read IMU, update orientation, call FSM =====
// ========================================================
//                IMU POLLING + MAIN SENSOR PIPELINE
// ========================================================
void pollIMUAndUpdatePitch() {

  unsigned long now = millis();
  if (now - lastIMUms < IMU_DT_MS) return;
  lastIMUms = now;

  // -------- 1) Read raw IMU values --------
  int16_t axr, ayr, azr;
  int16_t gxr, gyr, gzr;

  if (!mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr)) return;

  // -------- 2) Convert raw → physical units --------
  float ax_g = (float)axr / 16384.0f;
  float ay_g = (float)ayr / 16384.0f;
  float az_g = (float)azr / 16384.0f;

  float gx_dps = (float)gxr / 131.0f;
  float gy_dps = (float)gyr / 131.0f;
  float gz_dps = (float)gzr / 131.0f;

  // -------- 3) Apply mounting transform --------
  float ux, uy, uz;
  applyMount(ax_g, ay_g, az_g, ux, uy, uz);

  // -------- 4) Compute accel-only angles --------
  float pitch_acc = atan2f(-ux, sqrtf(uy * uy + uz * uz)) * 180.0f / PI;
  float roll_acc = atan2f(uy, uz) * 180.0f / PI;

  // -------- 5) Complementary filter --------
  float dt = (last_t_ms == 0)
               ? (IMU_DT_MS / 1000.0f)
               : (now - last_t_ms) / 1000.0f;
  last_t_ms = now;

  pitch_est = CF_ALPHA * (pitch_est + gy_dps * dt)
              + (1.0f - CF_ALPHA) * pitch_acc;

  roll_est = CF_ALPHA * (roll_est + gx_dps * dt)
             + (1.0f - CF_ALPHA) * roll_acc;

  // -------- 6) Yaw integration w/ bias compensation --------
  if (fabsf(gz_dps) < CALM_GZ_DPS)
    yaw_bias_dps = (1 - YAW_BIAS_ALPHA) * yaw_bias_dps
                   + YAW_BIAS_ALPHA * gz_dps;

  yaw_deg += (gz_dps - yaw_bias_dps) * dt;

  // -------- 7) pitch → frequency (regular mode only) --------
  if (baseFreq > 0 && !drumMode) {

    float norm = (pitch_est - PITCH_MIN_DEG)
                 / (PITCH_MAX_DEG - PITCH_MIN_DEG);

    if (norm < 0) norm = 0;
    if (norm > 1) norm = 1;

    float semi_cont = norm * TILT_RANGE_SEMITONES;
    int semi_disc = quantizeWithHys(semi_cont);

    float f = baseFreq * powf(2.0f, semi_disc / 12.0f);

    if (f < 50) f = 50;
    if (f > 4000) f = 4000;

    targetFreqHz = (int)f;

  } else {
    targetFreqHz = 0;
  }

  // -------- 8) Vibrato from roll angle --------
  float roll_deg = roll_est;
  if (roll_deg < 0) roll_deg = 0;
  if (roll_deg > 90) roll_deg = 90;

  int vibLevel;
  if (roll_deg < 15) vibLevel = 0;
  else if (roll_deg < 35) vibLevel = 1;
  else if (roll_deg < 60) vibLevel = 2;
  else vibLevel = 3;

  if (vibLevel != lastVibLevel)
    lastVibLevel = vibLevel;

  float desiredVibRate = vibRates[lastVibLevel];

  // -------- 9) FSM update (regular or drum mode) --------
  FS = updateFSM(FS,
                 (float)pitch_est,
                 roll_est,
                 yaw_deg,
                 drumMode,
                 now);

  // -------- 10) Recording support --------
  if (isRecording()) {
    int effective = notePlaying ? curFreq : 0;
    recordSample(effective);
  }

  // -------- 11) Serial note output --------
  if (notePlaying) {
    if (curFreq != lastAnnouncedHz) {
      sendNoteToSerial(curFreq);
      lastAnnouncedHz = curFreq;
    }
  } else {
    if (lastAnnouncedHz != 0) {
      Serial.println("NOTE:0");
      lastAnnouncedHz = 0;
    }
  }
}


// ========================================================
//                    MIDI + NOTE UTILITIES
// ========================================================


// static const char* NOTE12[12] =
//   {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

// static inline int hzToMidi(int hz) {
//   if (hz <= 0) return 0;
//   float n = 69.0f + 12.0f * (log((float)hz / 440.0f) / log(2.0f)); // A4=440 -> 69
//   return (int)lroundf(n);
// }

static inline void printHzAndNote(int hz) {
  int midi = hzToMidi(hz);
  const char *name = NOTE12[(midi % 12 + 12) % 12];
  int octave = midi / 12 - 1;
  Serial.print(hz);
  Serial.print(F(" Hz ("));
  Serial.print(name);
  Serial.print(octave);
  Serial.print(')');
}

void sendNoteToSerial(int hz) {
  int midi = hzToMidi(hz);
  const char *name = NOTE12[(midi % 12 + 12) % 12];

  Serial.print("NOTE:");
  Serial.println(name);  // sends it as NOTE:C
}