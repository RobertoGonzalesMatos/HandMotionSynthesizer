#include "SoundEngine.h"
#include <Wire.h>
#include <math.h>

// ===== GLOBAL FSM INSTANCE =====
// Start with 0 so first valid targetFreqHz will trigger a play.
static full_state FS = {0, 0, false, 0, s_INIT};

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
// Same as working version
const float PITCH_MIN_DEG = -60.0f;
const float PITCH_MAX_DEG =  60.0f;
const float TILT_RANGE_SEMITONES = 24.0f;

// Vibrato buckets by ROLL angle (0=off, then ~2,4,6 Hz)
static int   lastVibLevel = -1;
static const float vibRates[4] = {0.0f, 2.0f, 4.0f, 6.0f};

// Optional accel calibration & mounting transform (identity defaults)
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

// ===== Orientation estimation (complementary filter) =====
static float pitch_est = 0.0f;  // deg, forward/back
static float roll_est  = 0.0f;  // deg, left/right
const  float CF_ALPHA  = 0.98f;
static unsigned long last_t_ms = 0;

// Yaw is integrated gyro Z with a slow bias adaptation when the device is "calm".
static float yaw_deg = 0.0f;
static float yaw_bias_dps = 0.0f;
const  float YAW_BIAS_ALPHA = 0.002f;
const  float CALM_GZ_DPS    = 10.0f;

// We only allow playing when yaw is near forward to act as a "gate".
const  float PLAY_BAND_DEG  = 15.0f;      // ± yaw window, matches working code

// ===== Audio state =====
static bool  notePlaying  = false;
static int   targetFreqHz = 0;

// ===== Vibrato helpers =====
static bool  vibActive        = false;
static float vibCurrentRateHz = 0.0f;

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

static inline void vibForceStop() {
  if (vibActive) {
    stopVibrato();
    vibActive = false;
    vibCurrentRateHz = 0.0f;
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

static inline void pet_watchdog() { }
static inline bool readButton() {   // active HIGH by default; tweak if needed
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
  (void)rd16();             // temperature (ignored)
  gx = rd16(); gy = rd16(); gz = rd16();
  return true;
}

// ===== Semitone quantization with hysteresis =====
// Updated to match your working file: ±0.6 semitone band
static int   lastSemi = 9999;
static float semiEdgeLow  = -1e9f;
static float semiEdgeHigh =  1e9f;

static inline int quantizeWithHys(float semi_cont) {
  if (lastSemi == 9999) { // first run: seed with current value
    lastSemi = (int)roundf(semi_cont);
    semiEdgeLow  = lastSemi - 0.6f;   // hysteresis band ~1.2 semitones
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

// =====================================================
// ============= DRUM / GESTURE SUPPORT =================
// =====================================================

// Simple "drum" base frequencies
static const int KICK_FREQ   = 60;    // low
static const int SNARE_FREQ  = 220;   // snappy
static const int TOM_FREQ    = 140;   // mid-low
static const int HAT_FREQ    = 800;   // bright
static const int RIDE_FREQ   = 450;   // metallic-ish?
static const int CYMBAL_FREQ = 1200;  // rings

// === ADSR envelope model for drums ===

struct DrumEnvelope {
  float attackMs;   // A: time from 0 -> 1
  float decayMs;    // D: time from 1 -> sustain
  float sustainMs;  // S: hold at sustainLevel
  float releaseMs;  // R: time from sustainLevel -> 0
  float sustainLevel; // sustain amplitude (0..1)
};

struct DrumState {
  bool          active;
  int           freq;
  unsigned long startMs;
  DrumEnvelope  env;
};

// ADSR setting, might need to tweak
static const DrumEnvelope KICK_ENV   = { 5.0f, 40.0f,  30.0f,  60.0f, 0.3f };
static const DrumEnvelope SNARE_ENV  = { 2.0f, 30.0f,  20.0f,  80.0f, 0.2f };
static const DrumEnvelope TOM_ENV    = { 3.0f, 40.0f,  40.0f,  80.0f, 0.35f };
static const DrumEnvelope HAT_ENV    = { 1.0f, 10.0f,  20.0f,  40.0f, 0.15f };
static const DrumEnvelope RIDE_ENV   = { 3.0f, 50.0f, 120.0f, 150.0f, 0.4f };
static const DrumEnvelope CYMBAL_ENV = { 2.0f, 60.0f, 150.0f, 200.0f, 0.4f };

static DrumState drum = { false, 0, 0, {0,0,0,0,0} };

static inline void setDrumLevel(float level) {
  // Control volume
  //   - PWM duty cycle
  //   - DAC amplitude
  //   - your own "setVolume(level)" function.
  (void)level;
}

// Compute ADSR level at time t (ms since note start)
static float evalEnvelope(const DrumEnvelope &env, float tMs) {
  float A = env.attackMs;
  float D = env.decayMs;
  float S = env.sustainMs;
  float R = env.releaseMs;
  float sL = env.sustainLevel;

  if (tMs < 0.0f) tMs = 0.0f;

  // Attack: 0 -> 1
  if (tMs < A) {
    if (A <= 0.0f) return 1.0f;
    return tMs / A;
  }
  tMs -= A;

  // Decay: 1 -> sustainLevel
  if (tMs < D) {
    if (D <= 0.0f) return sL;
    float x = tMs / D;
    return 1.0f + (sL - 1.0f) * x;  // linear
  }
  tMs -= D;

  // Sustain: hold sustainLevel
  if (tMs < S) {
    return sL;
  }
  tMs -= S;

  // Release: sustainLevel -> 0
  if (tMs < R) {
    if (R <= 0.0f) return 0.0f;
    float x = tMs / R;
    return sL * (1.0f - x);  // linear fade out
  }

  return 0.0f;
}

// Start a drum hit with ADSR, non-blocking
static inline void playDrumADSR(int freq, const DrumEnvelope &env, unsigned long now) {
  doStop();

  drum.active  = true;
  drum.freq    = freq;
  drum.startMs = now;
  drum.env     = env;

  setDrumLevel(0.0f);
  playNote(freq);
}

// Called regularly to evolve the envelope and stop when done
static inline void pollDrum(unsigned long now) {
  if (!drum.active) return;

  float tMs = (float)(now - drum.startMs);
  float level = evalEnvelope(drum.env, tMs);

  if (level <= 0.001f) {
    // Envelope finished: stop the sound
    stopPlay();
    drum.active = false;
    setDrumLevel(0.0f);
    return;
  }

  // Update amplitude here (once you have real volume control)
  setDrumLevel(level);
}

// === Gesture functions: call these from the FSM ===
static inline void Kick(unsigned long now) {
  Serial.println(F("[DRUM] Kick"));
  playDrumADSR(KICK_FREQ, KICK_ENV, now);
}

static inline void Snare(unsigned long now) {
  Serial.println(F("[DRUM] Snare"));
  playDrumADSR(SNARE_FREQ, SNARE_ENV, now);
}

static inline void Tom(unsigned long now) {
  Serial.println(F("[DRUM] Tom"));
  playDrumADSR(TOM_FREQ, TOM_ENV, now);
}

static inline void Hat(unsigned long now) {
  Serial.println(F("[DRUM] Hat"));
  playDrumADSR(HAT_FREQ, HAT_ENV, now);
}

static inline void Ride(unsigned long now) {
  Serial.println(F("[DRUM] Ride"));
  playDrumADSR(RIDE_FREQ, RIDE_ENV, now);
}

static inline void Cymbal(unsigned long now) {
  Serial.println(F("[DRUM] Cymbal"));
  playDrumADSR(CYMBAL_FREQ, CYMBAL_ENV, now);
}


// ===== updateFSM (same style as your snake code) =====
// xRead = targetFreqHz, yRead = vibrato Hz, zRead = yaw_deg
full_state updateFSM(full_state currState,
                     float xRead, float yRead, float zRead,
                     bool buttonOn, unsigned long clock) {
  full_state ret = currState;
  bool fiveMs = (clock - currState.savedClock) >= 5;

  switch (currState.state) {
    case s_INIT:
      // Start playing once we have a valid freq AND yaw is in the play band
      if (xRead > 0.0f && fabsf(zRead) <= PLAY_BAND_DEG) {
        Serial.println(F("t 1–2: init → reg_calc (start note)"));
        ret.noteFrequency = (unsigned long)xRead;

        vibForceStop();
        curFreq = (int)ret.noteFrequency;
        playNote(curFreq);
        notePlaying = true;

        ret.savedClock = clock;
        ret.state = s_REG_CALC;
      }
      break;

    case s_REG_CALC:
      // 2-3: (clock - savedClock) >= 5 ∧ ¬buttonOn
      if (fiveMs && !buttonOn) {
        Serial.println(F("t 2–3: reg_calc → reg_wait"));
        pet_watchdog();
        ret.savedClock = clock;
        ret.state = s_REG_WAIT;
      }
      break;

    case s_REG_WAIT:
      // Silence condition: yaw out of band OR no valid freq → stop.
      if (fiveMs && !buttonOn &&
          (fabsf(zRead) > PLAY_BAND_DEG || xRead <= 0.0f)) {
        Serial.println(F("t 3–2a: stop() (yaw out-of-band or no freq)"));
        doStop();
        ret.savedClock = clock;
        ret.state = s_REG_CALC;
        break;
      }

      // Start playing if we are silent, have a freq, and yaw is in band
      if (fiveMs && !buttonOn && !notePlaying &&
          xRead > 0.0f && fabsf(zRead) <= PLAY_BAND_DEG) {
        Serial.println(F("t 3–2: start note (was silent)"));
        vibForceStop();
        ret.noteFrequency = (unsigned long)xRead;
        curFreq = (int)ret.noteFrequency;
        playNote(curFreq);
        notePlaying = true;
        ret.savedClock = clock;
        ret.state = s_REG_CALC;
        break;
      }

      // Apply vibrato if we are playing and yRead > 0
      if (fiveMs && !buttonOn && notePlaying && yRead > 0.0f) {
        Serial.println(F("t 3–2b: vibrato()"));
        vibApply(yRead);
        ret.vibratoLevel = (unsigned long)yRead;
        pet_watchdog();
        ret.savedClock = clock;
        ret.state = s_REG_CALC;
        break;
      }

      // Retune if frequency changed significantly
      if (fiveMs && !buttonOn &&
          fabsf(xRead - (float)currState.noteFrequency) > 1.0f) {
        Serial.println(F("t 3–2c: playNote(retune)"));
        vibForceStop();
        ret.noteFrequency = (unsigned long)xRead;

        if (ret.noteFrequency > 0 && fabsf(zRead) <= PLAY_BAND_DEG) {
          curFreq = (int)ret.noteFrequency;
          playNote(curFreq);
          notePlaying = true;
        } else {
          // If freq is invalid or yaw is out of band, we go silent
          doStop();
        }

        ret.savedClock = clock;
        ret.state = s_REG_CALC;
        break;
      }

      // 3-4: toggle to Drum Mode
      if (buttonOn && !currState.gestureModeOn) {
        Serial.println(F("t 3–4: You are in Drum Mode!"));
        ret.gestureModeOn = true;
        ret.state = s_GESTURE_WAIT;
      }
      break;

    case s_GESTURE_WAIT:
      // 4-3: toggle back to Regular Mode
      if (buttonOn && currState.gestureModeOn) {
        Serial.println(F("t 4–3: You are in Regular Mode!"));
        ret.gestureModeOn = false;
        ret.state = s_REG_WAIT;
        break;
      }
      // 4-5 (a..f): gesture hits
      if (fiveMs && !buttonOn) {
          // X axis: DOWN  -> Kick
          if (xRead < -25.0f) {
              Serial.println(F("Kick!"));
              Kick(clock);
              ret.state = s_GESTURE_CALC;
              ret.savedClock = clock;
              break;
          }

          // X axis: UP -> Snare 
          if (xRead >  25.0f) {
              Serial.println(F("Snare!"));
              Snare(clock);
              ret.state = s_GESTURE_CALC;
              ret.savedClock = clock;
              break;
          }

          // Y axis: RIGHT -> Tom
          if (yRead >  25.0f) {
              Serial.println(F("Tom!"));
              Tom(clock);
              ret.state = s_GESTURE_CALC;
              ret.savedClock = clock;
              break;
          }

          // Y axis: LEFT -> Hi-hat
          if (yRead < -25.0f) {
              Serial.println(F("Hat!"));
              Hat(clock);
              ret.state = s_GESTURE_CALC;
              ret.savedClock = clock;
              break;
          }

          // Z axis: RIGHT -> Ride
          if (zRead >  25.0f) {
              Serial.println(F("Ride!"));
              Ride(clock);
              ret.state = s_GESTURE_CALC;
              ret.savedClock = clock;
              break;
          }

          // Z axis: LEFT -> Crash/Cymbal
          if (zRead < -25.0f) {
              Serial.println(F("Cymbal!"));
              Cymbal(clock);
              ret.state = s_GESTURE_CALC;
              ret.savedClock = clock;
              break;
          }
      }
      break;