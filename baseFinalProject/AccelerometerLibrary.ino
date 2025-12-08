#include "SoundEngine.h"
#include <Wire.h>
#include <math.h>
#include "notes.h"


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
extern bool drumMode;


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
const  float PLAY_BAND_DEG  = 15.0f;      // ± yaw window

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
      setVibrato(rateHz*2);
      vibActive = true;
      vibCurrentRateHz = rateHz*2;
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

#ifndef TESTING // real stop function
static inline void doStop() {
  if (notePlaying) {
    stopPlay();
    notePlaying = false;
  }
  vibForceStop();
}
#else // mock stop function
static inline void doStop() {
  mockFunc = "doStop()";
}
#endif


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

full_state updateFSM(full_state currState,
                     float ax_g, float ay_g, float az_g,
                     float gx_dps, float gy_dps, float gz_dps,
                     unsigned long clock)
{
  
    full_state ret = currState;
    bool fiveMs = (clock - currState.savedClock) >= 5;

    // ===== TIME DELTA FOR COMPLEMENTARY FILTER =====
    float dt;
    if (currState.last_t_ms == 0)
        dt = IMU_DT_MS / 1000.0f;
    else
        dt = (clock - currState.last_t_ms) / 1000.0f;
    ret.last_t_ms = clock;

    // ===== REGULAR (NON-GESTURE) MODE COMPUTATION =====
    if (!drumMode)
    {
        //
        // ---- 1. MOUNTING TRANSFORM ----
        //
        float ux, uy, uz;
        computeMountTransform(ax_g, ay_g, az_g, ux, uy, uz);

        //
        // ---- 2. ACCEL ANGLES ----
        //
        float pitch_acc, roll_acc;
        computeAccelAngles(ux, uy, uz, pitch_acc, roll_acc);

        //
        // ---- 3. COMPLEMENTARY FILTER ----
        //
        applyComplementaryFilter(gx_dps, gy_dps,
                                 pitch_acc, roll_acc,
                                 dt,
                                 ret.pitch_est, ret.roll_est);

        //
        // ---- 4. YAW UPDATE ----
        //
        updateYaw(gz_dps, dt,
                  ret.yaw_bias_dps,
                  ret.yaw_deg);

        //
        // ---- 5. PITCH → FREQ ----
        //
        ret.noteFrequency = computeTargetFreq(ret.pitch_est);

        //
        // ---- 6. COMPUTE VIBRATO BUCKET ----
        //
        int vibLevel;
        float vibRateHz;
        computeVibratoBucket(ret.roll_est, vibLevel, vibRateHz);
        ret.vibratoLevel = vibLevel; 
        float yRead = vibRateHz;

        //
        // These values now replace xRead, yRead, zRead
        //
        float xRead = (float)ret.noteFrequency;
        float zRead = ret.yaw_deg;
        bool  buttonOn = readButton();
    }

    // ===== GESTURE MODE (DRUM MODE) COMPUTATION =====
    if (drumMode)
    {
        float gx, gy, gz;
        computeGestureAxes(ax_g, ay_g, az_g, gx, gy, gz);

        // For gesture mode, we temporarily store them here:
        ret.noteFrequency = 0;       // no pitched note
        ret.vibratoLevel = 0;
                updateYaw(gz_dps, dt,
                  ret.yaw_bias_dps,
                  ret.yaw_deg);
        // re-map placeholders:
        // xRead ≡ gx; yRead ≡ gy; zRead ≡ gz for gesture comparisons.
    }

    //
    // -------------------------------------------------------------------------
    //                           FINITE STATE MACHINE
    // -------------------------------------------------------------------------
    //

    #ifdef TESTING // set xRead, yRead, zRead from testing inputs
    ret.noteFrequency = ax_g;
    ret.vibratoLevel = ay_g;
    ret.yaw_deg = az_g;
    #endif

    switch (currState.state)
    {
        case s_INIT:
        {
            float xRead = ret.noteFrequency;
            float zRead = ret.yaw_deg;

            if (xRead > 0.0f && fabsf(zRead) <= PLAY_BAND_DEG)
            {
                Serial.println(F("t 1–2: init → reg_calc (start note)"));
                vibForceStop();
                curFreq = (int)xRead;
                playNote(curFreq);
                notePlaying = true;

                ret.savedClock = clock;
                ret.state = s_REG_CALC;
            }
            break;
        }

        case s_REG_CALC:
        {
            if (fiveMs && !drumMode)
            {
                Serial.println(F("t 2–3: reg_calc → reg_wait"));
                pet_watchdog();
                ret.savedClock = clock;
                ret.state = s_REG_WAIT;
            }
            break;
        }

        case s_REG_WAIT:
        {
            float xRead = (float)ret.noteFrequency;
            float zRead = ret.yaw_deg;
            float yRead = vibRates[ret.vibratoLevel];

            // ---- Silence condition ----
            if (fiveMs && !drumMode &&
                (fabsf(zRead) > PLAY_BAND_DEG || xRead <= 0.0f))
            {
                Serial.println(F("t 3–2a: stop() (yaw out-of-band or no freq)"));

                if (xRead <= 0.0f)
                    Serial.println(F("t 3–2a: stop() (no freq)"));

                Serial.println("NOTE:0");
                doStop();

                ret.savedClock = clock;
                ret.state = s_REG_CALC;
                break;
            }

            // ---- Start note if silent and okay ----
            if (fiveMs && !drumMode && !notePlaying &&
                xRead > 0.0f && fabsf(zRead) <= PLAY_BAND_DEG)
            {
                Serial.println(F("t 3–2: start note (was silent)"));
                vibForceStop();
                curFreq = (int)xRead;
                playNote(curFreq);
                notePlaying = true;

                ret.savedClock = clock;
                ret.state = s_REG_CALC;
                break;
            }

            // ---- Vibrato ----
            if (fiveMs && !drumMode && notePlaying && yRead > 0.0f)
            {
                Serial.println(F("t 3–2b: vibrato()"));
                vibApply(yRead);
                ret.savedClock = clock;
                ret.state = s_REG_CALC;
                break;
            }

            // ---- Retune ----
            if (fiveMs && !drumMode &&
                fabsf(ret.noteFrequency - currState.noteFrequency) > 1.0f)
            {
                Serial.println(F("t 3–2c: playNote(retune)"));
                vibForceStop();

                if (ret.noteFrequency > 0 &&
                    fabsf(zRead) <= PLAY_BAND_DEG)
                {
                    curFreq = (int)ret.noteFrequency;
                    playNote(curFreq);
                    notePlaying = true;
                }
                else
                {
                    doStop();
                }

                ret.savedClock = clock;
                ret.state = s_REG_CALC;
                break;
            }

            // ---- Switch to drum mode ----
            if (drumMode && !currState.gestureModeOn)
            {
                Serial.println(F("t 3–4: You are in Drum Mode!"));
                ret.gestureModeOn = true;
                ret.state = s_GESTURE_WAIT;
            }
            break;
        }

        case s_GESTURE_WAIT:
        {
            #ifndef TESTING // actual rotation values
            float gx, gy, gz;
            computeGestureAxes(ax_g, ay_g, az_g, gx, gy, gz);
            #else // mock rotation values
            float gx = ax_g;
            float gy = ay_g;
            float gz = az_g;
            #endif

            // Back to regular mode
            if (!drumMode && currState.gestureModeOn)
            {
                Serial.println(F("t 4–3: You are in Regular Mode!"));
                ret.gestureModeOn = false;
                ret.state = s_REG_WAIT;
                break;
            }

            if (fiveMs && drumMode)
            {
              unsigned long now = clock;
                if (gx < -25.0f) { Kick(now);   ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
                if (gx >  25.0f) { Snare(now);  ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
                if (gy >  25.0f) {  Tom(now);    ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
                if (gy < -25.0f) { Hat(now);    ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
                if (ret.yaw_deg > PLAY_BAND_DEG) { Ride(now);   ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
                if (ret.yaw_deg < -PLAY_BAND_DEG) { Cymbal(now); ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
            }
            break;
        }

        case s_GESTURE_CALC:
        {
            if (fiveMs && drumMode)
            {
                Serial.println(F("t 5–4: back to gesture wait"));
                pet_watchdog();
                ret.savedClock = clock;
                ret.state = s_GESTURE_WAIT;
            }
            break;
        }
    }

    return ret;
}


void pollIMUAndUpdatePitch() {

    unsigned long now = millis();
    if (now - lastIMUms < IMU_DT_MS)
        return;
    lastIMUms = now;

    // ---- Read IMU raw ----
    int16_t axr, ayr, azr, gxr, gyr, gzr;
    if (!mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr))
        return;

    // ---- Convert to physical units ----
    float ax_g   = (float)axr / 16384.0f;
    float ay_g   = (float)ayr / 16384.0f;
    float az_g   = (float)azr / 16384.0f;

    float gx_dps = (float)gxr / 131.0f;
    float gy_dps = (float)gyr / 131.0f;
    float gz_dps = (float)gzr / 131.0f;

    // ---- FSM CALL ----
    FS = updateFSM(FS, ax_g, ay_g, az_g,
                       gx_dps, gy_dps, gz_dps,
                       now);

    // ---- Recording ----
    if (isRecording()) {
        int effective = notePlaying ? curFreq : 0;
        recordSample(effective);
    }

    // ---- NOTE output ----
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

// static const char* NOTE12[12] =
//   {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

// static inline int hzToMidi(int hz) {
//   if (hz <= 0) return 0;
//   float n = 69.0f + 12.0f * (log((float)hz / 440.0f) / log(2.0f)); // A4=440 -> 69
//   return (int)lroundf(n);
// }

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

void sendNoteToSerial(int hz) {
  int midi = hzToMidi(hz);
  const char* name = NOTE12[(midi % 12 + 12) % 12];

  Serial.print("NOTE:");
  Serial.println(name); // sends it as NOTE:C
}
