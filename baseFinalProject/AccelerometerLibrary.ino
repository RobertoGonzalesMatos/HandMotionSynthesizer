#include "SoundEngine.h"
#include <Wire.h>
#include <math.h>
#include "notes.h"

static full_state FS = {0, 0, false, 0, s_INIT};
extern int  baseFreq;
extern void playNote(int freq);
extern void stopPlay();
extern void stopVibrato();
extern void setVibrato(float vibRateHz);
extern int  curFreq;
extern bool drumMode;


static int lastAnnouncedHz = -1;

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

const float PITCH_MIN_DEG = -60.0f;
const float PITCH_MAX_DEG =  60.0f;
const float TILT_RANGE_SEMITONES = 24.0f;

static int   lastVibLevel = -1;
static const float vibRates[4] = {0.0f, 2.0f, 4.0f, 6.0f};

const float AX_BIAS=0.0f, AY_BIAS=0.0f, AZ_BIAS=0.0f;
const float AX_SF  =1.0f, AY_SF  =1.0f, AZ_SF  =1.0f;

static float pitch_est = 0.0f;  // deg, forward/back
static float roll_est  = 0.0f;  // deg, left/right
const  float CF_ALPHA  = 0.98f;
static unsigned long last_t_ms = 0;

static float yaw_deg = 0.0f;
static float yaw_bias_dps = 0.0f;
const  float YAW_BIAS_ALPHA = 0.002f;
const  float CALM_GZ_DPS    = 10.0f;

const  float PLAY_BAND_DEG  = 25.0f;      // ± yaw window

static bool  notePlaying  = false;
static int   targetFreqHz = 0;

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

#ifndef TESTING 
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

//since angles are continuous and we want to play discrete notes there are times where the progrram will flicker between two notes. 
//to avoid this we add buffers to the current note so that when we change tho the adjacent note the edge is further away
static inline int quantizeWithHys(float semi_cont) {
  if (lastSemi == 9999) { 
    lastSemi = (int)roundf(semi_cont);
    semiEdgeLow  = lastSemi - 0.6f;   
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

    float dt;
    if (currState.last_t_ms == 0)
        dt = IMU_DT_MS / 1000.0f;
    else
        dt = (clock - currState.last_t_ms) / 1000.0f;
    ret.last_t_ms = clock;

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
            if (fiveMs)
            {
                Serial.println(F("t 2–3: reg_calc → reg_wait"));
                float ux, uy, uz = ax_g, ay_g, az_g;
                float pitch_acc, roll_acc;
                computeAccelAngles(ux, uy, uz, pitch_acc, roll_acc);
                applyComplementaryFilter(gx_dps, gy_dps,
                                        pitch_acc, roll_acc,
                                        dt,
                                        ret.pitch_est, ret.roll_est);
                updateYaw(gz_dps, dt,
                          ret.yaw_bias_dps,
                          ret.yaw_deg);
                ret.noteFrequency = computeTargetFreq(ret.pitch_est);

                int vibLevel;
                float vibRateHz;
                computeVibratoBucket(ret.roll_est, vibLevel, vibRateHz);
                ret.vibratoLevel = vibLevel; 
                float yRead = vibRateHz;

                float xRead = (float)ret.noteFrequency;
                float zRead = ret.yaw_deg;
                petWDT();
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

            //silence
            if (fiveMs && !drumMode &&
                (fabsf(zRead) > PLAY_BAND_DEG))
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

            //unsilence
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

            // vibrato
            if (fiveMs && !drumMode && notePlaying && yRead > 0.0f)
            {
                Serial.println(F("t 3–2b: vibrato()"));
                vibApply(yRead);
                ret.savedClock = clock;
                ret.state = s_REG_CALC;
                break;
            }

            // play note
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

            if (fiveMs && !drumMode && notePlaying &&
                xRead > 0.0f && fabsf(zRead) <= PLAY_BAND_DEG) {
              Serial.println(F("t 3–2c: stay on same note"));
              ret.savedClock = clock;
              ret.state = s_REG_CALC;
              break;
            }

            if (drumMode && !currState.gestureModeOn)
            {
                Serial.println(F("t 3–4: You are in Drum Mode!"));
                ret.gestureModeOn = true;
                ret.state = s_GESTURE_WAIT;
                break;
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
            if (!drumMode)
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
                if (ret.yaw_deg > 25) { Ride(now);   ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
                if (ret.yaw_deg < -25) { Cymbal(now); ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
                ret.state = s_GESTURE_CALC;
            }
            break;
        }

        case s_GESTURE_CALC:
        {
            if (fiveMs)
            {
                Serial.println(F("t 5–4: back to gesture wait"));
                float gx, gy, gz;
                computeGestureAxes(ax_g, ay_g, az_g, gx, gy, gz);
                ret.noteFrequency = 0;    
                ret.vibratoLevel = 0;
                updateYaw(gz_dps, dt, ret.yaw_bias_dps, ret.yaw_deg);
                petWDT();
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
  
    int16_t axr, ayr, azr, gxr, gyr, gzr;
    if (!mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr))
      return;

    float ax_g   = (float)axr / 16384.0f;
    float ay_g   = (float)ayr / 16384.0f;
    float az_g   = (float)azr / 16384.0f;

    float gx_dps = (float)gxr / 131.0f;
    float gy_dps = (float)gyr / 131.0f;
    float gz_dps = (float)gzr / 131.0f;

    FS = updateFSM(FS, ax_g, ay_g, az_g,
                       gx_dps, gy_dps, gz_dps,
                       now);

    if (isRecording()) {
        int effective = notePlaying ? curFreq : 0;
        recordSample(effective);
    }

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

void sendNoteToSerial(int hz) {
  int midi = hzToMidi(hz);
  const char* name = NOTE12[(midi % 12 + 12) % 12];

  Serial.print("NOTE:");
  Serial.println(name);
}
