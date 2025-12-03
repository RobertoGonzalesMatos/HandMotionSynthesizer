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
extern int drumMode;

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

// ===== Recording / Playback state =====
struct NoteEvent {
  int           freq;      // 0 => silence
  unsigned long duration;  // in ms
};

static const int MAX_EVENTS = 128;
static NoteEvent recBuf[MAX_EVENTS];
static int       recCount = 0;

static bool      recActive      = false;
static bool      playbackActive = false;

static int       lastRecFreq    = -1;     // last freq we stored/are timing
static unsigned long lastChangeMs = 0;    // when lastRecFreq started

// Playback cursor
static int       playIndex         = 0;
static unsigned long playEventStartMs = 0;

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

static inline void doStop() {
  if (notePlaying) {
    stopPlay();
    notePlaying = false;
  }
  vibForceStop();
}

bool isRecording()    { return recActive; }
bool isPlayingBack()  { return playbackActive; }


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
      if (fiveMs && !drumMode) {
        Serial.println(F("t 2–3: reg_calc → reg_wait"));
        pet_watchdog();
        ret.savedClock = clock;
        ret.state = s_REG_WAIT;
      }
      break;
      Serial.println("staying in rec_calc");

    case s_REG_WAIT:
      // Silence condition: yaw out of band OR no valid freq → stop.
      if (fiveMs && !drumMode &&
          (fabsf(zRead) > PLAY_BAND_DEG || xRead <= 0.0f)) {
        Serial.println(F("t 3–2a: stop() (yaw out-of-band or no freq)"));
        Serial.println("NOTE:0");
        doStop();
        ret.savedClock = clock;
        ret.state = s_REG_CALC;
        break;
      }

      // Start playing if we are silent, have a freq, and yaw is in band
      if (fiveMs && !drumMode && !notePlaying &&
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
      if (fiveMs && !drumMode && notePlaying && yRead > 0.0f) {
        Serial.println(F("t 3–2b: vibrato()"));
        vibApply(yRead);
        ret.vibratoLevel = (unsigned long)yRead;
        pet_watchdog();
        ret.savedClock = clock;
        ret.state = s_REG_CALC;
        break;
      }

      // Retune if frequency changed significantly
      if (fiveMs && !drumMode &&
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
      if (drumMode && !currState.gestureModeOn) {
        Serial.println(F("t 3–4: You are in Drum Mode!"));
        ret.gestureModeOn = true;
        ret.state = s_GESTURE_WAIT;
      }
      break;
      Serial.println("staying in red_wait");

    case s_GESTURE_WAIT:
      // 4-3: toggle back to Regular Mode
      if (drumMode && currState.gestureModeOn) {
        Serial.println(F("t 4–3: You are in Regular Mode!"));
        ret.gestureModeOn = false;
        ret.state = s_REG_WAIT;
        break;
      }
      // 4-5 (a..f): gesture hits; placeholders for now
      if (fiveMs && !drumMode) {
        if (xRead < -25.0f) { Serial.println(F("Kick!"));   ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
        if (xRead >  25.0f) { Serial.println(F("Snare!"));  ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
        if (yRead >  25.0f) { Serial.println(F("Tom!"));    ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
        if (yRead < -25.0f) { Serial.println(F("Hat!"));    ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
        if (zRead >  25.0f) { Serial.println(F("Ride!"));   ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
        if (zRead < -25.0f) { Serial.println(F("Cymbal!")); ret.state = s_GESTURE_CALC; ret.savedClock = clock; break; }
      }
      break;

    case s_GESTURE_CALC:
      // 5-4: wait 5ms then return to gesture wait
      if (fiveMs && !drumMode) {
        Serial.println(F("t 5–4: back to gesture wait"));
        pet_watchdog();
        ret.savedClock = clock;
        ret.state = s_GESTURE_WAIT;
      }
      break;
  }

  return ret;
}

// ===== Main polling function: read IMU, update orientation, call FSM =====
void pollIMUAndUpdatePitch() {
  if (playbackActive) {
    return;
  }
  unsigned long now = millis();
  if (now - lastIMUms < IMU_DT_MS) return; // ~100 Hz
  lastIMUms = now;

  // 1) Read raw IMU
  int16_t axr, ayr, azr, gxr, gyr, gzr;
  if (!mpuReadRaw(axr, ayr, azr, gxr, gyr, gzr)) return;

  // Convert raw to physical units
  float ax_g = (float)axr / 16384.0f;
  float ay_g = (float)ayr / 16384.0f;
  float az_g = (float)azr / 16384.0f;
  float gx_dps = (float)gxr / 131.0f;
  float gy_dps = (float)gyr / 131.0f;
  float gz_dps = (float)gzr / 131.0f;

  // 2) Apply mounting transform
  float ux, uy, uz;
  applyMount(ax_g, ay_g, az_g, ux, uy, uz);

  // 3) Accel-only reference angles
  float pitch_acc = atan2f(-ux, sqrtf(uy*uy + uz*uz)) * 180.0f / PI;
  float roll_acc  = atan2f( uy, uz ) * 180.0f / PI;

  // 4) Complementary filter
  float dt = (last_t_ms == 0) ? (IMU_DT_MS / 1000.0f)
                              : (now - last_t_ms) / 1000.0f;
  last_t_ms = now;
  pitch_est = CF_ALPHA * (pitch_est + gy_dps * dt) + (1.0f - CF_ALPHA) * pitch_acc;
  roll_est  = CF_ALPHA * (roll_est  + gx_dps * dt) + (1.0f - CF_ALPHA) * roll_acc;

  // 5) Yaw integrate with bias adaptation
  if (fabsf(gz_dps) < CALM_GZ_DPS)
    yaw_bias_dps = (1.0f - YAW_BIAS_ALPHA) * yaw_bias_dps + YAW_BIAS_ALPHA * gz_dps;
  yaw_deg += (gz_dps - yaw_bias_dps) * dt;

  // 6) Map pitch->freq (same as working file)
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

  // 7) Vibrato suggestion from roll (bucket -> Hz)
  float roll_deg = roll_est;
  if (roll_deg < 0.0f)  roll_deg = 0.0f;
  if (roll_deg > 90.0f) roll_deg = 90.0f;
  int vibLevel;
  if      (roll_deg < 15.0f) vibLevel = 0;
  else if (roll_deg < 35.0f) vibLevel = 1;
  else if (roll_deg < 60.0f) vibLevel = 2;
  else                       vibLevel = 3;
  if (vibLevel != lastVibLevel) lastVibLevel = vibLevel;
  float desiredVibRate = vibRates[lastVibLevel < 0 ? 0 : lastVibLevel];

  // 8) ==== FSM CALL (drives play/stop/vibrato per your table) ====
  const bool button = readButton();

  // xRead = freq, yRead = vibRate, zRead = yaw
  
  FS = updateFSM(FS, (float)targetFreqHz, desiredVibRate, yaw_deg, drumMode, now);

  // ---- print played note to Serial ----
  if (notePlaying) {
      // Instrument is playing → print current note if changed
      if (curFreq != lastAnnouncedHz) {
          sendNoteToSerial(curFreq);
          lastAnnouncedHz = curFreq;
      }
  } else {
      // Instrument is silent → print ONLY NOTE:0 (one time)
      if (lastAnnouncedHz != 0) {
          Serial.println("NOTE:0");
          lastAnnouncedHz = 0;
      }
  }


  // 9) ==== Recording logic: record EFFECTIVE freq (played or silence) ====
  if (recActive) {
    int effectiveFreq = (notePlaying ? curFreq : 0);  // 0 == silence

    if (lastRecFreq == -1) {
      // First sample
      lastRecFreq   = effectiveFreq;
      lastChangeMs  = now;
    } else if (effectiveFreq != lastRecFreq) {
      // Close previous event
      if (recCount < MAX_EVENTS) {
        unsigned long dur = now - lastChangeMs;
        if (dur > 0) {
          recBuf[recCount].freq     = lastRecFreq;
          recBuf[recCount].duration = dur;
          recCount++;
        }
      }
      // Start new event
      lastRecFreq  = effectiveFreq;
      lastChangeMs = now;
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
  Serial.println(name); //sends it as NOTE:C


}


//recording


void stopRecording() {
  if (!recActive) return;
  recActive = false;

  unsigned long now = millis();
  if (lastRecFreq != -1 && recCount < MAX_EVENTS) {
    unsigned long dur = now - lastChangeMs;
    if (dur > 0) {
      recBuf[recCount].freq     = lastRecFreq;
      recBuf[recCount].duration = dur;
      recCount++;
    }
  }
  Serial.print("REC:[");
  for (int i = 0; i < recCount; i++) {
  Serial.print("{\"freq\":");
  Serial.print(recBuf[i].freq);
  Serial.print(",\"duration\":");
  Serial.print(recBuf[i].duration);
  Serial.print("}");
  if (i < recCount - 1) Serial.print(",");
}
Serial.println("]");
}

void startRecording() {
  // If we were playing back, stop that first
  if (playbackActive) {
    stopPlayback();
  }

  // Clear buffer
  recCount     = 0;
  lastRecFreq  = -1;
  lastChangeMs = millis();

  recActive      = true;
  playbackActive = false;

  Serial.println(F("[REC] started"));
}

void stopPlayback() {
  if (!playbackActive) return;
  playbackActive = false;

  // Silence output
  doStop();
  curFreq = 0;

  Serial.println(F("[PLAY] stopped"));
}

void startPlayback() {
  // Finish recording if we were in record mode
  if (recActive) {
    stopRecording();
  }

  if (recCount == 0) {
    Serial.println(F("[PLAY] no events to play"));
    return;
  }

  // Reset playback cursor
  playIndex        = 0;
  playEventStartMs = millis();
  playbackActive   = true;

  Serial.print(F("[PLAY] starting, events="));
  Serial.println(recCount);
}

void servicePlaybackTick() {
  if (!playbackActive) return;

  unsigned long now = millis();

  if (playIndex >= recCount) {
    // Finished
    stopPlayback();
    return;
  }

  NoteEvent &ev = recBuf[playIndex];

  // If this is the first time for this event, or we just advanced, ensure correct note
  int f = ev.freq;

  if (f <= 0) {
    // Silence event
    if (notePlaying || curFreq != 0) {
      stopPlay();
      vibForceStop();
      curFreq     = 0;
      notePlaying = false;
    }
  } else {
    if (!notePlaying || curFreq != f) {
      vibForceStop();   // keep playback “clean”, no leftover vibrato state
      curFreq     = f;
      playNote(curFreq);
      notePlaying = true;
    }
  }

  // Check duration
  unsigned long elapsed = now - playEventStartMs;
  if (elapsed >= ev.duration) {
    // Move to next event
    playIndex++;
    playEventStartMs = now;

    if (playIndex >= recCount) {
      stopPlayback();
      return;
    }
  }
}