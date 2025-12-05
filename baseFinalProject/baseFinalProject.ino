#include <math.h>
#include "SoundEngine.h"

// ===== External globals from SoundEngine =====
extern int   curFreq;
extern int   baseFreq;
extern bool  drumMode;
int keyToFreq(char c);

// ===== IMU System =====
extern void mpuInit();
extern void pollIMUAndUpdatePitch();

// ===== GPT / Vibrato / Soft Synth =====
extern void initGPT();
extern void stopVibrato();
extern void setVibrato(float vibRateHz);

// These are WRAPPERS implemented inside SoundEngine now.
extern void playNote(int freq);
extern void stopPlay();

// ===== Setup =====
void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  initGPT();    // timers + soft synth
  mpuInit();    // IMU
  harmonyInit();
}

// ===== Main Loop =====
void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;

    // ============================
    //   RECORDING / PLAYBACK
    // ============================
    if (c == 'R') {
      startRecording();
      continue;
    }
    if (c == 'r') {
      startPlayback();
      continue;
    }

    // ============================
    //   LIVE PERFORMANCE CONTROLS  
    //   (Live + Playback simultaneous OK)
    // ============================

    // KEY → BASE NOTE (silent reference)
    int f = keyToFreq(c);
    if (f > 0) {
      baseFreq = f;
      stopPlay();      // Wait for IMU to determine pitch
      continue;
    }

      if (c == ' ') {        // mute / reset
        stopPlay();
        curFreq  = 0;
        baseFreq = 0;
        continue;
      }
      if (c == 'v') {        // force vibrato on (rate example)
        setVibrato(6.0f);
        continue;
      }
      if (c == 'V') {        // vibrato off
        stopVibrato();
        continue;
      }
    if (c == 'd') {        // Drum Mode on
      drumMode = !drumMode;
      continue;
    }

  }
  // Update harmony controls + associated variables
  // updateHaromnyControls();

  // ============================
  //       PLAYBACK ENGINE
  //   (always runs in parallel)
  // ============================
  // servicePlaybackTick();

  // ============================
  //        LIVE IMU ENGINE
  // (always runs, even during playback)
  // ============================
  pollIMUAndUpdatePitch();
}


