#include <math.h>
#include "SoundEngine.h"

extern int   curFreq;
extern int   baseFreq;
extern int    drumMode;
int keyToFreq(char c);

extern void mpuInit();
extern void pollIMUAndUpdatePitch();

extern void initGPT();
extern void playNote(int freq);
extern void stopPlay();
extern void stopVibrato();
extern void setVibrato(float vibRateHz);

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  initGPT();
  mpuInit();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;

    // Select a base note SILENTLY. Do NOT call playNote here.
    int f = keyToFreq(c);
    if (f > 0) {
      baseFreq = f;        // choose the base reference for tilt mapping
      // optional: silence any ongoing note until you twist:
      stopPlay();
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
      drumMode = 1;
      continue;
    }
    // 'c' was unused; removed to keep behavior clean
  }

  // IMU code will:
  // - map Y-tilt -> target pitch (silently),
  // - map X-tilt -> vibrato amount,
  // - trigger playNote(curFreq) ONLY on Z-twist.
  pollIMUAndUpdatePitch();
}
