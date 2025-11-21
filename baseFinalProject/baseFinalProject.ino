#include <math.h>
#include "SoundEngine.h"

extern int   curFreq;
extern int   baseFreq;
extern bool    drumMode;
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
  delay(500);              
  Serial.println("Arduino ready!");

  initGPT();
  mpuInit();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    Serial.println("Ping");
    delay(1000);
    if (c == '\r' || c == '\n') continue;

    // ---- Recording / Playback controls ----
    if (c == 'R') {
      // Start a fresh recording
      startRecording();
      continue;
    }
    if (c == 'r') {
      // Finish recording (if any) and start playback
      startPlayback();
      continue;
    }

    // ---- Normal controls (only make sense when not playing back) ----
    if (!isPlayingBack()) {
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
      drumMode = !drumMode;
      Serial.println("Drum Mode");
      playNote(440);
      continue;
    }
    }
  }

  // Service playback if active
  servicePlaybackTick();

  // IMU-driven control only when not playing back
  if (!isPlayingBack()) {
    pollIMUAndUpdatePitch();
  }
}

