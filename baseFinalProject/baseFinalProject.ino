#include <math.h>
#include "SoundEngine.h"
#include "DrumLibrary.h"
extern int   baseFreq;
extern bool  drumMode;
extern void mpuInit();
extern void pollIMUAndUpdatePitch();
extern void initGPT();
extern void playNote(int freq);
extern void stopPlay();

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  initGPT();
  mpuInit();    
  harmonyInit();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;

    if (c == 'R') {
      startRecording();
      continue;
    }
    if (c == 'r') {
      startPlayback();
      continue;
    }
    if (c == 'S') {
      stopRecording();
      continue;
    }
   
    int f = keyToFreq(c);
    if (f > 0) {
      baseFreq = f;
      stopPlay();      // Wait for IMU to determine pitch
      continue;
    }

    if (c == 'd') {       
      drumMode = !drumMode;
      continue;
    }

  }
  updateHaromnyControls();
  pollIMUAndUpdatePitch();
  pollDrum(millis());
}

int keyToFreq(char c) {
    switch (c) {
        case 'C': return 262;
        case 'D': return 294;
        case 'E': return 330;
        case 'F': return 349;
        case 'G': return 392;
        case 'A': return 440;
        case 'B': return 494;
        case '#': return 0;
    }
    return 0;
}
