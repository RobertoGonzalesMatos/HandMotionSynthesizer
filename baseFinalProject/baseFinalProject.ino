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

  Serial.println("SETUP");
  testAll();
  while(true);
  analogWriteResolution(12);
  analogRead(15);
  initGPT();
  mpuInit();    
  harmonyInit();
}
bool adcButtonPressed() {

  int v = analogRead(15);  
  if (v>25){
      Serial.print(v);
  }
  return v > 25;              
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
      initWDT();
      petWDT();
      continue;
    }

    if (c == 'd') {       
      flipDrumMode();
      continue;
    }

  }

  if(adcButtonPressed()){
    flipDrumMode();
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
