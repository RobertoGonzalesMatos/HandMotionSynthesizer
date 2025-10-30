#include <math.h>

extern int   curFreq;
extern int   baseFreq;
extern float pitch_deg_filt;
extern float anchor_pitch_deg;
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

    int f = keyToFreq(c);
    if (f > 0) {
      curFreq          = f;
      baseFreq         = f;               
      anchor_pitch_deg = pitch_deg_filt; 
      playNote(f);
      continue;
    }


    if (c == ' ') {     // mute
      stopPlay();
      curFreq  = 0;
      baseFreq = 0;
      continue;
    }
    if (c == 'v') {
      setVibrato(6.0f);
      continue;
    }
    if (c == 'V') {
      stopVibrato();
      continue;
    }
    if (c == 'c') {     
      anchor_pitch_deg = pitch_deg_filt;
      continue;
    }
  }

  pollIMUAndUpdatePitch();
}
