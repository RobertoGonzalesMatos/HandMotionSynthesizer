#ifndef SOUNDENGINE_H
#define SOUNDENGINE_H

typedef enum {
  s_INIT = 0,
  s_REG_CALC = 1,
  s_REG_WAIT = 2,
  s_GESTURE_WAIT = 3,
  s_GESTURE_CALC = 4,
} fsm_state;

typedef struct {
  unsigned long noteFrequency;
  unsigned long vibratoLevel;
  bool gestureModeOn;
  unsigned long savedClock;
  fsm_state state;
} full_state;

void startRecording();      
void stopRecording();      
void startPlayback();       
void stopPlayback();        
void servicePlaybackTick();
bool isRecording();         
bool isPlayingBack();

#endif

