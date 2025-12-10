#ifndef SOUNDENGINE_H
#define SOUNDENGINE_H
struct Voice;

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
    bool          gestureModeOn;
    unsigned long savedClock;
    fsm_state     state;
    bool          harmonies[3];

    float pitch_est;
    float roll_est;
    float yaw_deg;
    float yaw_bias_dps;

    unsigned long last_t_ms; 

} full_state;

void startRecording();
void stopRecording();

void startPlayback();
void stopPlayback();

void servicePlaybackTick();
void updateHarmonyControls();
void harmonyInit();
bool isRecording();         
bool isPlayingBack();
void recordSample(int freq);

bool playing;

char* mockFunc;

const unsigned int WDT_INT = 30;
void petWDT();
#endif
