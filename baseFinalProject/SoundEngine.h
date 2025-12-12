#ifndef SOUNDENGINE_H
#define SOUNDENGINE_H
#define TESTING
struct Voice;
const int CLOCKFREQ      = 3000000;

// GPT interrupt vectors
const unsigned int TIMER_INT     = 15;    // GPT2 live note
const unsigned int PLAYBACK_INT  = 31;    // GPT7 playback oscilate note
const unsigned int PLAYBACK_INT2 = 25;    // GPT6 playback change notes
const unsigned int HARMONY_INT   = 27;
const unsigned int HARMONY_INT2  = 26;
const unsigned int HARMONY_INT3  = 28;
const unsigned int NOTE_INT      = 29;     //vibrato
const unsigned int WDT_INT       = 30;


const int OUT_PORT = 1;
const int OUT_PORT1 = 4;
const int OUT_PORT2 = 1;
const int OUT_PORT3 = 3;
const int OUT_PIN  = 6;
const int OUT_PIN1 = 10;
const int OUT_PIN2 = 12;
const int OUT_PIN3 = 04;

const int OUT_PORT_PLAYBACK = 4;
const int OUT_PIN_PLAYBACK  = 11;

const int VIB_DEPTH_HZ = 5;

struct NoteEvent {
    int freq;
    unsigned long duration;
};

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
    bool          gestureMode;
    unsigned long savedClock;
    fsm_state     state;

    float pitch_est;
    float roll_est;
    float yaw_deg;
    float yaw_bias_dps;

    unsigned long last_t_ms; 
    bool          harmonies[3];

} full_state;

void startRecording();
void stopRecording();

void startPlayback();
void stopPlayback();

void servicePlaybackTick();
void updateHarmonyControls();
void harmonyInit();
bool isRecording();         
void recordSample(int freq);

bool playing;

char* mockFunc;

void petWDT();
#endif
