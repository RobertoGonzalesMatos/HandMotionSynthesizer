#ifndef DRUM_LIBRARY_H
#define DRUM_LIBRARY_H

#include <Arduino.h>

volatile uint16_t g_drumLevelQ15 = 0; 
volatile bool     g_drumGateEnable = false;

struct DrumEnvelope {
    float attackMs;
    float decayMs;
    float sustainMs;
    float releaseMs;
    float sustainLevel;
};

struct DrumState {
    bool active;
    int freq;
    unsigned long startMs;
    DrumEnvelope env;
};

extern const int KICK_FREQ;
extern const int SNARE_FREQ;
extern const int TOM_FREQ;
extern const int HAT_FREQ;
extern const int RIDE_FREQ;
extern const int CYMBAL_FREQ;

void stopPlay();
void doStopDrums();

#endif
