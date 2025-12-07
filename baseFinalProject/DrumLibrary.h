#ifndef DRUM_LIBRARY_H
#define DRUM_LIBRARY_H

#include <Arduino.h>

// =============================
// TYPES
// =============================
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

// =============
// CONSTANTS
// =============
extern const int KICK_FREQ;
extern const int SNARE_FREQ;
extern const int TOM_FREQ;
extern const int HAT_FREQ;
extern const int RIDE_FREQ;
extern const int CYMBAL_FREQ;

// You must declare functions here if they are used elsewhere
void stopPlay();
void doStopDrums();

#endif
