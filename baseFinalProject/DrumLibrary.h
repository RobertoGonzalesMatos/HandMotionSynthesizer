#ifndef DRUMLIBRARY_H
#define DRUMLIBRARY_H

static const int KICK_FREQ = 60;
static const int SNARE_FREQ = 220;
static const int TOM_FREQ = 140;
static const int HAT_FREQ = 800;
static const int RIDE_FREQ = 450;
static const int CYMBAL_FREQ = 1200;

// =====================================================
// ADSR Envelope Structure
// =====================================================
struct DrumEnvelope {
  float attackMs;      // A
  float decayMs;       // D
  float sustainMs;     // S
  float releaseMs;     // R
  float sustainLevel;  // sustain amplitude (0..1)
};

// =====================================================
// Drum State Machine
// =====================================================
struct DrumState {
  bool active;
  int freq;
  unsigned long startMs;
  DrumEnvelope env;
};

#endif  // DRUMLIBRARY_H
