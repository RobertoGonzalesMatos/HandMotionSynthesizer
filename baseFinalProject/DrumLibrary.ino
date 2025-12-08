#include "DrumLibrary.h"
#include <Arduino.h>
#include "SoundEngine.h"

static DrumState drum = { false, 0, 0, { 0, 0, 0, 0, 0 } };

// =====================================================
// ADSR Definitions
// =====================================================
static const DrumEnvelope KICK_ENV = { 3, 60, 20, 80, 0.2f };
static const DrumEnvelope SNARE_ENV = { 2, 40, 30, 100, 0.25f };
static const DrumEnvelope TOM_ENV = { 3.0f, 40.0f, 40.0f, 80.0f, 0.35f };
static const DrumEnvelope HAT_ENV = { 1, 5, 10, 30, 0.1f };
static const DrumEnvelope RIDE_ENV = { 3.0f, 50.0f, 120.0f, 150.0f, 0.4f };
static const DrumEnvelope CYMBAL_ENV = { 2.0f, 60.0f, 150.0f, 200.0f, 0.4f };
// =====================================================
// DRUM FREQUENCY DEFINITIONS (Actual Values)
// =====================================================
const int KICK_FREQ   = 60;   // low thump
const int SNARE_FREQ  = 180;  // snappy
const int TOM_FREQ    = 140;
const int HAT_FREQ    = 6000; // white noise emulated by high freq
const int RIDE_FREQ   = 350;  
const int CYMBAL_FREQ = 8000;
static unsigned long lastHitMs = 0;
static const unsigned long HIT_COOLDOWN_MS = 120; // adjust to taste

// =====================================================
// Envelope evaluation
// =====================================================
static float evalEnvelope(const DrumEnvelope &env, float tMs) {
  float A = env.attackMs;
  float D = env.decayMs;
  float S = env.sustainMs;
  float R = env.releaseMs;
  float sL = env.sustainLevel;

  if (tMs < 0.0f) tMs = 0.0f;

  // Attack 0 → 1
  if (tMs < A) {
    if (A <= 0.0f) return 1.0f;
    return tMs / A;
  }
  tMs -= A;

  // Decay 1 → sustainLevel
  if (tMs < D) {
    if (D <= 0.0f) return sL;
    float x = tMs / D;
    return 1.0f + (sL - 1.0f) * x;
  }
  tMs -= D;

  // Sustain
  if (tMs < S) {
    return sL;
  }
  tMs -= S;

  // Release sustainLevel → 0
  if (tMs < R) {
    if (R <= 0.0f) return 0.0f;
    float x = tMs / R;
    return sL * (1.0f - x);
  }

  return 0.0f;
}

void setDrumLevel(float level) {
  if (level < 0.0f) level = 0.0f;
  if (level > 1.0f) level = 1.0f;

  uint32_t period = R_GPT2->GTPR;
  if (period == 0) return;

  uint32_t duty = (uint32_t)(period * level);
  R_GPT2->GTCCR[0] = duty; 
}


void doStopDrums() {
    // Stop the PWM / GPT audio output
    stopPlay();

    // Hard mute output
    setDrumLevel(0.0f);

    // Mark drum inactive
    drum.active = false;
}


// =====================================================
// Start drum hit
// =====================================================
static inline void playDrumADSR(int freq, const DrumEnvelope &env, unsigned long now) {
  if (now - lastHitMs < HIT_COOLDOWN_MS) {
    return;  // too soon, ignore retrigger
}
lastHitMs = now;

    // Stop previous drum voice
    doStopDrums();

    // Start PWM on GPT2 at the drum frequency
    playNote(freq);      // <- THIS is REQUIRED for sound

    // Initialize envelope state
    drum.active  = true;
    drum.freq    = freq;
    drum.startMs = now;
    drum.env     = env;

    // Start at 0 level
    setDrumLevel(0.0f);
}


// =====================================================
// Poll function — call every loop()
// =====================================================
static inline void pollDrum(unsigned long now) {
    if (!drum.active) return;

    float tMs = (float)(now - drum.startMs);
    float level = evalEnvelope(drum.env, tMs);

    if (level <= 0.001f) {
        setDrumLevel(0.0f);
        stopPlay();
        drum.active = false;
        return;
    }

    setDrumLevel(level);
}
void setDrumFrequency(int freq) {
    if (freq <= 0) return;

    uint32_t clk = 3000000;    // GPT2 clock (3 MHz for R4 Uno typically)
    uint32_t period = clk / freq;

    if (period < 100)  period = 100;   // limit for high freqs
    if (period > 65000) period = 65000; // limit for low freqs

    R_GPT2->GTPR = period;
    R_GPT2->GTCCR[0] = 0;
}

// =====================================================
// Gesture trigger functions
// =====================================================
#ifndef TESTING // real drum functions
static inline void Kick(unsigned long now) {
  Serial.println(F("[DRUM] Kick"));
  playDrumADSR(KICK_FREQ, KICK_ENV, now);
}
static inline void Snare(unsigned long now) {
  Serial.println(F("[DRUM] Snare"));
  playDrumADSR(SNARE_FREQ, SNARE_ENV, now);
}
static inline void Tom(unsigned long now) {
  Serial.println(F("[DRUM] Tom"));
  playDrumADSR(TOM_FREQ, TOM_ENV, now);
}
static inline void Hat(unsigned long now) {
  Serial.println(F("[DRUM] Hat"));
  playDrumADSR(HAT_FREQ, HAT_ENV, now);
}
static inline void Ride(unsigned long now) {
  Serial.println(F("[DRUM] Ride"));
  playDrumADSR(RIDE_FREQ, RIDE_ENV, now);
}
static inline void Cymbal(unsigned long now) {
  Serial.println(F("[DRUM] Cymbal"));
  playDrumADSR(CYMBAL_FREQ, CYMBAL_ENV, now);
}
#else // mock drum functions
void Kick(unsigned long now) {
  mockFunc = "Kick()";
}
void Snare(unsigned long now) {
  mockFunc = "Snare()";
}
void Tom(unsigned long now) {
  mockFunc = "Tom()";
}
void Hat(unsigned long now) {
  mockFunc = "Hat()";
}
void Ride(unsigned long now) {
  mockFunc = "Ride()";
}
void Cymbal(unsigned long now) {
  mockFunc = "Cymbal()";
}
#endif