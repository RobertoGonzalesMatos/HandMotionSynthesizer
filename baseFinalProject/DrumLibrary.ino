#include "DrumLibrary.h"
#include <Arduino.h>

static DrumState drum = { false, 0, 0, { 0, 0, 0, 0, 0 } };

static const DrumEnvelope KICK_ENV   = { 5,  200,  1200, 1500, 0.25f };
static const DrumEnvelope SNARE_ENV  = { 2,  150,  900,  1200, 0.20f };
static const DrumEnvelope TOM_ENV    = { 5,  250,  1500, 1800, 0.30f };
static const DrumEnvelope HAT_ENV    = { 0,   20,  400,  600,  0.10f };
static const DrumEnvelope RIDE_ENV   = { 5,  300,  2500, 2500, 0.35f };
static const DrumEnvelope CYMBAL_ENV = { 5,  600,  3500, 3000, 0.25f };



const int KICK_FREQ   = 60;  
const int SNARE_FREQ  = 100;  
const int TOM_FREQ    = 130;
const int HAT_FREQ    = 6000;
const int RIDE_FREQ   = 350;  
const int CYMBAL_FREQ = 7000;
static unsigned long lastHitMs = 0;
static const unsigned long HIT_COOLDOWN_MS = 1000;


static float evalEnvelope(const DrumEnvelope &env, float tMs) {
  float A = env.attackMs;
  float D = env.decayMs;
  float S = env.sustainMs;
  float R = env.releaseMs;
  float sL = env.sustainLevel;

  if (tMs < 0.0f) tMs = 0.0f;

  if (tMs < A) {
    if (A <= 0.0f) return 1.0f;
    return tMs / A;
  }
  tMs -= A;

  if (tMs < D) {
    if (D <= 0.0f) return sL;
    float x = tMs / D;
    return 1.0f + (sL - 1.0f) * x;
  }
  tMs -= D;

  if (tMs < S) {
    return sL;
  }
  tMs -= S;

  if (tMs < R) {
    if (R <= 0.0f) return 0.0f;
    float x = tMs / R;
    return sL * (1.0f - x);
  }

  return 0.0f;
}


void doStopDrums() {
    stopPlay();
    drum.active = false;
}


static inline void playDrumADSR(int freq, const DrumEnvelope &env, unsigned long now) {
  if (now - lastHitMs < HIT_COOLDOWN_MS) return;
  lastHitMs = now;

  stopPlay();          // stop whatever was playing
  playNote(freq);      // starts GPT2 square wave

  drum.active  = true;
  drum.freq    = freq;
  drum.startMs = now;
  drum.env     = env;

  g_drumGateEnable = true;
  g_drumLevelQ15 = 0;
}

static inline uint16_t levelToQ15(float level) {
  if (level < 0) level = 0;
  if (level > 1) level = 1;
  return (uint16_t)(level * 32767.0f);
}

static inline void pollDrum(unsigned long now) {
  if (!drum.active) {
    g_drumGateEnable = false;
    g_drumLevelQ15 = 0;
    return;
  }

  float tMs = (float)(now - drum.startMs);
  float level = evalEnvelope(drum.env, tMs);

  if (level <= 0.001f) {
    g_drumGateEnable = false;
    g_drumLevelQ15 = 0;
    stopPlay();
    drum.active = false;
    return;
  }

  g_drumGateEnable = true;
  g_drumLevelQ15 = levelToQ15(level);
}



#ifndef TESTING
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