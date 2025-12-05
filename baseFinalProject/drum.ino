#include "SoundEngine.h"

#ifndef TESTING // real drum functions
void Kick() {
  Serial.println(F("Kick!"));
}

void Snare() {
  Serial.println(F("Snare!"));
}

void Tom() {
  Serial.println(F("Tom!"));
}

void Hat() {
  Serial.println(F("Hat!"));
}

void Ride() {
  Serial.println(F("Ride!"));
}

void Cymbal() {
  Serial.println(F("Cymbal!"));
}

#else // mock drum functions
void Kick() {
  mockFunc = "Kick()";
}

void Snare() {
  mockFunc = "Snare()";
}

void Tom() {
  mockFunc = "Tom()";
}

void Hat() {
  mockFunc = "Hat()";
}

void Ride() {
  mockFunc = "Ride()";
}

void Cymbal() {
  mockFunc = "Cymbal()";
}

#endif