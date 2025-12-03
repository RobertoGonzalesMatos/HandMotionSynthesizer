#include "SoundEngine.h"

typedef struct {
  unsigned long xRead;
  unsigned long yRead;
  unsigned long zRead;
  bool buttonOn;
  unsigned long clock;
} state_inputs;


bool testTransition(full_state start,
                    full_state end,
                    state_inputs inputs);

char* s2str(fsm_state s) {
  switch(s) {
    case s_INIT:
    return "(0) INIT";
    case s_REG_CALC:
    return "(1) REG_CALC";
    case s_REG_WAIT:
    return "(2) REG_WAIT";
    case s_GESTURE_WAIT:
    return "(3) GESTUREWAIT";
    case s_GESTURE_CALC:
    return "(4) GESTURE_CALC";
    default:
    return "???";
  }
}

bool testTransition(full_state start,
                    full_state end,
                    state_inputs inputs) {

  full_state res = updateFSM(start, inputs.xRead, inputs.yRead, inputs.zRead, inputs.buttonOn, inputs.clock);

  typedef struct {
  unsigned long noteFrequency;
  unsigned long vibratoLevel;
  bool gestureModeOn;
  unsigned long savedClock;
  fsm_state state;
  bool harmonies[3];
} full_state;

  bool passedTest = res.savedClock == end.savedClock && res.state == end.state;
  if (passedTest) {
    if (start.noteFrequency==end.noteFrequency) {
      if (start.noteFrequency!=res.noteFrequency) passedTest = false;
    }
    else {
      if (start.noteFrequency==res.noteFrequency) passedTest = false;
    }
    if (start.vibratoLevel==end.vibratoLevel) {
      if (start.vibratoLevel!=res.vibratoLevel) passedTest = false;
    }
    else {
      if (start.vibratoLevel==res.vibratoLevel) passedTest = false;
    }
  }

  if (passedTest) {
    char sToPrint[200];
    sprintf(sToPrint, "Test from %s to %s PASSED", s2str(start.state), s2str(end.state));
    Serial.println(sToPrint);
    return true;
  } else {
    char sToPrint[200];
    sprintf(sToPrint, "Test from %s to %s FAILED", s2str(start.state), s2str(end.state));
    Serial.println(sToPrint);
    sprintf(sToPrint, "End state expected: %s | actual: %s", s2str(end.state), s2str(res.state));
    Serial.println(sToPrint);
    sprintf(sToPrint, "Inputs: xRead %ld | yRead %ld | zRead %ld | buttonOn %s | clock %ld", inputs.xRead, inputs.yRead, inputs.zRead, inputs.buttonOn ? "true" : "false", inputs.clock);
    Serial.println(sToPrint);
    sprintf(sToPrint, "          %6s | %6s | %4s | %10s", "noteFrequency", "vibratoLevel", "gestureModeOn", "savedClock");
    Serial.println(sToPrint);
    sprintf(sToPrint, "starting: %6ld | %6ld | %4ld | %10ld", start.noteFrequency, start.vibratoLevel, start.gestureModeOn, start.savedClock);
    Serial.println(sToPrint);
    sprintf(sToPrint, "expected: %6ld | %6ld | %4ld | %10ld", end.noteFrequency, end.vibratoLevel, end.gestureModeOn, end.savedClock);
    Serial.println(sToPrint);
    sprintf(sToPrint, "actual:   %6ld | %6ld | %4ld | %10ld", res.noteFrequency, res.vibratoLevel, res.gestureModeOn, res.savedClock);
    Serial.println(sToPrint);
    Serial.println("");
    return false;
  }
}

const int numTests = 16;
const full_state testStatesIn[numTests] =  {{0, 0, 0, 0, s_INIT, {}}, {0, 0, 0, 0, s_INIT}, {500, 0, 0, 100, s_REG_CALC}, {500, 0, 0, 0, s_REG_WAIT}, {0, 0, 0, 0, s_REG_WAIT}, {0, 0, 0, 0, s_REG_WAIT}, {600, 0, 0, 0, s_REG_WAIT}, {0, 0, 0, 0, s_REG_WAIT}, {0, 0, 0, 0, s_GESTURE_CALC}, {0, 0, 0, 100, s_GESTURE_CALC}, {0, 0, 0, 0, s_GESTURE_WAIT}, {0, 0, 0, 0, s_GESTURE_WAIT}, {0, 0, 0, 0, s_GESTURE_WAIT}, {0, 0, 0, 0, s_GESTURE_WAIT}, {0, 0, 0, 0, s_GESTURE_WAIT}, {0, 0, 0, 0, s_GESTURE_WAIT}};
const full_state testStatesOut[numTests] = {{0, 0, 0, 0, s_INIT, {}}, {100, 0, 0, 0, s_REG_CALC}, {500, 0, 0, 105, s_REG_WAIT}, {650, 0, 0, 0, s_REG_CALC}, {0, 200, 0, 0, s_REG_CALC}, {0, 0, 0, 0, s_REG_CALC}, {600, 0, 0, 0, s_REG_CALC}, {0, 0, 0, 0, s_GESTURE_CALC}, {0, 0, 0, 0, s_REG_WAIT}, {0, 0, 0, 105, s_GESTURE_WAIT}, {0, 0, 0, 0, s_GESTURE_CALC}, {0, 0, 0, 0, s_GESTURE_CALC}, {0, 0, 0, 0, s_GESTURE_CALC}, {0, 0, 0, 0, s_GESTURE_CALC}, {0, 0, 0, 0, s_GESTURE_CALC}, {0, 0, 0, 0, s_GESTURE_CALC}};
const state_inputs testInputs[numTests] = {{0, 0, 0, 0, 0}, {100, 0, 0, 0, 0}, {0, 0, 0, 0, 105}, {0, 30, 0, 0, 0}, {30, 0, 0, 0, 0}, {0, 0, 30, 0, 0}, {0, 0, 20, 0, 0}, {0, 0, 0, 1, 0}, {0, 0, 0, 1, 0}, {0, 0, 0, 0, 105}, {100, 0, 0, 0, 0}, {-100, 0, 0, 0, 0}, {0, 100, 0, 0, 0}, {0, -100, 0, 0, 0}, {0, 0, 100, 0, 0}, {0, 0, -100, 0, 0}};
const char* testLastFunc[numTests] = {"", "", "", "", "", "doStop()", "", "", "", "", "Snare()", "Kick()", "Tom()", "Hat()", "Ride()", "Cymbal"};

bool testAll() {
  for (int i = 0; i < numTests; i++) {
    Serial.print("Running test ");
    Serial.print(i + 1);
    Serial.print(" of ");
    Serial.println(numTests);
    // reset();
    if (!testTransition(testStatesIn[i], testStatesOut[i], testInputs[i])) {
      return false;
    }
    Serial.println();
  }
  Serial.println("All tests passed!");
  return true;
}