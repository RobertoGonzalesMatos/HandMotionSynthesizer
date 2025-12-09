#include "SoundEngine.h"
extern bool drumMode;

typedef struct {
 float xRead;
 float yRead;
 float zRead;
 bool pressDrumButton;
 bool drumModeOn;
 unsigned long clock;
} state_inputs;


bool testTransition(full_state start,
                   full_state end,
                   state_inputs inputs,
                   char* lastFunc);


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
                   state_inputs inputs,
                   char* lastFunc) {

drumMode = inputs.drumModeOn;
 if (inputs.pressDrumButton) drumMode = !drumMode;


 full_state res = updateFSM(start, inputs.xRead, inputs.yRead, inputs.zRead, 0, 0, 0, inputs.clock);

 bool passedTest =  (((!drumMode && res.noteFrequency == end.noteFrequency) || drumMode) &&
                      ((!drumMode && res.vibratoLevel == end.vibratoLevel) || drumMode) &&
                      res.gestureModeOn == end.gestureModeOn &&
                      res.savedClock == end.savedClock &&
                      res.state == end.state);
 if (lastFunc!=mockFunc) passedTest = false;



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
   sprintf(sToPrint, "Inputs: xRead %f | yRead %f | zRead %f | pressDrumButton %s | drumModeOn %s | clock %ld", inputs.xRead, inputs.yRead, inputs.zRead, inputs.pressDrumButton ? "true" : "false", inputs.drumModeOn ? "true" : "false", inputs.clock);
   Serial.println(sToPrint);
   sprintf(sToPrint, "          %6s | %6s | %4s | %10s", "noteFrequency", "vibratoLevel", "gestureModeOn", "savedClock");
   Serial.println(sToPrint);
   sprintf(sToPrint, "starting: %6ld | %6ld | %4ld | %10ld", start.noteFrequency, start.vibratoLevel, start.gestureModeOn, start.savedClock);
   Serial.println(sToPrint);
   sprintf(sToPrint, "expected: %6ld | %6ld | %4ld | %10ld", end.noteFrequency, end.vibratoLevel, end.gestureModeOn, end.savedClock);
   Serial.println(sToPrint);
   sprintf(sToPrint, "actual:   %6ld | %6ld | %4ld | %10ld", res.noteFrequency, res.vibratoLevel, res.gestureModeOn, res.savedClock);
   Serial.println(sToPrint);
   sprintf(sToPrint, "mock func expected: %s, received: %s", lastFunc, mockFunc);
   Serial.println(sToPrint);
   Serial.println("");
   return false;
 }
}

bool multiStateTests() {
  char sToPrint[200];

  // test transition 3-4-5
  mockFunc = "";
  full_state start = {0, 0, 0, 100, s_REG_WAIT};
  drumMode = true;
  full_state intermediate = updateFSM(start, 0, 0, 0, 0, 0, 0, 105);
  full_state end = updateFSM(intermediate, 100, 0, 0, 0, 0, 0, 110);
  if (end.state!=s_GESTURE_CALC || mockFunc!="Snare()"){
    sprintf(sToPrint, "Test from %s to %s to %s FAILED", s2str(s_REG_WAIT), s2str(s_GESTURE_WAIT), s2str(s_GESTURE_CALC));
    Serial.println(sToPrint);
    sprintf(sToPrint, "found %s to %s to %s", s2str(start.state), s2str(intermediate.state), s2str(end.state));
    Serial.println(sToPrint);
    sprintf(sToPrint, "with mockFunc %s, expected %s", mockFunc, "Snare()");
    Serial.println(sToPrint);
    Serial.println("");
    return false;
  }
  else {
    sprintf(sToPrint, "Test from %s to %s to %s PASSED", s2str(start.state), s2str(intermediate.state), s2str(end.state));
    Serial.println(sToPrint);
    Serial.println("");
  }

  // test transition 5-4-3
  mockFunc = "";
  start = {0, 0, 1, 100, s_GESTURE_CALC};
  drumMode = true;
  intermediate = updateFSM(start, 0, 0, 0, 0, 0, 0, 105);
  drumMode = false;
  end = updateFSM(intermediate, 0, 0, 0, 0, 0, 0, 110);
  if (end.state!=s_REG_WAIT || mockFunc!=""){
    sprintf(sToPrint, "Test from %s to %s to %s FAILED", s2str(s_GESTURE_CALC), s2str(s_GESTURE_WAIT), s2str(s_REG_WAIT));
    Serial.println(sToPrint);
    sprintf(sToPrint, "found %s to %s to %s", s2str(start.state), s2str(intermediate.state), s2str(end.state));
    Serial.println(sToPrint);
    sprintf(sToPrint, "with mockFunc %s, expected %s", mockFunc, "");
    Serial.println(sToPrint);
    Serial.println("");
    return false;
  }
  else {
    sprintf(sToPrint, "Test from %s to %s to %s PASSED", s2str(start.state), s2str(intermediate.state), s2str(end.state));
    Serial.println(sToPrint);
    Serial.println("");
  }

  // test transition 3-2-3
  mockFunc = "";
  start = {0, 0, 0, 100, s_REG_WAIT};
  intermediate = updateFSM(start, 100, 0, 0, 0, 0, 0, 105);
  end = updateFSM(intermediate, 0, 0, 0, 0, 0, 0, 110);
  if (end.state!=s_REG_WAIT || mockFunc!=""){
    sprintf(sToPrint, "Test from %s to %s to %s FAILED", s2str(s_REG_WAIT), s2str(s_REG_CALC), s2str(s_REG_WAIT));
    Serial.println(sToPrint);
    sprintf(sToPrint, "found %s to %s to %s", s2str(start.state), s2str(intermediate.state), s2str(end.state));
    Serial.println(sToPrint);
    sprintf(sToPrint, "with mockFunc %s, expected %s", mockFunc, "");
    Serial.println(sToPrint);
    Serial.println("");
    return false;
  }
  else {
    sprintf(sToPrint, "Test from %s to %s to %s PASSED", s2str(start.state), s2str(intermediate.state), s2str(end.state));
    Serial.println(sToPrint);
    Serial.println("");
  }

  return true;
}


const int numTests = 16;
const full_state testStatesIn[numTests] =  {{0, 0, 0, 0, s_INIT, {}}, {0, 0, 0, 0, s_INIT}, {0, 0, 0, 100, s_REG_CALC}, {1, 0, 0, 100, s_REG_WAIT}, {0, 0, 0, 100, s_REG_WAIT}, {1, 0, 0, 100, s_REG_WAIT}, 
                                            {600, 0, 0, 100, s_REG_WAIT}, {0, 0, 0, 0, s_REG_WAIT}, {0, 0, 1, 0, s_GESTURE_WAIT}, {0, 0, 0, 100, s_GESTURE_CALC}, {0, 0, 0, 100, s_GESTURE_WAIT}, {0, 0, 0, 100, s_GESTURE_WAIT}, {0, 0, 0, 100, s_GESTURE_WAIT}, {0, 0, 0, 100, s_GESTURE_WAIT}, {0, 0, 0, 100, s_GESTURE_WAIT}, {0, 0, 0, 100, s_GESTURE_WAIT}};
const full_state testStatesOut[numTests] = {{0, 0, 0, 0, s_INIT, {}}, {100, 0, 0, 0, s_REG_CALC}, {0, 0, 0, 105, s_REG_WAIT}, {1, 30, 0, 105, s_REG_CALC}, {30, 0, 0, 105, s_REG_CALC}, {1, 0, 0, 105, s_REG_CALC}, 
                                            {600, 0, 0, 105, s_REG_CALC}, {0, 0, 1, 0, s_GESTURE_WAIT}, {0, 0, 0, 0, s_REG_WAIT}, {0, 0, 0, 105, s_GESTURE_WAIT}, {0, 0, 0, 105, s_GESTURE_CALC}, {0, 0, 0, 105, s_GESTURE_CALC}, {0, 0, 0, 105, s_GESTURE_CALC}, {0, 0, 0, 105, s_GESTURE_CALC}, {0, 0, 0, 105, s_GESTURE_CALC}, {0, 0, 0, 105, s_GESTURE_CALC}};
const state_inputs testInputs[numTests] = {{0, 0, 0, 0, 0, 0}, {100, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 105}, {1, 30, 0, 0, 0, 105}, {30, 0, 0, 0, 0, 105}, {1, 0, 30, 0, 0, 105}, 
                                          {600, 0, 10, 0, 0, 105}, {0, 0, 0, 1, 0, 0}, {0, 0, 0, 1, 1, 0}, {0, 0, 0, 0, 1, 105}, {100, 0, 0, 0, 1, 105}, {-100.0, 0, 0, 0, 1, 105}, {0, 100, 0, 0, 1, 105}, {0, -100, 0, 0, 1, 105}, {0, 0, 100, 0, 1, 105}, {0, 0, -100, 0, 1, 105}};
char* testLastFunc[numTests] = {"", "", "", "", "", "doStop()", 
                                "", "", "", "", "Snare()", "Kick()", "Tom()", "Hat()", "Ride()", "Cymbal()"};


bool testAll() {
 for (int i = 0; i < numTests; i++) {
   Serial.print("Running test ");
   Serial.print(i + 1);
   Serial.print(" of ");
   Serial.println(numTests);
   mockFunc = "";
   if (!testTransition(testStatesIn[i], testStatesOut[i], testInputs[i], testLastFunc[i])) {
     return false;
   }
   Serial.println();
 }
 if (!multiStateTests()) return false;
 Serial.println("All tests passed!");
 return true;
}