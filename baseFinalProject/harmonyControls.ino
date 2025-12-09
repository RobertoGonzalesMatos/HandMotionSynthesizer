#include "SoundEngine.h"
#include <CapacitiveSensor.h>
#include "notes.h"

extern const char* NOTE12[12];
extern int hzToMidi(int hz);

const int CTIME = 8;
int thresholds[3];
int capSensors[3];
bool prevHarmonies[3];

CapacitiveSensor s9 = CapacitiveSensor(10, 9);
CapacitiveSensor s8 = CapacitiveSensor(10, 6);
CapacitiveSensor s7 = CapacitiveSensor(10, 5);

void testCalibration() {
  String labels[3];
  labels[0] = "first harmony";
  labels[1] = "second harmony";
  labels[2] = "third harmony";

  while(true) {
    for(int i = 0; i < 3; i++) {
      CapacitiveSensor* s;
      switch(capSensors[i]) {
        case 9:
        s = &s9;
        break;
        case 8:
        s = &s8;
        break;
        case 7:
        s = &s7;
        break;
        default:
        break;      
      }
      if (s->capacitiveSensorRaw(CTIME) > thresholds[i]) {
        Serial.println(labels[i]);
        Serial.println(s->capacitiveSensorRaw(CTIME));
      }
    }
    delay(50);
  }
}

void updateHaromnyControls() {
  if (drumMode || FS.noteFrequency==0 || !playing) return;
  // update harmony variables
  if ((&s9)->capacitiveSensorRaw(CTIME) > thresholds[0]) {
    FS.harmonies[0] = 1;
  }
  else {
    FS.harmonies[0] = 0;
  }
  if ((&s8)->capacitiveSensorRaw(CTIME) > thresholds[1]) {
    FS.harmonies[1] = 1;
  }
  else {
    FS.harmonies[1] = 0;
  }
  if ((&s7)->capacitiveSensorRaw(CTIME) > thresholds[2]) {
    FS.harmonies[2] = 1;
  }
  else {
    FS.harmonies[2] = 0;
  }

  // update the ports correspondingly and play note if they changed
  if (prevHarmonies[0] != FS.harmonies[0]) {
    if (FS.harmonies[0]) {
      R_PFS->PORT[OUT_PORT1].PIN[OUT_PIN1].PmnPFS_b.PDR = 1;
    }
    else {
      R_PFS->PORT[OUT_PORT1].PIN[OUT_PIN1].PmnPFS_b.PDR = 0;
    }
    playNote(FS.noteFrequency);
  }
  if (prevHarmonies[1] != FS.harmonies[1]) {
    if (FS.harmonies[1]) {
      R_PFS->PORT[OUT_PORT2].PIN[OUT_PIN2].PmnPFS_b.PDR = 1;
    }
    else {
      R_PFS->PORT[OUT_PORT2].PIN[OUT_PIN2].PmnPFS_b.PDR = 0;
    }
    playNote(FS.noteFrequency);
  }
  if (prevHarmonies[2] != FS.harmonies[2]) {
    if (FS.harmonies[2]) {
      R_PFS->PORT[OUT_PORT3].PIN[OUT_PIN3].PmnPFS_b.PDR = 1;
    }
    else {
      R_PFS->PORT[OUT_PORT3].PIN[OUT_PIN3].PmnPFS_b.PDR = 0;
    }
    playNote(FS.noteFrequency);
  }

  // set previous harmonies
  prevHarmonies[0] = FS.harmonies[0];
  prevHarmonies[1] = FS.harmonies[1];
  prevHarmonies[2] = FS.harmonies[2];

  printActiveHarmonies();
}

void harmonyInit() {
  capSensors[0] = 9;
  capSensors[1] = 8;
  capSensors[2] = 7;
  thresholds[0] = 700; //700
  thresholds[1] = 800; //700
  thresholds[2] = 700; //700
  // testCalibration();
}

void printActiveHarmonies() {
    int midi1 = hzToMidi(FS.noteFrequency * pow(2, 4.0/12.0));
    int midi2 = hzToMidi(FS.noteFrequency * pow(2, 7.0/12.0));
    int midi3 = hzToMidi(FS.noteFrequency * 2);

    if (FS.harmonies[0]) {
        Serial.print("1_HARM:");
        Serial.println(NOTE12[midi1 % 12]);
    }
    if (FS.harmonies[1]) {
        Serial.print("2_HARM:");
        Serial.println(NOTE12[midi2 % 12]);
    }
    if (FS.harmonies[2]) {
        Serial.print("3_HARM:");
        Serial.println(NOTE12[midi3 % 12]);
    }
}

