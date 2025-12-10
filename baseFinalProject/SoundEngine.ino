#ifndef SOUND_ENGINE_H
#define SOUND_ENGINE_H

#include <Arduino.h>

const int CLOCKFREQ      = 3000000;

// GPT interrupt vectors
const unsigned int TIMER_INT     = 15;    // GPT2 live note
const unsigned int PLAYBACK_INT  = 31;    // GPT7 playback oscilate note
const unsigned int PLAYBACK_INT2 = 25;    // GPT6 playback change notes
const unsigned int HARMONY_INT   = 27;
const unsigned int HARMONY_INT2  = 26;
const unsigned int HARMONY_INT3  = 28;
const unsigned int NOTE_INT      = 29;


const int OUT_PORT = 1;
const int OUT_PORT1 = 4;
const int OUT_PORT2 = 1;
const int OUT_PORT3 = 3;
const int OUT_PIN  = 6;
const int OUT_PIN1 = 10;
const int OUT_PIN2 = 12;
const int OUT_PIN3 = 04;

// Playback pin 
const int OUT_PORT_PLAYBACK = 4;
const int OUT_PIN_PLAYBACK  = 11;

int   curFreq  = 0;
bool  liveActive = false;

bool  playbackActive = false;

bool drumMode = false;
int baseFreq = 0;

void gptISR();
void playbackISR();
void gptISRHarmony();
void gptISRHarmony2();
void gptISRHarmony3();
void noteISR();

void initNoteGPT();

void initGPT() {
    Serial.println(F("initGPT()"));

    R_MSTP->MSTPCRD_b.MSTPD6 = 0; // GPT2, GPT6
    R_MSTP->MSTPCRD_b.MSTPD5 = 0; // GPT4, GPT5
    R_MSTP->MSTPCRD_b.MSTPD0 = 0; // GPT7

    //GPT2
    R_GPT2->GTCR_b.CST = 0;
    R_GPT2->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT2->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT2->GTCR  = 0b010 << 24;

    NVIC_SetVector((IRQn_Type)TIMER_INT, (uint32_t)&gptISR);
    NVIC_SetPriority((IRQn_Type)TIMER_INT, 10);
    NVIC_EnableIRQ((IRQn_Type)TIMER_INT);
    R_ICU->IELSR[TIMER_INT] = (0x06d << R_ICU_IELSR_IELS_Pos);

    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PMR = 0;
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PDR = 1;

    R_PFS->PORT[OUT_PORT_PLAYBACK].PIN[OUT_PIN_PLAYBACK].PmnPFS_b.PMR = 0;
    R_PFS->PORT[OUT_PORT_PLAYBACK].PIN[OUT_PIN_PLAYBACK].PmnPFS_b.PDR = 1;


    //Harmonies
    R_GPT4->GTCR_b.CST = 0;
    R_GPT4->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT4->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT4->GTCR  = 0b010 << 24;

    R_GPT5->GTCR_b.CST = 0;
    R_GPT5->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT5->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT5->GTCR  = 0b010 << 24;

    R_GPT1->GTCR_b.CST = 0;
    R_GPT1->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT1->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT1->GTCR  = 0b010 << 24;

    // Harmony pins
    R_PFS->PORT[OUT_PORT1].PIN[OUT_PIN1].PmnPFS_b.PDR = 1;
    R_PFS->PORT[OUT_PORT2].PIN[OUT_PIN2].PmnPFS_b.PDR = 1;
    R_PFS->PORT[OUT_PORT3].PIN[OUT_PIN3].PmnPFS_b.PDR = 1;

    R_ICU->IELSR[HARMONY_INT] = 0;
    R_ICU->IELSR[HARMONY_INT2] = 0;
    R_ICU->IELSR[HARMONY_INT3] = 0;

    // Harmony interrupts
    NVIC_SetVector((IRQn_Type)HARMONY_INT, (uint32_t)&gptISRHarmony);
    NVIC_EnableIRQ((IRQn_Type)HARMONY_INT);

    NVIC_SetVector((IRQn_Type)HARMONY_INT2, (uint32_t)&gptISRHarmony2);
    NVIC_EnableIRQ((IRQn_Type)HARMONY_INT2);

    NVIC_SetVector((IRQn_Type)HARMONY_INT3, (uint32_t)&gptISRHarmony3);
    NVIC_EnableIRQ((IRQn_Type)HARMONY_INT3);

    initNoteGPT();
    initPlayBackGPT();
}

//Live play
void playNote(int freq) {
    playing = true;
  R_GPT2->GTCR_b.CST = 0;
if (FS.harmonies[0]) R_GPT4->GTCR_b.CST = 0;
if (FS.harmonies[1]) R_GPT5->GTCR_b.CST = 0;
if (FS.harmonies[2]) R_GPT1->GTCR_b.CST = 0;
curFreq = freq;

if (freq <= 0) {
    liveActive = false;
    R_GPT2->GTCR_b.CST = 0;
    return;
}

  int harmony = freq * pow(2, 4.0/12.0);
  int harmony2 = freq * pow(2, 7.0/12.0);
  int harmony3 = freq * 2;

#ifdef SINUSOID
  R_GPT2->GTPR = CLOCKFREQ / (16.0 * freq);
  if (FS.harmonies[0]) R_GPT4->GTPR = CLOCKFREQ / (16.0 * harmony);
  if (FS.harmonies[1]) R_GPT5->GTPR = CLOCKFREQ / (16.0 * harmony2);
  if (FS.harmonies[2]) R_GPT1->GTPR = CLOCKFREQ / (16.0 * harmony3);
#else
  R_GPT2->GTPR = CLOCKFREQ / (2.0 * freq);
  if (FS.harmonies[0]) R_GPT4->GTPR = CLOCKFREQ / (2.0 * harmony);
  if (FS.harmonies[1]) R_GPT5->GTPR = CLOCKFREQ / (2.0 * harmony2);
  if (FS.harmonies[2]) R_GPT1->GTPR = CLOCKFREQ / (2.0 * harmony3);
#endif

  R_ICU->IELSR[TIMER_INT] = (0x06d << R_ICU_IELSR_IELS_Pos);
  R_GPT2->GTCR_b.CST = 1;
  if (FS.harmonies[0]) {
    R_ICU->IELSR[HARMONY_INT] = (0x07d << R_ICU_IELSR_IELS_Pos);
    R_GPT4->GTCNT = 0;
    R_GPT4->GTCR_b.CST = 1;
  }
  if (FS.harmonies[1]) {
    R_ICU->IELSR[HARMONY_INT2] = (0x085 << R_ICU_IELSR_IELS_Pos);
    R_GPT5->GTCNT = 0;
    R_GPT5->GTCR_b.CST = 1;
  }
  if (FS.harmonies[2]) {
    R_ICU->IELSR[HARMONY_INT3] = (0x065 << R_ICU_IELSR_IELS_Pos);
    R_GPT1->GTCNT = 0;
    R_GPT1->GTCR_b.CST = 1;
  }
    liveActive = true;
}

void stopPlay() {
    playing = false;
    curFreq = 0;
    liveActive = false;
    R_GPT2->GTCR_b.CST = 0;
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR = 0;
}


void gptISR() {
    if (liveActive)
        R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR ^= 1;

    R_GPT2->GTCR_b.CST = 1;
    R_ICU->IELSR_b[TIMER_INT].IR = 0;
}

//Playback
void playbackSetFreq(int freq) {
    if (freq <= 0) {
        playbackActive = false;
        R_GPT7->GTCR_b.CST = 0;  
        return;
    }

    playbackActive = true;

    uint32_t pr = CLOCKFREQ / (2 * freq);

    R_GPT7->GTCR_b.CST = 0;
    R_GPT7->GTPR = pr;
    R_GPT7->GTCR_b.CST = 1;
}


void playbackISR() {
    if (playbackActive) {
        R_PFS->PORT[OUT_PORT_PLAYBACK]
            .PIN[OUT_PIN_PLAYBACK]
            .PmnPFS_b.PODR ^= 1;
    }


    R_GPT7->GTCR_b.CST = 1;
    R_ICU->IELSR_b[PLAYBACK_INT].IR = 0;
}

//Harmonies
void gptISRHarmony() {
    R_PFS->PORT[OUT_PORT1].PIN[OUT_PIN1].PmnPFS_b.PODR ^= 1;
    R_ICU->IELSR_b[HARMONY_INT].IR = 0;
}

void gptISRHarmony2() {
    R_PFS->PORT[OUT_PORT2].PIN[OUT_PIN2].PmnPFS_b.PODR ^= 1;
    R_ICU->IELSR_b[HARMONY_INT2].IR = 0;
}

void gptISRHarmony3() {
    R_PFS->PORT[OUT_PORT3].PIN[OUT_PIN3].PmnPFS_b.PODR ^= 1;
    R_ICU->IELSR_b[HARMONY_INT3].IR = 0;
}

//vibrato
const int VIB_DEPTH_HZ = 5;

void initNoteGPT() {
    R_GPT3->GTCR_b.CST = 0;
    R_GPT3->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT3->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT3->GTCR  = 0b101 << 24;

    R_ICU->IELSR[NOTE_INT] = (0x075 << R_ICU_IELSR_IELS_Pos);

    NVIC_SetVector((IRQn_Type)NOTE_INT, (uint32_t)&noteISR);
    NVIC_EnableIRQ((IRQn_Type)NOTE_INT);
}

void initPlayBackGPT() {
    R_GPT7->GTCR_b.CST = 0;
    R_GPT7->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT7->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT7->GTCR  = 0b010 << 24;

    R_ICU->IELSR[PLAYBACK_INT] = (0x095 << R_ICU_IELSR_IELS_Pos);

    NVIC_SetVector((IRQn_Type)PLAYBACK_INT, (uint32_t)&playbackISR);
        NVIC_SetPriority((IRQn_Type)TIMER_INT, 10);
    NVIC_EnableIRQ((IRQn_Type)PLAYBACK_INT);
    
}

void stopVibrato() {
    if (curFreq > 0)
        playNote(curFreq);
    R_GPT3->GTCR_b.CST = 0;
}

void setVibrato(float vibRateHz) {
    if (!(vibRateHz > 0.1f)) {
        stopVibrato();
        return;
    }

    float f_isr = 4.0f * VIB_DEPTH_HZ * vibRateHz;
    uint32_t gpr = (uint32_t)(46875.0f / f_isr);
    if (gpr == 0) gpr = 1;

    R_GPT3->GTCR_b.CST = 0;
    R_GPT3->GTPR = gpr;
    R_GPT3->GTCR_b.CST = 1;
}

void noteISR() {
    static int offset = 0;
    static bool up = true;

    R_GPT3->GTCR_b.CST = 0;

    if (curFreq > 0)
        playNote(curFreq + offset);

    if (up) offset++; else offset--;
    if (offset >= VIB_DEPTH_HZ) up = false;
    if (offset <= -VIB_DEPTH_HZ) up = true;

    R_GPT3->GTCR_b.CST = 1;
    R_ICU->IELSR_b[NOTE_INT].IR = 0;
}

//recording + playback
struct NoteEvent {
    int freq;
    unsigned long duration;
};

const int MAX_EVENTS = 128;

NoteEvent recBuf[MAX_EVENTS];
int recCount = 0;
bool recActive = false;
unsigned long lastChangeMs = 0;
int lastRecFreq = -1;

bool playbackRunning = false;
int playIndex = 0;
int playbackLastFreq = -1;

void startRecording() {
    recCount = 0;
    recActive = true;
    lastRecFreq = -1;
    lastChangeMs = millis();
}

void stopRecording() {
    recActive = false;
    unsigned long now = millis();

    if (lastRecFreq != -1 && recCount < MAX_EVENTS) {
        unsigned long dur = now - lastChangeMs;
        recBuf[recCount++] = { lastRecFreq, dur };
    }
    Serial.print ("REC:[");
    for (int i = 0; i < recCount; i++) {
        Serial.print("{\"freq\":");
        Serial.print(recBuf[i].freq);
        Serial.print(",\"duration\":");
        Serial.print(recBuf[i].duration);
        Serial.print("}");
        if (i < recCount- 1) Serial.print(",");
        }
    Serial.println("]");
}

void recordSample(int f) {
    if (!recActive) return;

    unsigned long now = millis();

    if (lastRecFreq == -1) {
        lastRecFreq = f;
        lastChangeMs = now;
        return;
    }

    if (f != lastRecFreq) {
        if (recCount < MAX_EVENTS)
            recBuf[recCount++] = { lastRecFreq, now - lastChangeMs };
        lastRecFreq = f;
        lastChangeMs = now;
    }
}
void startPlayback() {
    debugPrintRecording();
    playbackRunning = true;
    playIndex = 0;
    playbackLastFreq = -1;
    recActive = false;

    R_GPT6->GTCR_b.CST = 0;
    R_GPT6->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT6->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT6->GTCR = (0b101 << 24);
    R_ICU->IELSR[PLAYBACK_INT2] = (0x08d << R_ICU_IELSR_IELS_Pos);
    NVIC_SetVector((IRQn_Type)PLAYBACK_INT2, (uint32_t)&gptPlaybackDurationISR);
    NVIC_SetPriority((IRQn_Type)PLAYBACK_INT2, 14);
    NVIC_EnableIRQ((IRQn_Type)PLAYBACK_INT2);
    R_GPT6->GTPR = 0;
    R_GPT6->GTCR_b.CST = 1;
    NoteEvent &ev = recBuf[0];
    playbackLastFreq = -1;
    playbackSetFreq(ev.freq);
    gpt6StartDuration(ev.duration);
}


void gpt6StartDuration(unsigned long ms) {
    uint32_t ticks = (uint32_t)((46875.0 * ms) / 1000.0);

    if (ticks == 0) {
        ticks = 1; 
    }

    R_GPT6->GTCR_b.CST = 0;
    R_GPT6->GTPR = ticks;
    R_GPT6->GTCNT = 0;
    R_GPT6->GTCR_b.CST = 1;
}
void gptPlaybackDurationISR() {
    R_GPT6->GTCR_b.CST = 0;
    R_ICU->IELSR_b[PLAYBACK_INT2].IR = 0;

    playIndex++;

    if (playIndex >= recCount) {
        stopPlayback();
        return;
    }

    NoteEvent &ev = recBuf[playIndex];
    playbackLastFreq = -1;
    playbackSetFreq(ev.freq);

    gpt6StartDuration(ev.duration);
    R_GPT6->GTCR_b.CST = 1;
}

void stopPlayback() {
    playbackRunning = false;
    playbackActive = false;

    R_GPT7->GTCR_b.CST = 0;

    R_PFS->PORT[OUT_PORT_PLAYBACK]
        .PIN[OUT_PIN_PLAYBACK].PmnPFS_b.PODR = 0;
}

bool isPlayingBack() {
    return playbackRunning;
}

bool isRecording() {
    return recActive;
}
void debugPrintRecording() {
    Serial.println(F("===== RECORDED NOTE BUFFER ====="));
    Serial.print(F("Total events: "));
    Serial.println(recCount);
    Serial.println(F("--------------------------------"));

    for (int i = 0; i < recCount; i++) {
        Serial.print(i);
        Serial.print(F(": freq="));
        Serial.print(recBuf[i].freq);
        Serial.print(F(", dur="));
        Serial.print(recBuf[i].duration);
        Serial.println(F(" ms"));
    }

    Serial.println(F("===== END OF RECORDING =====\n"));
}
#endif
