#ifndef SOUND_ENGINE_H
#define SOUND_ENGINE_H

#include <Arduino.h>

/*********************************************************
 *                   CONSTANTS
 *********************************************************/

const int CLOCKFREQ      = 3000000;

// GPT interrupt vectors
const unsigned int TIMER_INT     = 15;    // GPT2 live note
const unsigned int PLAYBACK_INT  = 31;    // GPT7 playback
const unsigned int HARMONY_INT   = 27;
const unsigned int HARMONY_INT2  = 32;
const unsigned int HARMONY_INT3  = 28;
const unsigned int NOTE_INT      = 29;

// OUTPUT pins
const int OUT_PORT = 1;
const int OUT_PIN  = 6;
const int OUT_PIN1 = 5;
const int OUT_PIN2 = 2;
const int OUT_PIN3 = 11;

// Playback pin (unchanged as requested)
const int OUT_PORT_PLAYBACK = 1;
const int OUT_PIN_PLAYBACK  = 7;

int   curFreq  = 0;
bool  liveActive = false;

bool  playbackActive = false;

bool drumMode = false;
int baseFreq = 0;

/*********************************************************
 *                   PROTOTYPES
 *********************************************************/
void gptISR();
void playbackISR();
void gptISRHarmony();
void gptISRHarmony2();
void gptISRHarmony3();
void noteISR();

/*********************************************************
 *                  KEY → FREQ MAP
 *********************************************************/
int keyToFreq(char c) {
    switch (c) {
        case 'C': return 262;
        case 'D': return 294;
        case 'E': return 330;
        case 'F': return 349;
        case 'G': return 392;
        case 'A': return 440;
        case 'B': return 494;
        case '#': return 0;
    }
    return 0;
}

/*********************************************************
 *               GPT INITIALIZATION
 *********************************************************/
void initNoteGPT();

void initGPT() {
    Serial.println(F("initGPT()"));

    // Enable GPT clocks
    R_MSTP->MSTPCRD_b.MSTPD6 = 0; // GPT2, GPT6
    R_MSTP->MSTPCRD_b.MSTPD5 = 0; // GPT4, GPT5
    R_MSTP->MSTPCRD_b.MSTPD0 = 0; // GPT7 playback

    /***********************
     *   GPT2 = LIVE NOTE
     ***********************/
    R_GPT2->GTCR_b.CST = 0;
    R_GPT2->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT2->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT2->GTCR  = 0b010 << 24;

    NVIC_SetVector((IRQn_Type)TIMER_INT, (uint32_t)&gptISR);
    NVIC_SetPriority((IRQn_Type)TIMER_INT, 10);
    NVIC_EnableIRQ((IRQn_Type)TIMER_INT);
    R_ICU->IELSR[TIMER_INT] = (0x06d << R_ICU_IELSR_IELS_Pos);

    // Live output pin
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PMR = 0;
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PDR = 1;


    /************************
     *   GPT7 = PLAYBACK
     ************************/
    // R_GPT7->GTCR_b.CST = 0;
    // R_GPT7->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    // R_GPT7->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    // R_GPT7->GTCR  = 0b010 << 24;
    // NVIC_SetVector((IRQn_Type)PLAYBACK_INT, (uint32_t)&playbackISR);
    // NVIC_SetPriority((IRQn_Type)PLAYBACK_INT, 10);
    // NVIC_EnableIRQ((IRQn_Type)PLAYBACK_INT);
    // R_ICU->IELSR[PLAYBACK_INT] = (0x095 << R_ICU_IELSR_IELS_Pos);

    R_PFS->PORT[OUT_PORT_PLAYBACK].PIN[OUT_PIN_PLAYBACK].PmnPFS_b.PMR = 0;
    R_PFS->PORT[OUT_PORT_PLAYBACK].PIN[OUT_PIN_PLAYBACK].PmnPFS_b.PDR = 1;


    /***********************
     *   HARMONY (unchanged)
     ***********************/
    R_GPT4->GTCR_b.CST = 0;
    R_GPT4->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT4->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT4->GTCR  = 0b010 << 24;

    R_GPT5->GTCR_b.CST = 0;
    R_GPT5->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT5->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT5->GTCR  = 0b010 << 24;

    R_GPT6->GTCR_b.CST = 0;
    R_GPT6->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
    R_GPT6->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
    R_GPT6->GTCR  = 0b010 << 24;

    // Harmony pins
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN1].PmnPFS_b.PDR = 1;
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN2].PmnPFS_b.PDR = 1;
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN3].PmnPFS_b.PDR = 1;

    // Harmony interrupts
    NVIC_SetVector((IRQn_Type)HARMONY_INT, (uint32_t)&gptISRHarmony);
    NVIC_EnableIRQ((IRQn_Type)HARMONY_INT);

    NVIC_SetVector((IRQn_Type)HARMONY_INT2, (uint32_t)&gptISRHarmony2);
    NVIC_EnableIRQ((IRQn_Type)HARMONY_INT2);

    NVIC_SetVector((IRQn_Type)HARMONY_INT3, (uint32_t)&gptISRHarmony3);
    NVIC_EnableIRQ((IRQn_Type)HARMONY_INT3);


    /***********************
     *   VIBRATO
     ***********************/
    initNoteGPT();
    initPlayBackGPT();
}

/*********************************************************
 *             LIVE NOTE (GPT2)
 *********************************************************/
void playNote(int freq) {
    curFreq = freq;

    if (freq <= 0) {
        liveActive = false;
        R_GPT2->GTCR_b.CST = 0;
        return;
    }

    liveActive = true;
    R_GPT2->GTCR_b.CST = 0;
    R_GPT2->GTPR = CLOCKFREQ / (2 * freq);
    R_GPT2->GTCR_b.CST = 1;
}

void stopPlay() {
    curFreq = 0;
    liveActive = false;
    R_GPT2->GTCR_b.CST = 0;
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR = 0;
}

/*********************************************************
 *                 LIVE ISR (GPT2)
 *********************************************************/
void gptISR() {
    if (liveActive)
        R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR ^= 1;

    R_GPT2->GTCR_b.CST = 1;
    R_ICU->IELSR_b[TIMER_INT].IR = 0;
}

/*********************************************************
 *             PLAYBACK (GPT7)
 *********************************************************/
void playbackSetFreq(int freq) {
    Serial.print("[playbackSetFreq] freq = ");
    Serial.println(freq);

    if (freq <= 0) {
        Serial.println("[playbackSetFreq] freq <= 0 -> stopping");
        playbackActive = false;
        R_GPT7->GTCR_b.CST = 0;
        R_PFS->PORT[OUT_PORT_PLAYBACK]
            .PIN[OUT_PIN_PLAYBACK].PmnPFS_b.PODR = 0;
        return;
    }

    playbackActive = true;


    uint32_t pr = CLOCKFREQ / (2 * freq);
    Serial.print("[playbackSetFreq] GTPR = ");
    Serial.println(pr);

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

/*********************************************************
 *                      HARMONY ISR
 *********************************************************/
void gptISRHarmony() {
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN1].PmnPFS_b.PODR ^= 1;
    R_ICU->IELSR_b[HARMONY_INT].IR = 0;
}

void gptISRHarmony2() {
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN2].PmnPFS_b.PODR ^= 1;
    R_ICU->IELSR_b[HARMONY_INT2].IR = 0;
}

void gptISRHarmony3() {
    R_PFS->PORT[OUT_PORT].PIN[OUT_PIN3].PmnPFS_b.PODR ^= 1;
    R_ICU->IELSR_b[HARMONY_INT3].IR = 0;
}

/*********************************************************
 *                  VIBRATO (unchanged)
 *********************************************************/
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

/*********************************************************
 *      RECORDING + PLAYBACK LOGIC (unchanged)
 *********************************************************/
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
unsigned long playEventStartMs = 0;
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
    Serial.println("[startPlayback] BEGIN");

    playbackRunning = true;
    playIndex = 0;
    playEventStartMs = millis();
    playbackLastFreq = -1;

    Serial.print("[startPlayback] recCount = ");
    Serial.println(recCount);

    if (recCount == 0) {
        Serial.println("[startPlayback] ERROR: recCount=0");
    }
}

void stopPlayback() {
    Serial.println("[stopPlayback] STOPPED");

    playbackRunning = false;
    playbackActive = false;
    R_GPT7->GTCR_b.CST = 0;

    R_PFS->PORT[OUT_PORT_PLAYBACK]
        .PIN[OUT_PIN_PLAYBACK]
        .PmnPFS_b.PODR = 0;
}

bool isPlayingBack() {
    return playbackRunning;
}

bool isRecording() {
    return recActive;
}


void servicePlaybackTick() {
    if (!playbackRunning) return;

    unsigned long now = millis();

    if (playIndex >= recCount) {
        Serial.println("[servicePlaybackTick] DONE — reached end");
        stopPlayback();
        return;
    }

    NoteEvent &ev = recBuf[playIndex];

    // print event info when loading a new one
    if (ev.freq != playbackLastFreq) {
        Serial.print("[servicePlaybackTick] NEW EVENT: index=");
        Serial.print(playIndex);
        Serial.print(" freq=");
        Serial.print(ev.freq);
        Serial.print(" dur=");
        Serial.println(ev.duration);

        playbackLastFreq = ev.freq;
        playbackSetFreq(ev.freq);
    }

    if (now - playEventStartMs >= ev.duration) {
        Serial.print("[servicePlaybackTick] advancing index from ");
        Serial.print(playIndex);
        Serial.print(" to ");
        Serial.println(playIndex + 1);

        playIndex++;
        playEventStartMs = now;
        playbackLastFreq = -1;
    }
}


#endif // SOUND_ENGINE_H
