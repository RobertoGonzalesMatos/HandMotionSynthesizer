#include "SoundEngine.h"

const int CLOCKFREQ      = 3000000;
const unsigned int TIMER_INT = 31;
const unsigned int NOTE_INT  = 29;
const int OUT_PORT = 1;
const int OUT_PIN  = 6;
const int sampleRate = 20000;
const int numVoices = 4;
int freq1 = 0;
int vibOffset = 0;

int   curFreq        = 0; 
int   baseFreq       = 0; 

const float VIB_DEPTH_HZ = 10;
const float VIB_RATE_HZ  = 6.0f;

static uint32_t phase[numVoices] = {0};
static uint32_t phaseInc[numVoices] = {0};
static float    freq[numVoices] = {0};

void gptISR();
void noteISR();

int keyToFreq(char c) {
  switch (c) {
    case 'C': return 262; 
    case 'D': return 294; 
    case 'E': return 330;
    case 'F': return 349;
    case 'G': return 392;
    case 'A': return 440;
    case 'B': return 494;
    case 'C#': return 277;
    case 'D#': return 311;
    case 'F#': return 370;
    case 'G#': return 415;
    case 'A#': return 466;
  }
  return 0;
}

void initGPT() {
  Serial.println(F("initGPT()"));


  R_MSTP->MSTPCRD_b.MSTPD6 = 0;


  R_GPT2->GTCR_b.CST = 0;
  R_GPT2->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
  R_GPT2->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
  R_GPT2->GTCR  = 0b010 << 24;   
  R_GPT2->GTPR = CLOCKFREQ / sampleRate;

  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PDR = 1;


  R_ICU->IELSR[TIMER_INT] = 0;


  NVIC_SetVector((IRQn_Type)TIMER_INT, (uint32_t)&gptISR);
  NVIC_SetPriority((IRQn_Type)TIMER_INT, 14);
  NVIC_EnableIRQ((IRQn_Type)TIMER_INT);

  initNoteGPT();
}

void playNote(int f) {
  R_GPT2->GTCR_b.CST = 0;

#ifdef SINUSOID
  // R_GPT2->GTPR = CLOCKFREQ / (16.0 * freq);
#else
  // R_GPT2->GTPR = CLOCKFREQ / (2.0 * freq);
#endif
  freq[0] = f;
  phaseInc[0] = (uint32_t)((f * 4294967296.0) / sampleRate);
  freq[1] = f * pow(2, 1.0/3.0);
  phaseInc[1] = (uint32_t)((freq[1] * 4294967296.0) / sampleRate);

  R_ICU->IELSR[TIMER_INT] = (0x06d << R_ICU_IELSR_IELS_Pos);

  R_GPT2->GTCR_b.CST = 1;
}

void stopPlay() {
  R_GPT2->GTCR_b.CST = 0;
  R_ICU->IELSR[TIMER_INT] = 0;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR = 0;
}

void gptISR() {
  // static uint32_t c1 = 0, c2 = 0;
  // static uint8_t s1 = 0, s2 = 0;
  // freq1 = round(freq1);
  // uint32_t p1 = (freq1 > 0) ? sampleRate / (freq1 * 2) : 0;
  // float freq2 = pow(2, 1.0/3.0) * freq1;
  // freq2 = 0;
  // uint32_t p2 = (freq2 > 0) ? sampleRate / (freq2 * 2) : 0;
  // if (p1 && ++c1 >= p1) { c1 = 0; s1 ^= 1; }
  // if (p2 && ++c2 >= p2) { c2 = 0; s2 ^= 1; }
  // uint8_t out = s1;
  uint32_t mix = 0;
  // --- 4-voice DDS mixer ---
  for (int i = 0; i < numVoices; i++) {
      phase[i] += phaseInc[i];

      // square wave: top bit of accumulator
      uint32_t s = (phase[i] >> 31) & 1;

      // mix together (0..4)
      mix += s;
  }
  // scale mix to PWM range (0..255)
  uint8_t out = (mix * 255) / numVoices;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR = out;

  // R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR ^= 1;
  R_GPT2->GTCR_b.CST = 1;
  R_ICU->IELSR_b[TIMER_INT].IR = 0;
  NVIC_ClearPendingIRQ((IRQn_Type)TIMER_INT);
}

void initNoteGPT() {
  R_GPT3->GTCR_b.CST = 0;
  R_GPT3->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
  R_GPT3->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
  R_GPT3->GTCR  = 0b101 << 24;

  // hook ISR
  R_ICU->IELSR[NOTE_INT] = (0x075 << R_ICU_IELSR_IELS_Pos);
  NVIC_SetVector((IRQn_Type)NOTE_INT, (uint32_t)&noteISR);
  NVIC_SetPriority((IRQn_Type)NOTE_INT, 14);
  NVIC_EnableIRQ((IRQn_Type)NOTE_INT);

  R_GPT3->GTCR_b.CST = 0;
}

void stopVibrato() {
  if (curFreq > 0) {
    playNote(curFreq);
  }
  R_GPT3->GTCR_b.CST = 0;
}

void setVibrato(float vibRateHz) {
  if (vibRateHz <= 0.1f) {
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
  static int  offset = 0;
  static bool up     = true;

  R_GPT3->GTCR_b.CST = 0;

  if (curFreq > 0) {
    playNote(curFreq + offset);
  }

  if (up) offset++;
  else    offset--;

  if (offset >= VIB_DEPTH_HZ)  up = false;
  if (offset <= -VIB_DEPTH_HZ) up = true;

  R_GPT3->GTCR_b.CST = 1;
  R_ICU->IELSR_b[NOTE_INT].IR = 0;
  NVIC_ClearPendingIRQ((IRQn_Type)NOTE_INT);
}