#include "SoundEngine.h"

const int CLOCKFREQ      = 3000000;
const unsigned int TIMER_INT = 30;
const unsigned int HARMONY_INT = 27;
const unsigned int HARMONY_INT2 = 31;
const unsigned int HARMONY_INT3 = 28;
const unsigned int NOTE_INT  = 29;
const int OUT_PORT = 1;
const int OUT_PIN  = 5;
const int OUT_PIN1 = 6;
const int OUT_PIN2 = 7;
const int OUT_PIN3 = 11;

int   curFreq        = 0; 
int   baseFreq       = 0; 
int drumMode = 0;


const int VIB_DEPTH_HZ = 5;
const float VIB_RATE_HZ  = 6.0f;

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


  R_MSTP->MSTPCRD_b.MSTPD6 = 0; // enables gpt2, gpt6
  R_MSTP->MSTPCRD_b.MSTPD5 = 0;  // enables gpt4, gpt5

  // setup for gpt2 (main note)
  R_GPT2->GTCR_b.CST = 0;
  R_GPT2->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
  R_GPT2->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
  R_GPT2->GTCR  = 0b010 << 24;   

  // setup for gpt4 (harmonies)
  R_GPT4->GTCR_b.CST = 0;
  R_GPT4->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
  R_GPT4->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
  R_GPT4->GTCR  = 0b010 << 24; 

  // setup for gpt5 (harmony 2)
  R_GPT5->GTCR_b.CST = 0;
  R_GPT5->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
  R_GPT5->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
  R_GPT5->GTCR  = 0b010 << 24; 

  // setup for gpt6 (harmony 3)
  R_GPT6->GTCR_b.CST = 0;
  R_GPT6->GTSSR = (1 << R_GPT0_GTSSR_CSTRT_Pos);
  R_GPT6->GTPSR = (1 << R_GPT0_GTPSR_CSTOP_Pos);
  R_GPT6->GTCR  = 0b010 << 24; 


  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PDR = 1;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN1].PmnPFS_b.PDR = 1;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN2].PmnPFS_b.PDR = 1;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN3].PmnPFS_b.PDR = 1;


  R_ICU->IELSR[TIMER_INT] = 0;
  R_ICU->IELSR[HARMONY_INT] = 0;
  R_ICU->IELSR[HARMONY_INT2] = 0;
  R_ICU->IELSR[HARMONY_INT3] = 0;


  NVIC_SetVector((IRQn_Type)TIMER_INT, (uint32_t)&gptISR);
  NVIC_SetPriority((IRQn_Type)TIMER_INT, 14);
  NVIC_EnableIRQ((IRQn_Type)TIMER_INT);

  NVIC_SetVector((IRQn_Type)HARMONY_INT, (uint32_t)&gptISRHarmony);
  NVIC_SetPriority((IRQn_Type)HARMONY_INT, 14);
  NVIC_EnableIRQ((IRQn_Type)HARMONY_INT);

  NVIC_SetVector((IRQn_Type)HARMONY_INT2, (uint32_t)&gptISRHarmony2);
  NVIC_SetPriority((IRQn_Type)HARMONY_INT2, 14);
  NVIC_EnableIRQ((IRQn_Type)HARMONY_INT2);

  NVIC_SetVector((IRQn_Type)HARMONY_INT3, (uint32_t)&gptISRHarmony3);
  NVIC_SetPriority((IRQn_Type)HARMONY_INT3, 14);
  NVIC_EnableIRQ((IRQn_Type)HARMONY_INT3);

  initNoteGPT();
}

void playNote(int freq) {
  R_GPT2->GTCR_b.CST = 0;
  R_GPT4->GTCR_b.CST = 0;
  R_GPT5->GTCR_b.CST = 0;
  R_GPT6->GTCR_b.CST = 0;

  int harmony = freq * pow(2, 4.0/12.0);
  int harmony2 = freq * pow(2, 7.0/12.0);
  int harmony3 = freq * 2;

#ifdef SINUSOID
  R_GPT2->GTPR = CLOCKFREQ / (16.0 * freq);
  R_GPT4->GTPR = CLOCKFREQ / (16.0 * harmony);
  R_GPT5->GTPR = CLOCKFREQ / (16.0 * harmony2);
  R_GPT6->GTPR = CLOCKFREQ / (16.0 * harmony3);
#else
  R_GPT2->GTPR = CLOCKFREQ / (2.0 * freq);
  R_GPT4->GTPR = CLOCKFREQ / (2.0 * harmony);
  R_GPT5->GTPR = CLOCKFREQ / (2.0 * harmony2);
  R_GPT6->GTPR = CLOCKFREQ / (2.0 * harmony3);
#endif

  R_ICU->IELSR[TIMER_INT] = (0x06d << R_ICU_IELSR_IELS_Pos);
  R_ICU->IELSR[HARMONY_INT] = (0x07d << R_ICU_IELSR_IELS_Pos);
  R_ICU->IELSR[HARMONY_INT2] = (0x085 << R_ICU_IELSR_IELS_Pos);
  R_ICU->IELSR[HARMONY_INT3] = (0x08d << R_ICU_IELSR_IELS_Pos);


  R_GPT2->GTCR_b.CST = 1;
  R_GPT4->GTCR_b.CST = 1;
  R_GPT5->GTCR_b.CST = 1;
  R_GPT6->GTCR_b.CST = 1;
}

void stopPlay() {
  R_GPT2->GTCR_b.CST = 0;
  R_GPT4->GTCR_b.CST = 0;
  R_GPT5->GTCR_b.CST = 0;
  R_GPT6->GTCR_b.CST = 0;
  R_ICU->IELSR[TIMER_INT] = 0;
  R_ICU->IELSR[HARMONY_INT] = 0;
  R_ICU->IELSR[HARMONY_INT2] = 0;
  R_ICU->IELSR[HARMONY_INT3] = 0;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR = 0;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN1].PmnPFS_b.PODR = 0;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN2].PmnPFS_b.PODR = 0;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN3].PmnPFS_b.PODR = 0;
}

void gptISR() {
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR ^= 1;
  R_GPT2->GTCR_b.CST = 1;
  R_ICU->IELSR_b[TIMER_INT].IR = 0;
  NVIC_ClearPendingIRQ((IRQn_Type)TIMER_INT);
}

void gptISRHarmony() {
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN1].PmnPFS_b.PODR ^= 1;
  R_GPT4->GTCR_b.CST = 1;
  R_ICU->IELSR_b[HARMONY_INT].IR = 0;
  NVIC_ClearPendingIRQ((IRQn_Type)HARMONY_INT);
}

void gptISRHarmony2() {
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN2].PmnPFS_b.PODR ^= 1;
  R_GPT5->GTCR_b.CST = 1;
  R_ICU->IELSR_b[HARMONY_INT2].IR = 0;
  NVIC_ClearPendingIRQ((IRQn_Type)HARMONY_INT2);
}

void gptISRHarmony3() {
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN3].PmnPFS_b.PODR ^= 1;
  R_GPT6->GTCR_b.CST = 1;
  R_ICU->IELSR_b[HARMONY_INT3].IR = 0;
  NVIC_ClearPendingIRQ((IRQn_Type)HARMONY_INT3);
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

