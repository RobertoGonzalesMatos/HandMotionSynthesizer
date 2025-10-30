const int CLOCKFREQ      = 3000000;
const unsigned int TIMER_INT = 31;
const unsigned int NOTE_INT  = 29;
const int OUT_PORT = 1;
const int OUT_PIN  = 6;

int   curFreq        = 0; 
int   baseFreq       = 0; 
float anchor_pitch_deg = 0.0f; 
extern float pitch_deg_filt;

const int VIB_DEPTH_HZ = 5;
const float VIB_RATE_HZ  = 6.0f;

void gptISR();
void noteISR();

int keyToFreq(char c) {
  switch (c) {
    case 'a': return 262;
    case 's': return 294;
    case 'd': return 330;
    case 'f': return 349;
    case 'g': return 392;
    case 'h': return 440;
    case 'j': return 494;
    case 'w': return 277;
    case 'e': return 311;
    case 't': return 370;
    case 'y': return 415;
    case 'u': return 466;
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


  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PDR = 1;


  R_ICU->IELSR[TIMER_INT] = 0;


  NVIC_SetVector((IRQn_Type)TIMER_INT, (uint32_t)&gptISR);
  NVIC_SetPriority((IRQn_Type)TIMER_INT, 14);
  NVIC_EnableIRQ((IRQn_Type)TIMER_INT);

  initNoteGPT();
}

void playNote(int freq) {
  R_GPT2->GTCR_b.CST = 0;

#ifdef SINUSOID
  R_GPT2->GTPR = CLOCKFREQ / (16.0 * freq);
#else
  R_GPT2->GTPR = CLOCKFREQ / (2.0 * freq);
#endif

  R_ICU->IELSR[TIMER_INT] = (0x06d << R_ICU_IELSR_IELS_Pos);

  R_GPT2->GTCR_b.CST = 1;
}

void stopPlay() {
  R_GPT2->GTCR_b.CST = 0;
  R_ICU->IELSR[TIMER_INT] = 0;
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR = 0;
}

void gptISR() {
  R_PFS->PORT[OUT_PORT].PIN[OUT_PIN].PmnPFS_b.PODR ^= 1;
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
