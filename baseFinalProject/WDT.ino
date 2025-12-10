#include <Arduino.h>

void initWDT() {
  R_WDT->WDTCR_b.CKS = 2000;
  R_WDT->WDTCR_b.TOPS = 10;
  R_WDT->WDTCR_b.RPSS = 11;
  R_WDT->WDTCR_b.RPES = 11;

  R_DEBUG->DBGSTOPCR_b.DBGSTOP_WDT = 0;
  R_WDT->WDTSR = 0; 

  R_WDT->WDTRCR= (0 << 7);
  R_ICU->IELSR_b[WDT_INT].IELS = 0x025;

  NVIC_SetVector((IRQn_Type)WDT_INT, (uint32_t)&wdtISR);
  NVIC_SetPriority((IRQn_Type)WDT_INT, 14);
  NVIC_EnableIRQ((IRQn_Type)WDT_INT);
}

void petWDT() {
  R_WDT->WDTRR = 0x00;
  R_WDT->WDTRR = 0xFF;
}

void wdtISR() {
  Serial.println("WOOF!!!");
  while (true); 
}
