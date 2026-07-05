/*
 * PwmDrv.h
 *
 *  Created on: 15 ??? 2026
 *      Author: spyro
 */

#ifndef PWMDRV_PWMDRV_H_
#define PWMDRV_PWMDRV_H_

#include <drivers/epwm.h>
#include <drivers/hw_include/hw_types.h>
#include <stdint.h>


/* Status return values */
#define EPWM_DC_SOK (0)       /* no error */
#define EPWM_DC_INV_PRMS (-1) /* error, invalid parameters */

/* EPWM IDs */
#define EPWM_ID_MASTER (3)
#define EPWM_ID_0 (4)
#define EPWM_ID_1 (5)
#define EPWM_ID_2 (6)

/* EPWM configuration parameters */
typedef struct _EPwmCfgPrms_t {
  uint32_t epwmId;           /* EPWM ID */
  uint32_t epwmBaseAddr;     /* EPWM base address */
  uint32_t epwmFclkFreq;     /* EPWM functional clock frequency */
  uint32_t hspClkDiv;        /* EPWM High-Speed Time-base Clock Prescale Bits */
  uint32_t clkDiv;           /* EPWM Time-base Clock Prescale Bits */
  uint32_t epwmOutFreq;      /* EPWM output frequency */
  float epwmDutyCycle;       /* EPWM init duty cycle, 0.0-100.0*/
  uint32_t epwmTbCounterDir; /* EPWM counter direction (Up, Down, Up/Down) */

  /* TB sync in config */
  Bool cfgTbSyncIn; /* config TB sync in flag (true/false) */
  float tbPhsValue; /* cfgTbSyncIn==TRUE: timer phase value to load on Sync In
                       event, 0.0-100.0*/
  uint32_t tbSyncInCounterDir; /* cfgTbSyncIn==TRUE: counter direction on Sync
                                  In event */

  /* TB sync out config */
  Bool cfgTbSyncOut;      /* config TB sync output flag (true/false) */
  uint32_t tbSyncOutMode; /* cfgTbSyncOut==TRUE: Sync Out mode */

  /* AQ config */
  EPWM_AqActionCfg aqCfg;

  /* DB config */
  Bool cfgDb;             /* config DB flag (true/false) */
  EPWM_DeadbandCfg dbCfg; /* Deadband config */

  /* ET config */
  Bool cfgEt;      /* config ET module */
  uint32_t intSel; /* ET interrupt select */
  uint32_t intPrd; /* ET interrupt period */
  uint32_t isPin;  /* In case there is no a pwm but a timer */
} EPwmCfgPrms_t;

/* EPWM object */
typedef struct _EPwmObj_t {
  EPwmCfgPrms_t epwmCfgPrms; /* EPWM configuration parameters */
  uint32_t epwmPrdVal;       /* EPWM period value */
} EPwmObj_t;

/* EPWM Handle */
typedef EPwmObj_t *Epwm_Handle;

/* Initialize EPWM */
Epwm_Handle epwmInit(EPwmCfgPrms_t *pEpwmCfgPrms, EPwmObj_t *pEpwmObj);

/* Update EPWM outputs */
int32_t epwmUpdateOut(Epwm_Handle hEpwm, float amp);

/* Write EPWM CMPA */
static inline void writeCmpA(uint32_t baseAddr, volatile uint16_t cmpVal) {
  HW_WR_FIELD16((baseAddr + PWMSS_EPWM_CMPA), PWMSS_EPWM_CMPA,
                (uint16_t)cmpVal);
}

static inline uint16_t Epwm_Get_Period(uint32_t baseAddr) {
  return (HW_RD_REG16(baseAddr + PWMSS_EPWM_TBPRD));
}

static inline void Epwm_Get_Duty(uint32_t baseAddr, volatile uint16_t *Duty) {
  *Duty = HW_RD_REG16(baseAddr + PWMSS_EPWM_CMPA);
}

static inline void Epwm_Change_Duty(uint16_t Period, volatile uint16_t *Duty,
                                    volatile int16_t *Increment) {
  int32_t duty_tmp = (int16_t)*Duty;
  if (((duty_tmp + *Increment) > (int16_t)Period) ||
      (duty_tmp + *Increment < 0)) {
    *Increment = -*Increment;
  }
  duty_tmp += *Increment;

  *Duty = (uint16_t)duty_tmp;
}

/* Write TBCTL HSPDIV & CLKDIV */
static inline void writeTbClkDiv(uint32_t baseAddr, uint32_t hspClkDiv,
                                 uint32_t clkDiv) {
  uint32_t regVal = 0U;

  regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_TBCTL);
  HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_CLKDIV, clkDiv);
  HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_HSPCLKDIV, hspClkDiv);
  HW_WR_REG16((baseAddr + PWMSS_EPWM_TBCTL), (uint16_t)regVal);
}

/* Configure PWM Time base counter Frequency/Period */
void tbPwmFreqCfg(uint32_t baseAddr, uint32_t tbClk, uint32_t pwmFreq,
                  uint32_t counterDir, uint32_t enableShadowWrite,
                  uint32_t *pPeriodCount);

extern void epwm_duty_cycle_sync_init(void *args);
extern void epwm_duty_cycle_sync_task(void *args);

#endif /* PWMDRV_PWMDRV_H_ */
