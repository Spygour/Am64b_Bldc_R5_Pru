/*
 * PwmDrv.c
 *
 *  Created on: 15 ??? 2026
 *      Author: spyro
 */
#include "PwmDrv.h"

/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "FreeRTOS.h"
#include "PwmDrv.h"
#include "task.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/epwm.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/hw_types.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <math.h>
#include <stdint.h>

/* Configure PWM Time base counter Frequency/Period */
void tbPwmFreqCfg(uint32_t baseAddr, uint32_t tbClk, uint32_t pwmFreq,
                  uint32_t counterDir, uint32_t enableShadowWrite,
                  uint32_t *pPeriodCount) {
  uint32_t tbPeriodCount;
  float tbPeriodCount_f;
  uint32_t regVal = 0U;

  regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_TBCTL);
  HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_PRDLD, enableShadowWrite);
  HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_CTRMODE, counterDir);
  HW_WR_REG16((baseAddr + PWMSS_EPWM_TBCTL), (uint16_t)regVal);

  /* compute period using floating point */
  tbPeriodCount_f = (float)tbClk / pwmFreq;
  if (EPWM_TB_COUNTER_DIR_UP_DOWN == counterDir) {
    tbPeriodCount_f = tbPeriodCount_f / 2.0;
  }
  tbPeriodCount_f = roundf(tbPeriodCount_f);
  tbPeriodCount = (uint32_t)tbPeriodCount_f;

  regVal = (counterDir == EPWM_TB_COUNTER_DIR_UP_DOWN) ? tbPeriodCount
                                                       : tbPeriodCount - 1;
  HW_WR_REG16((baseAddr + PWMSS_EPWM_TBPRD), (uint16_t)regVal);

  *pPeriodCount = tbPeriodCount;
}

/* Min / max output amplitude.
   Waveform amplitude values beyond these thresholds are saturated. */
#define AMP_MAX (1.0f)
#define AMP_MIN (-1.0f)

/* Compute Duty Cycle & CMPx given amplitude & EPWM period */
static void computeCmpx(float amp, uint32_t epwmPrdVal, float *pEpwmDutyCycle,
                        uint16_t *pEpwmCmpVal);

/* Initialize EPWM */
Epwm_Handle epwmInit(EPwmCfgPrms_t *pEpwmCfgPrms, EPwmObj_t *pEpwmObj) {
  Epwm_Handle hEpwm;         /* EPWM handle */
  uint32_t epwmBaseAddr;     /* EPWM base address */
  uint32_t epwmTbFreq;       /* EPWM timebase clock */
  uint32_t epwmOutFreq;      /* EPWM output frequency */
  uint32_t epwmTbCounterDir; /* EPWM TB counter direction */
  uint32_t epwmPrdVal;
  float epwmTbCntVal_f;
  uint32_t epwmTbCntVal;
  float epwmCmpAVal_f;
  uint32_t epwmCmpAVal;

  if ((pEpwmCfgPrms == NULL) || (pEpwmObj == NULL)) {
    return NULL;
  }

  /* Get configuration parameters */
  epwmBaseAddr = pEpwmCfgPrms->epwmBaseAddr;
  epwmOutFreq = pEpwmCfgPrms->epwmOutFreq;
  epwmTbCounterDir = pEpwmCfgPrms->epwmTbCounterDir;

  /* Configure Time Base submodule, clock dividers */
  writeTbClkDiv(epwmBaseAddr, pEpwmCfgPrms->hspClkDiv, pEpwmCfgPrms->clkDiv);

  /* Calculate TB frequency */
  if (pEpwmCfgPrms->hspClkDiv != 0) {
    epwmTbFreq = pEpwmCfgPrms->epwmFclkFreq /
                 (2 * pEpwmCfgPrms->hspClkDiv * (1 << pEpwmCfgPrms->clkDiv));
  } else {
    epwmTbFreq = pEpwmCfgPrms->epwmFclkFreq / (1 * (1 << pEpwmCfgPrms->clkDiv));
  }

  /* Configure Time Base Period, immediate load */
  tbPwmFreqCfg(epwmBaseAddr, epwmTbFreq, epwmOutFreq, epwmTbCounterDir,
               EPWM_SHADOW_REG_CTRL_DISABLE, &epwmPrdVal);
  /* Configure Time Base Period, shadow load mode */
  tbPwmFreqCfg(epwmBaseAddr, epwmTbFreq, epwmOutFreq, epwmTbCounterDir,
               EPWM_SHADOW_REG_CTRL_ENABLE, &epwmPrdVal);

  /* Configure TB Sync In Mode */
  if (pEpwmCfgPrms->cfgTbSyncIn == FALSE) {
    EPWM_tbSyncDisable(epwmBaseAddr);
  } else {
    epwmTbCntVal_f = epwmPrdVal * pEpwmCfgPrms->tbPhsValue;
    epwmTbCntVal = (uint32_t)roundf(epwmTbCntVal_f);
    EPWM_tbSyncEnable(epwmBaseAddr, epwmTbCntVal,
                      pEpwmCfgPrms->tbSyncInCounterDir);
  }

  /* Configure TB Sync Out Mode */
  if (pEpwmCfgPrms->cfgTbSyncOut == FALSE) {
    EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_DISABLE);
  } else {
    EPWM_tbSetSyncOutMode(epwmBaseAddr, pEpwmCfgPrms->tbSyncOutMode);
  }

  /* Configure emulation mode */
  EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

  /*
   *  Compute COMPA value - this determines the duty cycle
   *  COMPA = (PRD - ((DC * PRD) / 100)
   */
  epwmCmpAVal_f = (float)epwmPrdVal;
  epwmCmpAVal_f -= (pEpwmCfgPrms->epwmDutyCycle * epwmPrdVal) / 100.0;
  epwmCmpAVal_f = roundf(epwmCmpAVal_f);
  epwmCmpAVal = (uint32_t)epwmCmpAVal_f;

  /* Configure counter compare submodule */
  EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A, epwmCmpAVal,
                            EPWM_SHADOW_REG_CTRL_ENABLE,
                            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_PRD, TRUE);

  if (pEpwmCfgPrms->isPin == TRUE) {
    /* Configure Action Qualifier Submodule */
    EPWM_aqActionOnOutputCfg(epwmBaseAddr, EPWM_OUTPUT_CH_A,
                             &pEpwmCfgPrms->aqCfg);
  }

  /* Configure Dead Band Submodule */
  if (pEpwmCfgPrms->cfgDb == TRUE) {
    EPWM_deadbandCfg(epwmBaseAddr, &pEpwmCfgPrms->dbCfg);
  } else {
    EPWM_deadbandBypass(epwmBaseAddr);
  }

  /* Configure Chopper Submodule */
  EPWM_chopperEnable(epwmBaseAddr, FALSE);

  /* Configure trip zone Submodule */
  EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
  EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

  /* Configure event trigger Submodule */
  if (pEpwmCfgPrms->cfgEt == TRUE) {
    EPWM_etIntrCfg(epwmBaseAddr, pEpwmCfgPrms->intSel, pEpwmCfgPrms->intPrd);
    EPWM_etIntrEnable(epwmBaseAddr);
  }

  /* Init PWM object */
  hEpwm = (Epwm_Handle)pEpwmObj;
  hEpwm->epwmCfgPrms = *pEpwmCfgPrms;
  hEpwm->epwmPrdVal = epwmPrdVal;

  return hEpwm;
}

/* Update EPWM outputs */
int32_t epwmUpdateOut(Epwm_Handle hEpwm, float amp) {
  float dcVal;     /* EPWM duty cycle value */
  uint16_t cmpVal; /* EPWM CMP value */

  if (hEpwm == NULL) {
    return EPWM_DC_INV_PRMS;
  }

  /* Compute next Duty Cycle and CMP values */
  computeCmpx(amp, hEpwm->epwmPrdVal, &dcVal, &cmpVal);

  /* Write next CMPA value */
  writeCmpA(hEpwm->epwmCfgPrms.epwmBaseAddr, cmpVal);

  return EPWM_DC_SOK;
}

/* Compute Duty Cycle & CMPx given amplitude & EPWM period */
static void computeCmpx(float amp, uint32_t epwmPrdVal, float *pEpwmDutyCycle,
                        uint16_t *pEpwmCmpVal) {
  float dc_f;
  float cmp_f;
  uint16_t cmp;

  if ((pEpwmDutyCycle == NULL) || (pEpwmCmpVal == NULL)) {
    return;
  }

  if (amp >= AMP_MAX) {
    /* 100% duty cycle */
    dc_f = 1.0;
  } else if (amp <= AMP_MIN) {
    /* 0% duty cycle */
    dc_f = 0.0;
  } else {
    /* compute Duty Cycle */
    dc_f = 0.5 * (amp + 1.0);
  }

  /* compute CMPx */
  cmp_f = (1.0 - dc_f) * epwmPrdVal; /* up-down count */
  cmp = (uint16_t)roundf(cmp_f);

  *pEpwmDutyCycle = dc_f;
  *pEpwmCmpVal = cmp;
}

/* EPWM functional clock */
/* Functional clock is the same for all EPWMs */
#define APP_EPWM_FCLK_FREQ (CONFIG_EPWM0_FCLK)

/* EPWM functional clock dividers */
#define APP_EPWM_FCLK_HSPCLKDIV                                                \
  (0x0) /* EPWM_TBCTL:HSPCLKDIV, High-Speed Time-base Clock Prescale Bits */
#define APP_EPWM_FCLK_CLKDIV                                                   \
  (0x0) /* EPWM_TBCTL:CLKDIV, Time-base Clock Prescale Bits */

/* Frequency of PWM output signal in Hz */
#define APP_EPWM_OUTPUT_FREQ_4K (1U * 4000U)
#define APP_EPWM_OUTPUT_FREQ_8K (1U * 8000U)
#define APP_EPWM_OUTPUT_FREQ_16K (1U * 16000U)
#define APP_EPWM_OUTPUT_FREQ                                                   \
  (APP_EPWM_OUTPUT_FREQ_4K) /* EPWM output frequency */

/* Initial Duty Cycle of PWM output, % of EPWM period, 0.0 to 100.0 */
#define APP_MASTER_DUTY_CYCLE (50.0f)
#define APP_EPWM0_DUTY_CYCLE (5.0f)
#define APP_EPWM1_DUTY_CYCLE (5.0f)
#define APP_EPWM2_DUTY_CYCLE (5.0f)

/* Phase loaded on Sync In event, % of TB period, 0.0 to 100.0 */
#define APP_EPWM0_TB_PHASE (0.0f)
#define APP_EPWM1_TB_PHASE (0.0f)
#define APP_EPWM2_TB_PHASE (0.0f)

/* Deadband RED/FED timer counts */
#define APP_EPWM_DB_RED_COUNT (250) /* 1 usec @ 250 MHz, 250/250e6*1e6=1 */
#define APP_EPWM_DB_FED_COUNT (250) /* 1 usec @ 250 MHz, 250/250e6*1e6=1 */

/* Sinusoid parameters */
#define SIN_FREQ (50.0) /* sinusoid frequency */
#define SIN_AMP (0.9) /* sinusoid amplitude */

typedef enum { WAIT_SYNC, ACTUAL_PROGRAM } AppEpwm_State_t;

/* Global variables and objects */
static HwiP_Object gEpwm0HwiObject;
static SemaphoreP_Object gEpwmSyncSemObject;
static AppEpwm_State_t AppEpwm_State = WAIT_SYNC;
/* EPWM ISR */
static void AppEpwm_epwmIntrISR(void *handle);

/* EPWM base addresses */
uint32_t gEpwmMaster;
uint32_t gEpwmBaseAddr[3];

/* EPWM objects */
EPwmObj_t gMasterObj;
EPwmObj_t gEpwm0Obj;
EPwmObj_t gEpwm1Obj;
EPwmObj_t gEpwm2Obj;

/* EPWM handles */
Epwm_Handle hEpwmMaster;
Epwm_Handle hEpwm0;
Epwm_Handle hEpwm1;
Epwm_Handle hEpwm2;

/* Dutycycle */
static volatile uint16_t EpwmDuty[3];

/* Period */
static uint16_t EpwmPeriod;

/* Increment */
static volatile int16_t EpwmInc[3] = {200, 100, 300};

/* Sinusoid variables */
float gSinAmp = SIN_AMP;   /* sinusoid amplitude */
float gSinFreq = SIN_FREQ; /* sinusoid frequency */
uint32_t gSampCnt = 0;     /* sinusoid current sample count */

#define SET_SWSYNC_THR (10U) /* number of EPWM ISRs before SW force sync */
#define SET_UPDOUTISR_THR                                                      \
  (20U) /* number of EPWM ISRs before enabling output generation in ISR */
#define APP_EPWM_RUN_TIME (60U) /* APP run time in seconds */

volatile Bool gUpdOutIsr = FALSE;  /* Flag for updating PWM output in ISR */
volatile uint32_t gEpwmIsrCnt = 0; /* EPWM ISR count */

static volatile Bool IsrHits = FALSE;

/* Debug */
uint32_t gLoopCnt = 0; /* main loop count */

BaseType_t xHigherPriorityTaskWoken = pdFALSE;

void Pwm_InitChannel(uint32_t epwmBaseAddr, uint32_t epwmCh,
                     uint32_t epwmFuncClk, uint32_t epwmFreq,
                     uint32_t epwmDutyCycle, uint32_t epwmPrescaler) {
  EPWM_AqActionCfg aqConfig;
  uint32_t pwmBaseFreq = 250000000 / epwmPrescaler;
  uint32_t pwmCompVal = (pwmBaseFreq / epwmFreq) * (100 - epwmDutyCycle) / 200;

  /* Configure Time base submodule */
  EPWM_tbTimebaseClkCfg(epwmBaseAddr, pwmBaseFreq, epwmFuncClk);
  EPWM_tbPwmFreqCfg(epwmBaseAddr, pwmBaseFreq, epwmFreq,
                    EPWM_TB_COUNTER_DIR_UP_DOWN, EPWM_SHADOW_REG_CTRL_ENABLE);
  EPWM_tbSyncDisable(epwmBaseAddr);
  EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_SYNCIN);
  EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

  /* Configure counter compare submodule */
  EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A, pwmCompVal,
                            EPWM_SHADOW_REG_CTRL_ENABLE,
                            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);
  EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_B, pwmCompVal,
                            EPWM_SHADOW_REG_CTRL_ENABLE,
                            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);

  /* Configure Action Qualifier Submodule */
  aqConfig.zeroAction = EPWM_AQ_ACTION_DONOTHING;
  aqConfig.prdAction = EPWM_AQ_ACTION_DONOTHING;
  aqConfig.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
  aqConfig.cmpADownAction = EPWM_AQ_ACTION_LOW;
  aqConfig.cmpBUpAction = EPWM_AQ_ACTION_HIGH;
  aqConfig.cmpBDownAction = EPWM_AQ_ACTION_LOW;
  EPWM_aqActionOnOutputCfg(epwmBaseAddr, epwmCh, &aqConfig);

  /* Configure Dead Band Submodule */
  EPWM_deadbandBypass(epwmBaseAddr);

  /* Configure Chopper Submodule */
  EPWM_chopperEnable(epwmBaseAddr, FALSE);

  /* Configure trip zone Submodule */
  EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
  EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);
}

void epwm_duty_cycle_sync_init(void *args) {
  int32_t status;
  HwiP_Params hwiPrms;
  EPwmCfgPrms_t epwmCfgPrms;

  DebugP_log("EPWM Duty Cycle Sync Test Started ...\r\n");
  DebugP_log(
      "Please refer to the EXAMPLES_DRIVERS_EPWM_DUTY_CYCLE_SYNC example \
user guide for the test setup to probe the EPWM signals.\r\n");
  DebugP_log("App will wait for 60 seconds (using PWM period ISR) ...\r\n");
  AppEpwm_State = WAIT_SYNC;
  gUpdOutIsr = FALSE;
  ///* Debug, configure GPIO */
  // GPIO_setDirMode(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN,
  // CONFIG_GPIO0_DIR); GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR,
  // CONFIG_GPIO0_PIN); GPIO_pinWriteLow(CONFIG_GPIO0_BASE_ADDR,
  // CONFIG_GPIO0_PIN);
  /* Address translate */
  gEpwmMaster = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM0_BASE_ADDR);
  gEpwmBaseAddr[0] =
      (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM1_BASE_ADDR);
  gEpwmBaseAddr[1] =
      (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM2_BASE_ADDR);
  gEpwmBaseAddr[2] =
      (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_EPWM3_BASE_ADDR);

  /* Create semaphore */
  status = SemaphoreP_constructBinary(&gEpwmSyncSemObject, 0);
  DebugP_assert(SystemP_SUCCESS == status);

  (void)HwiP_disable();
  /* Register & enable EPWM0 interrupt */
  HwiP_Params_init(&hwiPrms);
  hwiPrms.intNum = CONFIG_EPWM0_INTR;
  hwiPrms.priority = 4;
  hwiPrms.callback = &AppEpwm_epwmIntrISR;
  hwiPrms.args = 0;
  hwiPrms.isPulse = CONFIG_EPWM0_INTR_IS_PULSE;
  status = HwiP_construct(&gEpwm0HwiObject, &hwiPrms);
  DebugP_assert(status == SystemP_SUCCESS);

  /* Configure EPWM0 */
  epwmCfgPrms.epwmId = EPWM_ID_MASTER;
  epwmCfgPrms.epwmBaseAddr = gEpwmMaster;
  epwmCfgPrms.epwmFclkFreq = APP_EPWM_FCLK_FREQ;
  epwmCfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
  epwmCfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
  epwmCfgPrms.epwmOutFreq = APP_EPWM_OUTPUT_FREQ;
  epwmCfgPrms.epwmDutyCycle = APP_MASTER_DUTY_CYCLE;
  epwmCfgPrms.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
  epwmCfgPrms.cfgTbSyncIn = TRUE;
  epwmCfgPrms.tbPhsValue = APP_EPWM0_TB_PHASE;
  epwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
  epwmCfgPrms.cfgTbSyncOut = TRUE;
  epwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_CNT_EQ_ZERO;
  epwmCfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
  epwmCfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
  epwmCfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.cfgDb = FALSE;
  epwmCfgPrms.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
  epwmCfgPrms.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
  epwmCfgPrms.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
  epwmCfgPrms.dbCfg.risingEdgeDelay = APP_EPWM_DB_RED_COUNT;
  epwmCfgPrms.dbCfg.fallingEdgeDelay = APP_EPWM_DB_FED_COUNT;
  epwmCfgPrms.cfgEt = TRUE;
  epwmCfgPrms.intSel = EPWM_ET_INTR_EVT_CNT_EQ_ZRO;
  epwmCfgPrms.intPrd = EPWM_ET_INTR_PERIOD_FIRST_EVT;
  epwmCfgPrms.isPin = FALSE;
  hEpwmMaster = epwmInit(&epwmCfgPrms, &gMasterObj);
  DebugP_assert(hEpwmMaster != NULL);

  /* Configure EPWM1 */
  epwmCfgPrms.epwmId = EPWM_ID_0;
  epwmCfgPrms.epwmBaseAddr = gEpwmBaseAddr[0];
  epwmCfgPrms.epwmFclkFreq = APP_EPWM_FCLK_FREQ;
  epwmCfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
  epwmCfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
  epwmCfgPrms.epwmOutFreq = APP_EPWM_OUTPUT_FREQ;
  epwmCfgPrms.epwmDutyCycle = APP_EPWM1_DUTY_CYCLE;
  epwmCfgPrms.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
  epwmCfgPrms.cfgTbSyncIn = TRUE;
  epwmCfgPrms.tbPhsValue = APP_EPWM1_TB_PHASE;
  epwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
  epwmCfgPrms.cfgTbSyncOut = TRUE;
  epwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_SYNCIN;
  epwmCfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
  epwmCfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
  epwmCfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.cfgDb = FALSE;
  epwmCfgPrms.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
  epwmCfgPrms.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
  epwmCfgPrms.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
  epwmCfgPrms.dbCfg.risingEdgeDelay = APP_EPWM_DB_RED_COUNT;
  epwmCfgPrms.dbCfg.fallingEdgeDelay = APP_EPWM_DB_FED_COUNT;
  epwmCfgPrms.cfgEt = FALSE;
  epwmCfgPrms.isPin = TRUE;
  hEpwm0 = epwmInit(&epwmCfgPrms, &gEpwm0Obj);
  DebugP_assert(hEpwm0 != NULL);

  /* Configure EPWM2 */
  epwmCfgPrms.epwmId = EPWM_ID_1;
  epwmCfgPrms.epwmBaseAddr = gEpwmBaseAddr[1];
  epwmCfgPrms.epwmFclkFreq = APP_EPWM_FCLK_FREQ;
  epwmCfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
  epwmCfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
  epwmCfgPrms.epwmOutFreq = APP_EPWM_OUTPUT_FREQ;
  epwmCfgPrms.epwmDutyCycle = APP_EPWM1_DUTY_CYCLE;
  epwmCfgPrms.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
  epwmCfgPrms.cfgTbSyncIn = TRUE;
  epwmCfgPrms.tbPhsValue = APP_EPWM1_TB_PHASE;
  epwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
  epwmCfgPrms.cfgTbSyncOut = TRUE;
  epwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_SYNCIN;
  epwmCfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
  epwmCfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
  epwmCfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.cfgDb = FALSE;
  epwmCfgPrms.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
  epwmCfgPrms.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
  epwmCfgPrms.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
  epwmCfgPrms.dbCfg.risingEdgeDelay = APP_EPWM_DB_RED_COUNT;
  epwmCfgPrms.dbCfg.fallingEdgeDelay = APP_EPWM_DB_FED_COUNT;
  epwmCfgPrms.cfgEt = FALSE;
  epwmCfgPrms.isPin = TRUE;
  hEpwm1 = epwmInit(&epwmCfgPrms, &gEpwm1Obj);
  DebugP_assert(hEpwm1 != NULL);

  /* Configure EPWM3 */
  epwmCfgPrms.epwmId = EPWM_ID_2;
  epwmCfgPrms.epwmBaseAddr = gEpwmBaseAddr[2];
  epwmCfgPrms.epwmFclkFreq = APP_EPWM_FCLK_FREQ;
  epwmCfgPrms.hspClkDiv = APP_EPWM_FCLK_HSPCLKDIV;
  epwmCfgPrms.clkDiv = APP_EPWM_FCLK_CLKDIV;
  epwmCfgPrms.epwmOutFreq = APP_EPWM_OUTPUT_FREQ;
  epwmCfgPrms.epwmDutyCycle = APP_EPWM2_DUTY_CYCLE;
  epwmCfgPrms.epwmTbCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
  epwmCfgPrms.cfgTbSyncIn = TRUE;
  epwmCfgPrms.tbPhsValue = APP_EPWM2_TB_PHASE;
  epwmCfgPrms.tbSyncInCounterDir = EPWM_TB_COUNTER_DIR_UP_DOWN;
  epwmCfgPrms.cfgTbSyncOut = TRUE;
  epwmCfgPrms.tbSyncOutMode = EPWM_TB_SYNC_OUT_EVT_DISABLE;
  epwmCfgPrms.aqCfg.zeroAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.prdAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.cmpAUpAction = EPWM_AQ_ACTION_HIGH;
  epwmCfgPrms.aqCfg.cmpADownAction = EPWM_AQ_ACTION_LOW;
  epwmCfgPrms.aqCfg.cmpBUpAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.aqCfg.cmpBDownAction = EPWM_AQ_ACTION_DONOTHING;
  epwmCfgPrms.cfgDb = TRUE;
  epwmCfgPrms.dbCfg.inputMode = EPWM_DB_IN_MODE_A_RED_A_FED;
  epwmCfgPrms.dbCfg.outputMode = EPWM_DB_OUT_MODE_A_RED_B_FED;
  epwmCfgPrms.dbCfg.polaritySelect = EPWM_DB_POL_SEL_ACTV_HIGH_COMPLEMENTARY;
  epwmCfgPrms.dbCfg.risingEdgeDelay = APP_EPWM_DB_RED_COUNT;
  epwmCfgPrms.dbCfg.fallingEdgeDelay = APP_EPWM_DB_FED_COUNT;
  epwmCfgPrms.cfgEt = FALSE;
  epwmCfgPrms.isPin = TRUE;
  hEpwm2 = epwmInit(&epwmCfgPrms, &gEpwm2Obj);
  DebugP_assert(hEpwm2 != NULL);
  EpwmPeriod = Epwm_Get_Period(gEpwmMaster);
  /* Store the Period and Dutycycle */
  for (uint8_t i = 0; i < 3; i++) {
    Epwm_Get_Duty(gEpwmBaseAddr[0], &EpwmDuty[i]);
  }

  /* 6. Explicitly force core-level enable if this is the first ISR */
  EPWM_etIntrDisable(gEpwmMaster);
  EPWM_etIntrClear(gEpwmMaster);
  EPWM_etIntrEnable(gEpwmMaster);

  HwiP_enable();

  /* Force SW sync for EPWM0.
     Other PWMs will be sync'd through chain.
     SW sync simulates EPWM0SYNCI. */
  EPWM_tbTriggerSwSync(gEpwmMaster);
  /* Initialize the pwm task */
}
void epwm_duty_cycle_sync_task(void *args) {
  while (1) {
    switch (AppEpwm_State) {
    case WAIT_SYNC: {
      if (gEpwmIsrCnt > SET_UPDOUTISR_THR) {
        gUpdOutIsr = TRUE;
        AppEpwm_State = ACTUAL_PROGRAM;
      }
      break;
    }
    case ACTUAL_PROGRAM: {
      if (IsrHits == TRUE) {
        IsrHits = FALSE;
        for (uint8_t i = 0; i < 3; i++) {
          Epwm_Change_Duty(EpwmPeriod, &EpwmDuty[i], &EpwmInc[i]);
        }
        gLoopCnt++;
      }
      break;
    }

    default:
      break;
    }
    gLoopCnt++;

    /* Wait for about 10ms */
    ClockP_usleep(10000);
  }

  /* Disable and clear interrupts for EPWM0 */
  EPWM_etIntrDisable(gEpwmMaster); /* Disable interrupts */
  EPWM_etIntrClear(gEpwmMaster);   /* Clear pending interrupts */

  /* Destroy HWI & semaphore */
  HwiP_destruct(&gEpwm0HwiObject);
  SemaphoreP_destruct(&gEpwmSyncSemObject);

  DebugP_log("EPWM Duty Cycle Sync Test Passed!!\r\n");
  DebugP_log("All tests have passed!!\r\n");

  vTaskDelete(NULL);
}
/* EPWM ISR */
static void AppEpwm_epwmIntrISR(void *args) {
  uint16_t status;

  /* debug, show ISR timing */
  // GPIO_pinWriteHigh(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);
  /* debug, increment ISR count */
  gEpwmIsrCnt++;

  status = EPWM_etIntrStatus(gEpwmMaster);
  if (status & EPWM_ETFLG_INT_MASK) {
    if (gUpdOutIsr == TRUE) {
      for (uint8_t i = 0; i < 3; i++) {
        writeCmpA(gEpwmBaseAddr[i], EpwmDuty[i]);
      }
    }
    IsrHits = TRUE;
    EPWM_etIntrClear(gEpwmMaster);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  /* debug, show ISR timing */
  // GPIO_pinWriteLow(CONFIG_GPIO0_BASE_ADDR, CONFIG_GPIO0_PIN);

  return;
}
