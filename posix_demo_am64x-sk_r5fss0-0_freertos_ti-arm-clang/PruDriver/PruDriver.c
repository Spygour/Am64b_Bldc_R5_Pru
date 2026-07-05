/* Includes */
#include "PruDriver.h"
#include "../Ads8688/Ads8688_Os.h"
#include "PruImage.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/pruicss.h>
#include <kernel/dpl/SemaphoreP.h>
#include <pru_io/driver/pru_ipc.h>

/* Definitions */
#define SHARED_MEM_BUFFER_SIZE 128

#define MAX_DUTY 119500
#define MIN_DUTY 500
/* Data Types*/

/* Local Variables */

static volatile bool doublebuf = false;

volatile bool mem_config_init = false;
/* Global variables */
PRUICSS_Handle gPruIcss0Handle;

PRU_IPC_Handle gPruIpc0Handle;

PRUICSS_HwAttrs const *hwAttrs;

uint32_t reg_read = 0;

uint32_t ecap_time = 0;
/* Local functions */
void PRU_IPC_Isr(void *args) { SemaphoreP_post(&AdcSem);
PRUICSS_clearEvent(gPruIcss0Handle, gPruIpc0Handle->attrs->sysEventNum); }

void Pwm_EnableOutputs(bool enable) {
  uint32_t memloc = (uint32_t)&gPruIpc0Handle->attrs->config->bufferAddrs;

  CSL_REG32_WR(memloc + (8 << 2), (uint32_t)enable);
}

void Pwm_SendEvent(void) {
  /* Tell to the PRU that we read this */
  PRUICSS_sendEvent(gPruIcss0Handle, 17);
}

void Pwm_SetDeadTime(uint32_t deadTime) {
  uint32_t memloc = (uint32_t)&gPruIpc0Handle->attrs->config->bufferAddrs;

  CSL_REG32_WR(memloc + (7 << 2), deadTime);
}

void Pwm_SetDutycycle(uint32_t *pwmArray) {
  uint32_t pwmArray_loc[3];

  for (uint8_t i = 0; i < 3; i++){
      if (pwmArray[i] > MAX_DUTY){
          pwmArray_loc[i] = MAX_DUTY;
      }
      else if(pwmArray[i] < MIN_DUTY){
          pwmArray_loc[i] = MIN_DUTY;
      }
      else {
          pwmArray_loc[i] = pwmArray[i];
      }
  }
  uint32_t memloc = (uint32_t)&gPruIpc0Handle->attrs->config->bufferAddrs;

  if (mem_config_init == true) {
    doublebuf = !doublebuf;
    if (doublebuf == false) {
      for (uint8_t i = 0; i < 3; i++) {
        CSL_REG32_WR(memloc + (i << 2), pwmArray_loc[i]);
      }
    } else {
      for (uint8_t i = 3; i < 6; i++) {
        CSL_REG32_WR(memloc + (i << 2), pwmArray_loc[i - 3]);
      }
    }
    CSL_REG32_WR(memloc + (6 << 2), (uint32_t)doublebuf);
  }
}

static void PRUICSS_WaitIdle(void) {
  uint32_t Pru_ready = 1;
  while (Pru_ready == 1) {
    Pru_ready = HW_RD_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE +
                                  CSL_ICSSCFG_CGR,
                              CSL_ICSSCFG_CGR_ICSS_PWR_IDLE);
  }
  mem_config_init = true;
}
static void PRUICSS_custom_Init(void) {
  int status;
  gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

  /* Init the isr */
  status = PRUICSS_intcInit(gPruIcss0Handle, &icss0_intc_initdata);
  DebugP_assert(SystemP_SUCCESS == status);

  /* Init the memory */
  status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
  DebugP_assert(status != 0);

  status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU1));
  DebugP_assert(status != 0);

  status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRU0));
  DebugP_assert(status != 0);

  status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRU1));
  DebugP_assert(status != 0);

  status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_SHARED_RAM);
  DebugP_assert(status != 0);

  status = PRUICSS_disableCore(gPruIcss0Handle, 0);
  DebugP_assert(SystemP_SUCCESS == status);
  status = PRUICSS_resetCore(gPruIcss0Handle, 0);
  DebugP_assert(SystemP_SUCCESS == status);

  hwAttrs = (PRUICSS_HwAttrs const *)gPruIcss0Handle->hwAttrs;

  /* Enable core clock */
  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE +
                    CSL_ICSSCFG_CORE_SYNC_REG,
                CSL_ICSSCFG_CORE_SYNC_REG_CORE_VBUSP_SYNC_EN, 1);
  reg_read = HW_RD_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE +
                               CSL_ICSSCFG_CORE_SYNC_REG,
                           CSL_ICSSCFG_CORE_SYNC_REG_CORE_VBUSP_SYNC_EN);

  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_IEPCLK,
                CSL_ICSSCFG_IEPCLK_OCP_EN, 1);
  reg_read = HW_RD_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE +
                               CSL_ICSSCFG_IEPCLK,
                           CSL_ICSSCFG_IEPCLK_OCP_EN);

  /* Enable peripheral clocks */
  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                CSL_ICSSCFG_CGR_ECAP_CLK_EN, 1);
  reg_read =
      HW_RD_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                    CSL_ICSSCFG_CGR_ECAP_CLK_EN);

  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                CSL_ICSSCFG_CGR_INTC_CLK_EN, 1);

  reg_read =
      HW_RD_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                    CSL_ICSSCFG_CGR_ECAP_CLK_EN);

  /* Disable protection register */
  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSS_G_PR1_PROTECT_SLV_REGS_BASE +
                    CSL_ICSS_G_PR1_PROTECT_SLV_UNLOCK_KEY,
                CSL_ICSS_G_PR1_PROTECT_SLV_UNLOCK_KEY_UNLOCK_KEY, 0x83E70B13);

  reg_read =
      HW_RD_FIELD32(hwAttrs->baseAddr + CSL_ICSS_G_PR1_PROTECT_SLV_REGS_BASE +
                        CSL_ICSS_G_PR1_PROTECT_SLV_UNLOCK_KEY,
                    CSL_ICSS_G_PR1_PROTECT_SLV_UNLOCK_KEY_UNLOCK_KEY);

  HW_WR_REG32(hwAttrs->baseAddr + CSL_ICSS_G_PR1_PROTECT_SLV_REGS_BASE +
                  CSL_ICSS_G_PR1_PROTECT_SLV_CFG,
              0);

  reg_read =
      HW_RD_REG32(hwAttrs->baseAddr + CSL_ICSS_G_PR1_PROTECT_SLV_REGS_BASE +
                  CSL_ICSS_G_PR1_PROTECT_SLV_CFG);

  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSS_G_PR1_PROTECT_SLV_REGS_BASE +
                    CSL_ICSS_G_PR1_PROTECT_SLV_UNLOCK_KEY,
                CSL_ICSS_G_PR1_PROTECT_SLV_UNLOCK_KEY_UNLOCK_KEY, 0x0);

  reg_read =
      HW_RD_FIELD32(hwAttrs->baseAddr + CSL_ICSS_G_PR1_PROTECT_SLV_REGS_BASE +
                        CSL_ICSS_G_PR1_PROTECT_SLV_UNLOCK_KEY,
                    CSL_ICSS_G_PR1_PROTECT_SLV_UNLOCK_KEY_UNLOCK_KEY);

  /* Remove the clock stop requests */
  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                CSL_ICSSCFG_CGR_ICSS_STOP_REQ, 0);

  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                CSL_ICSSCFG_CGR_IEP_CLK_STOP_REQ, 0);

  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                CSL_ICSSCFG_CGR_ECAP_CLK_STOP_REQ, 0);

  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                CSL_ICSSCFG_CGR_UART_CLK_STOP_REQ, 0);

  HW_WR_FIELD32(hwAttrs->baseAddr + CSL_ICSSCFG_REGS_BASE + CSL_ICSSCFG_CGR,
                CSL_ICSSCFG_CGR_INTC_CLK_STOP_REQ, 0);
  /* Enable Peripheral mode */
  // PRUICSS_setGpMuxSelect(gPruIcss0Handle, CONFIG_PRU_ICSS0, 1);
}

static void PRUICSS_EnablePwmPorts(void) {
  uint32_t portVal;
  portVal = HW_RD_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                        CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG88);
  /* Mode 3 PWM */
  portVal &= 0xFFFFFFF3;
  HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                  CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG88,
              portVal);

  /* PRU0_GPO1 PAD89 */
  portVal = HW_RD_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                        CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG89);
  /* Mode 3 PWM */
  portVal &= 0xFFFFFFF3;
  HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                  CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG89,
              portVal);

  /* PRU0_GPO3 PAD91 */
  portVal = HW_RD_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                        CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG91);
  /* Mode 3 PWM */
  portVal &= 0xFFFFFFF3;
  HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                  CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG91,
              portVal);

  /* PRU0_GPO5 PAD93 */
  portVal = HW_RD_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                        CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG93);
  /* Mode 3 PWM */
  portVal &= 0xFFFFFFF3;
  ;
  HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                  CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG93,
              portVal);

  /* PRU0_GPO6 PAD94 */
  portVal = HW_RD_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                        CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG94);
  /* Mode 3 PWM */
  portVal &= 0xFFFFFFF3;
  HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                  CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG94,
              portVal);

  /* PRU0_GPO7 PAD95 */
  portVal = HW_RD_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                        CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG95);
  /* Mode 3 PWM */
  portVal &= 0xFFFFFFF3;
  HW_WR_REG32(CSL_PADCFG_CTRL0_CFG0_BASE +
                  CSL_MAIN_PADCFG_CTRL_MMR_CFG0_PADCONFIG95,
              portVal);
}

static void PRUICSS_IsrInit(void) {
  int status;

  mem_config_init = false;
  PRU_IPC_Params pruIpcparams = {
      .pruicssHandle = gPruIcss0Handle,
      .transferCallbackFxn = &PRU_IPC_Isr,
  };
  gPruIpc0Handle = PRU_IPC_open(CONFIG_PRU_IPC0, &pruIpcparams);

  gPruIpc0Handle->attrs->config->blockId = 0;
  uint16_t id_block = PRU_IPC_getBlockId(gPruIpc0Handle);
  uint32_t blockOffset = id_block * gPruIpc0Handle->attrs->blockSizeBytes;
  uint32_t mem_loc =
      (uint32_t)&gPruIpc0Handle->attrs->config->bufferAddrs + blockOffset;

  for (uint8_t i = 0; i < 6; i++) {
    CSL_REG32_WR(mem_loc + (i << 2), 0);
  }
  /* Set the deatime for now to be 5000 */
  CSL_REG32_WR(mem_loc + (7 << 2), 0);
  CSL_REG32_WR(mem_loc + (8 << 2), 0); /* Disable Pwms */

  /* Tell to the PRU that we read this */
  PRUICSS_sendEvent(gPruIcss0Handle, 17);
  status = PRUICSS_setConstantTblEntry(gPruIcss0Handle, 0,
                                       PRUICSS_CONST_TBL_ENTRY_C30, mem_loc);
  DebugP_assert(SystemP_SUCCESS == status);
}

/* Global functions */
void Pru_InitCore(void) {
  int status;
  PRUICSS_custom_Init();

  PRUICSS_IsrInit();

  status = PRUICSS_loadFirmware(gPruIcss0Handle, PRUICSS_PRU0, TestPru_image_0,
                                sizeof(TestPru_image_0));
  DebugP_assert(SystemP_SUCCESS == status);
  PRUICSS_WaitIdle();
  PRUICSS_EnablePwmPorts();
  /* Tell to the PRU that we read this */
  PRUICSS_sendEvent(gPruIcss0Handle, gPruIpc0Handle->attrs->txEventNum);
}
