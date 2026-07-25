/* Includes */
#include "Igd.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/gpio.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <string.h>

/* Definitions */
#define IGD_CURRENT_AMPS 2
#define IGD_VREF 3.3
#define IGD_GAIN_DEFAULT 10
#define IGD_GAIN_MAX 40
#define IGD_SHUNT 0.005
#define IGD_TASK_WAIT 100000

/* Types */

/* Static Variables */
static float Igd_voltageOffset[IGD_CURRENT_AMPS];
static float current_div;
/* Global Variables */

/* Function definitions */
void Igd_Enable(bool enable);

/* Local functions */
static void Igd_Main(void) {
  uint32_t baseAddr;

  uint32_t nFault, nOctw;

  /* Read the NFAULT */
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(N_FAULT_BASE_ADDR);
  nFault = GPIO_pinRead(baseAddr, N_FAULT_PIN);
  if (nFault == GPIO_PIN_HIGH) {
    Igd_Enable(false);
  }

  /* Read the overcurrent status */
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(N_OCTW_BASE_ADDR);
  nOctw = GPIO_pinRead(baseAddr, N_OCTW_PIN);
  if (nOctw == GPIO_PIN_HIGH) {
    Igd_Enable(false);
  }
}

void Igd_Task(void *args) {
  for (;;) {

    Igd_Main();

    ClockP_usleep(IGD_TASK_WAIT);
  }
}
/* Global functions */
void Igd_Init(void) {
  uint32_t baseAddr;

  /* Set up the pwm to be 6 phases */
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(M_PWM_BASE_ADDR);
  GPIO_pinWriteLow(baseAddr, M_PWM_PIN);

  /* Enable overcurrent protection */
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(M_OC_BASE_ADDR);
  GPIO_pinWriteHigh(baseAddr, M_OC_PIN);

  /* Disable DC_CAL */
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(DC_CAL_BASE_ADDR);
  GPIO_pinWriteLow(baseAddr, DC_CAL_PIN);

  /* Enable the gate driver */
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(IGD_ENABLE_BASE_ADDR);
  GPIO_pinWriteHigh(baseAddr, IGD_ENABLE_PIN);

  /* Check on what calibration is the Gate Driver */
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(GAIN_BASE_ADDR);
  if (GPIO_pinRead(baseAddr, GAIN_PIN) == GPIO_PIN_HIGH) {
    current_div = IGD_GAIN_MAX * IGD_SHUNT;
  } else {
    current_div = IGD_GAIN_DEFAULT * IGD_SHUNT;
  }
}

void Igd_Enable(bool enable) {
  uint32_t baseAddr;

  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(IGD_ENABLE_BASE_ADDR);

  if (enable) {
    GPIO_pinWriteHigh(baseAddr, IGD_ENABLE_PIN);
  } else {
    GPIO_pinWriteLow(baseAddr, IGD_ENABLE_PIN);
  }
}

void Igd_UpdateVoltageOffset(float *adc_offset, uint8_t size) {
  uint8_t current_num = 0;
  if (size > IGD_CURRENT_AMPS) {
    current_num = IGD_CURRENT_AMPS;
  }
  for (uint8_t i = 0; i < current_num; i++) {
    Igd_voltageOffset[i] = IGD_VREF / 2 + adc_offset[i];
  }
}

void Igd_CalculateCurrent(float *updated_current, float *adc_voltage,
                          uint8_t size) {
  uint8_t current_num = 0;
  if (size > IGD_CURRENT_AMPS) {
    current_num = IGD_CURRENT_AMPS;
  } else {
    current_num = size;
  }

  for (uint8_t i = 0; i < current_num; i++) {
    updated_current[i] = (adc_voltage[i] - Igd_voltageOffset[i]);
    updated_current[i] /= current_div;
  }
}
