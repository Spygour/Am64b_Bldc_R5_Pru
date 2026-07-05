/* Includes */
#include "Igd.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <string.h>

/* Definitions */
#define IGD_CURRENT_AMPS 2
#define IGD_VREF 3.3
#define IGD_GAIN 10
#define IGD_SHUNT 0.005

#define IGD_CURRENT_DIV IGD_GAIN *IGD_SHUNT;
/* Types */

/* Static Variables */
static float Igd_voltageOffset[IGD_CURRENT_AMPS];
/* Global Variables */

/* Local functions */

/* Global functions */
void Igd_Init(void) {}

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
  }
  else
  {
    current_num = size;
  }

  for (uint8_t i = 0; i < current_num; i++) {
    updated_current[i] = (adc_voltage[i] - Igd_voltageOffset[i]);
    updated_current[i] /= IGD_CURRENT_DIV;
  }
}
