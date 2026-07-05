#ifndef IGD_H
#define IGD_H
/* Includes */
#include <stdint.h>
/* Definitions */
/* Data Types*/

/* Local Variables */

/* Global variables */

/* Local functions */

/* Global functions */
extern void Igd_Init(void);
extern void Igd_UpdateVoltageOffset(float *adc_offset, uint8_t size);
extern void Igd_CalculateCurrent(float *updated_current, float *adc_voltage,
                                 uint8_t size);
#endif
