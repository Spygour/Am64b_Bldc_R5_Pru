#ifndef IGD_H
#define IGD_H
/* Includes */
#include <stdbool.h>
#include <stdint.h>

/* Definitions */
/* Data Types*/

/* Local Variables */

/* Global variables */
extern bool Igd_Start;
/* Local functions */

/* Global functions */
extern void Igd_Init(void);
extern void Igd_UpdateVoltageOffset(float *adc_offset, uint8_t size);
extern void Igd_CalculateCurrent(float *updated_current, float *adc_voltage,
                                 uint8_t size);
extern void Igd_Enable(bool enable);
extern void Igd_Task(void *args);
#endif
