#ifndef PRUDRIVER_H
#define PRUDRIVER_H
/* Includes */
#include <stdbool.h>
#include <stdint.h>

/* Definitions */

/* Data Types*/
typedef struct {
  uint32_t ShareBufferAddr;
} PRU_R5_MEMORY;

/* Local Variables */

/* Global variables */

/* Local functions */

/* Global functions */
extern void Pru_InitCore(void);

extern void Pwm_EnableOutputs(bool enable);
extern void Pwm_SendEvent(void);
extern void Pwm_SetDutycycle(uint32_t *pwmArray);
extern void Pwm_SetDeadTime(uint32_t deadTime);
#endif
