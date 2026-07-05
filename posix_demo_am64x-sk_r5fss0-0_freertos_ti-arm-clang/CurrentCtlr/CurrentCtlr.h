#ifndef CURRENTCTLR_H
#define CURRENTCTLR_H
/* Includes */
#include <stdint.h>
/* Definitions */
#define CURRENTCTLR_PHASES 3
/* Data Types*/
typedef struct {
  float Ia;
  float Ib;
  float Ia_prev;
  float Ib_prev;
  float Va;
  float Vb;
} CurrentCtlr_Clarke;

typedef struct {
  float Id;
  float Iq;
  float Iq_desired;
  float Vd;
  float Vq;
  float Vd_prev;
  float Vq_prev;
  float eq;
  float ed;
  float eq_prev;
  float ed_prev;
} CurrentCtlr_Park;

typedef struct {
  float theta;
  float R;
  float L;
  float I[CURRENTCTLR_PHASES];
  float V[CURRENTCTLR_PHASES];
  CurrentCtlr_Clarke clarke;
  CurrentCtlr_Park park;
} CurrentCtlr_MainData_t;

/* Local Variables */

/* Global variables */

/* Local functions */

/* Global functions */
extern void CurrentCtlr_Init(void);
extern void CurrentCtlr_Isr(void);
#endif
