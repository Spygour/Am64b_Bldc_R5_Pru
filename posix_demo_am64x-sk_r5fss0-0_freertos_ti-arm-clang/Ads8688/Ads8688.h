#ifndef ADS8688_H
#define ADS8688_H
/* Includes */
#include <stdint.h>
/* Definitions */
/* Data Types*/

#define ADS8688_CHANNELS 2
#define NOP 0x00
#define STDBY 0x82
#define PWR_DN 0x83
#define RST 0x85
#define AUTO_RST 0xA0
#define MAN_CH0 0xC0
#define MAN_CH1 0xC4
#define MAN_CH2 0xC8
#define MAN_CH3 0xCC
#define MAN_CH4 0xD0
#define MAN_CH5 0xD4
#define MAN_CH6 0xD8
#define MAN_CH7 0xDC
#define MAN_AUX 0xE0

typedef union {
  uint32_t Ads8688_Tx;
  struct {
    unsigned dummyData : 16;
    unsigned command : 16;
  } Ads8688_Txbit;
} Ads8688_TxData_u;

typedef union {
  uint32_t Ads8688_Rx;
  struct {
    unsigned Data : 16;
    unsigned dummyData : 16;
  } Ads8688_Rxbit;
} Ads8688_RxData_u;

typedef struct {
  uint32_t CurrentTime;
  uint32_t PreviousTime;
  uint32_t DeltaTime;
} Ads8688_TimingData_t;
/* Local Variables */

/* Global variables */
extern float Ads8688_AdcVoltage[ADS8688_CHANNELS];

extern Ads8688_TimingData_t Ads8688_TimingData;
/* Local functions */

/* Global functions */
extern void Ads8688_Init(void);
extern void Ads8688_Isr(void);
#endif
