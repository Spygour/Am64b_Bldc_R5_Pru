/* Includes */
#include "Ads8688.h"
#include "Ads8688_Os.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <stdint.h>
#include <string.h>

/* Definitions */
#define SHARED_MEM_BUFFER_SIZE 128
#define ADS8688_VOLTAGE_MAX 4.096

#define AUTO_SEQ_EN 0x1
#define CH_PWR_DOWN 0x2
#define VADC_RANGE_SELECT 0x5

#define WRITE_REG 0x1
#define READ_REG 0x0

#define ADS8688_UINT16_MAX 0xFFFF

typedef enum { VREF_MAX, VREF_EQUAL, VREF_HALF } ADC_MULTIPLY;

#define ADS8688_BUFFER_SIZE 4

typedef enum {
  IDLE,
  AUTO_SEQ_CFG,
  CH_PWR_DOWN_CFG,
  VADC_RANGE_SELECT_CFG_0,
  VADC_RANGE_SELECT_CFG_1,
  VADC_RANGE_SELECT_CFG_2,
  START_ADC,
  ADC_0,
  ADC_1
} SpiAdcCallback_st;

uint8_t Spi_TxBuffer[ADS8688_BUFFER_SIZE]
    __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
uint8_t Spi_RxBuffer[ADS8688_BUFFER_SIZE]
    __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* semaphore used to indicate that the ISR has finished reading samples */

MCSPI_Handle Spi_Handler;
MCSPI_OpenParams Spi_Params;
MCSPI_Transaction Spi_TransactionConfig;

static volatile bool Spi_Finished = FALSE;

static float Ads8688_VoltageMax = 0;

static uint16_t Ads8688_AdcData[ADS8688_CHANNELS];

float Ads8688_AdcVoltage[ADS8688_CHANNELS];

Ads8688_TimingData_t Ads8688_TimingData;

static volatile SpiAdcCallback_st Ads8688_SpiRxCbSt = AUTO_SEQ_CFG;

SemaphoreP_Object AdcSem;

/* Global Variables */

/* Static functions */
void Ads8688_SpiRxCb(void) {
  Spi_TransactionConfig.timeout = 0;
  switch (Ads8688_SpiRxCbSt) {
  case AUTO_SEQ_CFG: {
    Spi_TxBuffer[1] = (AUTO_SEQ_EN << 1) | WRITE_REG;
    Spi_TxBuffer[0] = 0x7; /* Enable channels 0 to 2 */
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    Ads8688_SpiRxCbSt = CH_PWR_DOWN_CFG;
  } break;

  case CH_PWR_DOWN_CFG: {
    Spi_TxBuffer[1] = (CH_PWR_DOWN << 1) | WRITE_REG;
    Spi_TxBuffer[0] = 0xF8; /* Power down channels from 3 to 7 */
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    Ads8688_SpiRxCbSt = VADC_RANGE_SELECT_CFG_0;
  } break;

  case VADC_RANGE_SELECT_CFG_0: {
    Spi_TxBuffer[1] = (VADC_RANGE_SELECT << 1) | WRITE_REG;
    Spi_TxBuffer[0] = 0x1;
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    Ads8688_SpiRxCbSt = VADC_RANGE_SELECT_CFG_1;
  } break;

  case VADC_RANGE_SELECT_CFG_1: {
    Spi_TxBuffer[1] = ((VADC_RANGE_SELECT + 0x1) << 1) | WRITE_REG;
    Spi_TxBuffer[0] = 0x1;
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    Ads8688_SpiRxCbSt = VADC_RANGE_SELECT_CFG_2;
  } break;

  case VADC_RANGE_SELECT_CFG_2: {
    Spi_TxBuffer[1] = ((VADC_RANGE_SELECT + 0x2) << 1) | WRITE_REG;
    Spi_TxBuffer[0] = 0x1;
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    Ads8688_SpiRxCbSt = IDLE;
  } break;

  case IDLE: {
    Spi_TxBuffer[1] = AUTO_RST;
    Spi_TxBuffer[0] = 0x0;
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    Ads8688_SpiRxCbSt = START_ADC;
  } break;

  case START_ADC: {
    Spi_TxBuffer[1] = NOP;
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    Ads8688_SpiRxCbSt = ADC_0;
  } break;

  case ADC_0: {
    Spi_TxBuffer[1] = AUTO_RST;
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    CacheP_inv(&Spi_RxBuffer[0U], sizeof(Spi_RxBuffer), CacheP_TYPE_ALLD);
    Ads8688_AdcData[0] = ((uint16_t)Spi_RxBuffer[3] << 8) | Spi_RxBuffer[2];
    Ads8688_SpiRxCbSt = ADC_1;
  } break;

  case ADC_1: {
    Spi_TxBuffer[1] = NOP;
    CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
    CacheP_inv(&Spi_RxBuffer[0U], sizeof(Spi_RxBuffer), CacheP_TYPE_ALLD);
    Ads8688_AdcData[1] = ((uint16_t)Spi_RxBuffer[3] << 8) | Spi_RxBuffer[2];
    Ads8688_SpiRxCbSt = ADC_0;
  } break;

  default: {
    break;
  }
  }
}

static void Ads8688_CalculateVoltageMax(ADC_MULTIPLY adc_multi){
  /* Calculate the Multiplier */
  switch (adc_multi) {
  case VREF_MAX: {
    Ads8688_VoltageMax = ((ADS8688_VOLTAGE_MAX * 2.5) * 2);
  } break;

  case VREF_EQUAL: {
    Ads8688_VoltageMax = ((ADS8688_VOLTAGE_MAX * 1.25) * 2);
  } break;

  case VREF_HALF: {
    Ads8688_VoltageMax = ((ADS8688_VOLTAGE_MAX * 0.625) * 2);
  } break;

  default:
    break;
  }
}

static inline void Ads8688_CalculateVoltage(void) {

  /* Now calculate each channel */
  for (uint8_t i = 0; i < ADS8688_CHANNELS; i++) {
    Ads8688_AdcVoltage[i] =
        (float)Ads8688_AdcData[i] * Ads8688_VoltageMax / ADS8688_UINT16_MAX;
    /* Remove half the value */
    Ads8688_AdcVoltage[i] -= (Ads8688_VoltageMax / 2);
  }
}

static void Ads8688_SpiInit(void) {
  int status;

  /* Initialize the timings */
  Ads8688_TimingData.CurrentTime = 0;
  Ads8688_TimingData.PreviousTime = 0;
  Ads8688_TimingData.DeltaTime = 0;
  /* Default value is x1.25 */
  Ads8688_CalculateVoltageMax(VREF_EQUAL);
  /* Init the Spi status of the application */
  Spi_Finished = FALSE;

  /* Create the semaphore the isr provides from the pru isr */
  SemaphoreP_constructBinary(&AdcSem, 0);
  /* Initialize the spi cfg and transaction structs */
  MCSPI_OpenParams_init(&Spi_Params);
  MCSPI_Transaction_init(&Spi_TransactionConfig);

  /* State machine of adc isr due to spi sequence */
  Ads8688_SpiRxCbSt = AUTO_SEQ_CFG;

  /* Initialize the Adc array */
  for (uint8_t i = 0; i < ADS8688_CHANNELS; i++) {
    Ads8688_AdcData[i] = 0u;
  }
  /* Start the Spi communication */
  Spi_Params.msMode = MCSPI_MS_MODE_CONTROLLER;
  Spi_Params.mcspiDmaIndex = 0;
  Spi_Params.transferMode = MCSPI_TRANSFER_MODE_BLOCKING;
  Spi_Params.transferTimeout = 40;
  /* Open the driver */
  Spi_Handler = MCSPI_open(CONFIG_MCSPI0, &Spi_Params);

  /* Initialize the Spi buffers */
  memset((void *)Spi_TxBuffer, 0, sizeof(Spi_TxBuffer));
  memset((void *)Spi_RxBuffer, 0, sizeof(Spi_RxBuffer));
  /* Send 2 bytes */
  Spi_TxBuffer[0] = 0;
  Spi_TxBuffer[1] = RST;
  Spi_TxBuffer[2] = 0;
  Spi_TxBuffer[3] = 0;

  for (uint8_t i = 0; i < ADS8688_BUFFER_SIZE; i++) {
    Spi_RxBuffer[i] = 0;
  }

  /* Cached data update their values */
  CacheP_wb(&Spi_TxBuffer[0U], sizeof(Spi_TxBuffer), CacheP_TYPE_ALLD);
  CacheP_wb(&Spi_RxBuffer[0U], sizeof(Spi_RxBuffer), CacheP_TYPE_ALLD);

  /* Configure the transaction */
  Spi_TransactionConfig.args = NULL;
  Spi_TransactionConfig.channel = CONFIG_MCSPI0;
  Spi_TransactionConfig.dataSize = 16;
  Spi_TransactionConfig.csDisable = 1;
  Spi_TransactionConfig.txBuf = (void *)Spi_TxBuffer;
  Spi_TransactionConfig.rxBuf = (void *)Spi_RxBuffer;
  Spi_TransactionConfig.count = 2;
  Spi_TransactionConfig.timeout = 0;

  /* Send reset command */
  status = MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  DebugP_assert(SystemP_SUCCESS == status);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();

  /* Enable adc channels */
  status = MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  DebugP_assert(SystemP_SUCCESS == status);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();

  /* Power off unused channels */
  status = MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  DebugP_assert(SystemP_SUCCESS == status);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();

  /* Configure Vref for channel 0 */
  status = MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  DebugP_assert(SystemP_SUCCESS == status);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();

  /* Configure Vref for channel 1 */
  status = MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  DebugP_assert(SystemP_SUCCESS == status);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();

  /* Configure Vref for channel 2 */
  status = MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  DebugP_assert(SystemP_SUCCESS == status);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();

  /* Send the auto reset command */
  status = MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  DebugP_assert(SystemP_SUCCESS == status);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();
}

static void Ads8688_Main(void) {
  /* Send the Spi message ADC 0 */
  (void)MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();

  /* Send the Spi message ADC 1 */
  (void)MCSPI_transfer(Spi_Handler, &Spi_TransactionConfig);
  while (Spi_TransactionConfig.status != 0) {
  }
  Ads8688_SpiRxCb();
}

/* Global functions */
void Ads8688_Init(void) {
  Ads8688_SpiInit();

  Ads8688_Main();

  /* Culculate the timings */
  Ads8688_TimingData.CurrentTime = ClockP_getTimeUsec();
  Ads8688_TimingData.PreviousTime = Ads8688_TimingData.CurrentTime;
  Ads8688_TimingData.DeltaTime = 0;

  /* Calculate the Voltages */
  Ads8688_CalculateVoltage();
}

void Ads8688_Isr(void) {
  (void)SemaphoreP_pend(&AdcSem, SystemP_WAIT_FOREVER);
  /* PRU ISR hits, update the adc values*/
  Ads8688_Main();

  /* Update the timings */
  Ads8688_TimingData.CurrentTime = ClockP_getTimeUsec();
  Ads8688_TimingData.DeltaTime =
      Ads8688_TimingData.CurrentTime - Ads8688_TimingData.PreviousTime;
  Ads8688_TimingData.PreviousTime = Ads8688_TimingData.CurrentTime;

  /* Calculate the Voltages */
  Ads8688_CalculateVoltage();
}
