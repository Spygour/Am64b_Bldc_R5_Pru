/* Includes */
#include "Modbus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <kernel/dpl/SemaphoreP.h>
#include <math.h>
#include <string.h>

/* Definitions */
#define MODBUS_TASK_PRI (configMAX_PRIORITIES - 2)
#define MODBUS_TASK_WAIT 10000
#define MODBUS_TASK_SIZE (16384U / sizeof(configSTACK_DEPTH_TYPE))
#define MODBUS_PKT_NUM 16
/* Types */
typedef enum { INIT_STATE, WRITE_STATE, READ_STATE, FAIL_STATE } Modbus_Task_st;

/* global variables */
StackType_t ModbusTaskStack[MODBUS_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t ModbusTaskObj;
TaskHandle_t ModbusTask;
Modbus_Task_st Modbus_TaskSt = INIT_STATE;

/* static Variables */
static volatile bool Modbus_ReadOk = false;
static MODBUS_PKT_T Modbus_TxPkt[MODBUS_PKT_NUM];
static MODBUS_PKT_T Modbus_RxPkt[MODBUS_PKT_NUM];

static UART_Transaction Modbus_TxTransmitCfg;
static UART_Transaction Modbus_RxTransmitCfg;

static SemaphoreP_Object gUartWriteDoneSem;
static SemaphoreP_Object gUartReadDoneSem;

/* Static function */
static uint16_t Modbus_CrcCalc(MODBUS_PKT_T *modbuspkt) {
  uint8_t *data = (uint8_t *)modbuspkt; /* Remove the data len */
  uint16_t modbus_crc = 0xFFFF;
  const uint16_t polynomial = 0xA001; // Reversed polynomial

  for (size_t i = 0; i < sizeof(MODBUS_PKT_T) - 2; i++) {
    // Step 2: XOR the next byte into the low byte of the CRC register
    modbus_crc ^= data[i];
    // Loop through all 8 bits of the current byte
    for (int bit = 0; bit < 8; bit++) {
      // Step 4: Check if the Least Significant Bit (LSB) is 1
      if (modbus_crc & 0x0001) {
        // Step 3 & 4a: Shift right and XOR with the polynomial
        modbus_crc = (modbus_crc >> 1) ^ polynomial;
      } else {
        // Step 3 & 4b: Just shift right
        modbus_crc >>= 1;
      }
    }
  }

  return modbus_crc;
}

static void Modbus_Write(void) {
  uint16_t modbus_crc;
  uint32_t baseAddr;

  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(WR_EN_BASE_ADDR);
  GPIO_pinWriteHigh(baseAddr, WR_EN_PIN);

  Modbus_TxPkt[0].function = 0x1;
  Modbus_TxPkt[0].slave_addr = 0x10;
  Modbus_TxPkt[0].data_num = 0x8;
  for (uint8_t i = 0; i < MODBUS_DATA_SIZE; i++) {
    Modbus_TxPkt[0].data_pck[i] = i * 0x10;
  }

  modbus_crc = Modbus_CrcCalc(&Modbus_TxPkt[0]);

  /* Store the Crc */
  Modbus_TxPkt[0].crc_l = (uint8_t)modbus_crc; /* Store the lower bytes */
  Modbus_TxPkt[0].crc_h =
      (uint8_t)(modbus_crc >> 8); /* Store the higher bytes */

  UART_write(gUartHandle[CONFIG_UART0], &Modbus_TxTransmitCfg);
}

static void Modbus_Read(void) {
  Modbus_ReadOk = false;
  uint32_t baseAddr;
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(WR_EN_BASE_ADDR);
  GPIO_pinWriteLow(baseAddr, WR_EN_PIN);
  UART_read(gUartHandle[CONFIG_UART0], &Modbus_RxTransmitCfg);
}

static void Modbus_InitTask(void) {
  UART_Transaction_init(&Modbus_TxTransmitCfg);
  UART_Transaction_init(&Modbus_RxTransmitCfg);
  /* Create the semaphore the isr provides from the uart isr */
  SemaphoreP_constructBinary(&gUartWriteDoneSem, 0);
  SemaphoreP_constructBinary(&gUartReadDoneSem, 0);
  /* Initialize the tx transmit cfg */
  Modbus_TxTransmitCfg.buf = &Modbus_TxPkt[0];
  Modbus_TxTransmitCfg.count = sizeof(MODBUS_PKT_T);

  /* Initialize the rx transmit cfg */
  Modbus_RxTransmitCfg.buf = &Modbus_RxPkt[0];
  Modbus_RxTransmitCfg.count = sizeof(MODBUS_PKT_T);

  Drivers_uartOpen();

  Modbus_TaskSt = INIT_STATE;
}
/* Global functions */
void Modbus_Task(void *args);

void Uart1_RxCb(void) {
  Modbus_ReadOk = true;
  SemaphoreP_post(&gUartReadDoneSem);
}

void Uart1_TxCb(void) { SemaphoreP_post(&gUartWriteDoneSem); }

void Modbus_Init(void) {
  /* Initialize the modbus port */
  uint32_t baseAddr;
  uint32_t RS485_EN;
  /* Read the NFAULT */
  baseAddr = (uint32_t)AddrTranslateP_getLocalAddr(WR_EN_BASE_ADDR);
  RS485_EN = GPIO_pinRead(baseAddr, WR_EN_PIN);
  if (RS485_EN == GPIO_PIN_HIGH) {
    return;
  }
  /* This task is created at highest priority, it should create more tasks and
   * then delete itself */
  ModbusTask = xTaskCreateStatic(
      Modbus_Task,   /* Pointer to the function that implements the task. */
      "modbus_main", /* Text name for the task.  This is to facilitate debugging
                        only. */
      MODBUS_TASK_SIZE, /* Stack depth in units of StackType_t typically
                           uint32_t on 32b CPUs */
      NULL,             /* We are not using the task parameter. */
      MODBUS_TASK_PRI,  /* task priority, 0 is lowest priority,
                           configMAX_PRIORITIES-1 is highest */
      ModbusTaskStack,  /* pointer to stack base */
      &ModbusTaskObj);  /* pointer to statically allocated task object memory */
  configASSERT(ModbusTask != NULL);
}

void Modbus_Task(void *args) {
  Modbus_InitTask();

  for (;;) {

    switch (Modbus_TaskSt) {
    case INIT_STATE: {
      Modbus_Write();
      Modbus_TaskSt = READ_STATE;
    } break;

    case WRITE_STATE: {
      SemaphoreP_pend(&gUartReadDoneSem,
                      ClockP_usecToTicks(1000000)); /* Maximum 1 sec wait */
      if (Modbus_ReadOk) {
        uint16_t crc_calc = Modbus_CrcCalc(&Modbus_RxPkt[0]);
        uint16_t crc_actual = ((uint16_t)Modbus_RxPkt[0].crc_h << 8) |
                              ((uint16_t)Modbus_RxPkt[0].crc_l);
        /* CRC is correct continue */
        if (crc_calc == crc_actual) {
          Modbus_Write();
          Modbus_TaskSt = READ_STATE;
        } else {
          Modbus_Write();
          Modbus_TaskSt = READ_STATE;
        }
      } else {
        Modbus_Write();
        Modbus_TaskSt = READ_STATE;
      }
    } break;

    case READ_STATE: {
      SemaphoreP_pend(
          &gUartWriteDoneSem,
          SystemP_WAIT_FOREVER); /* Wait till the write is finished */
      ClockP_usleep(
          40); /* Provide extra delay till we send completely the data */
      Modbus_Read();
      Modbus_TaskSt = WRITE_STATE;
    } break;

    case FAIL_STATE: {
      Modbus_Write();
      Modbus_TaskSt = READ_STATE;
    } break;

    default:
      break;
    }
    ClockP_usleep(MODBUS_TASK_WAIT);
  }
}
