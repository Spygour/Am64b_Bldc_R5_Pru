/* Includes */
#include "Modbus.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "FreeRTOS.h"
#include "task.h"
#include <kernel/dpl/SemaphoreP.h>
#include <math.h>
#include <string.h>

/* Definitions */
#define MODBUS_TASK_PRI  (configMAX_PRIORITIES-2)
#define MODBUS_TASK_WAIT 500000
#define MODBUS_TASK_SIZE (16384U/sizeof(configSTACK_DEPTH_TYPE))
#define MODBUS_PKT_NUM 16
/* Types */
typedef enum
{
    WRITE_STATE,
    READ_STATE
}Modbus_Task_st;

/* global variables */
StackType_t ModbusTaskStack[MODBUS_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t ModbusTaskObj;
TaskHandle_t ModbusTask;
Modbus_Task_st Modbus_TaskSt = READ_STATE;

/* static Variables */
static MODBUS_PKT_T Modbus_TxPkt[MODBUS_PKT_NUM];
static MODBUS_PKT_T Modbus_RxPkt[MODBUS_PKT_NUM];

static UART_Transaction Modbus_TxTransmitCfg;
static UART_Transaction Modbus_RxTransmitCfg;

static SemaphoreP_Object gUartWriteDoneSem;
static SemaphoreP_Object gUartReadDoneSem;

/* Static function */
static void Modbus_CrcCalc(MODBUS_PKT_T *modbuspkt)
{
    uint8_t *modbusPkt_Fixed = (uint8_t *)modbuspkt; /* Remove the data len */
    uint16_t modbus_crc = 0;
    /* Don't include the crc bytes */
    for(uint16_t pos = 0; pos < (sizeof(MODBUS_PKT_T) - 2); pos++)
    {
        modbus_crc ^= modbusPkt_Fixed[pos];

        for(uint8_t i = 0; i < 8; i++)
        {
            if(modbus_crc & 0x0001)
            {
                modbus_crc >>= 1;
                modbus_crc ^= 0xA001;
            }
            else
            {
                modbus_crc >>= 1;
            }
        }
    }

    /* Store the Crc */
    modbuspkt->crc_l = (uint8_t)modbus_crc; /* Store the lower bytes */
    modbuspkt->crc_h = (uint8_t)(modbus_crc >> 8); /* Store the higher bytes */
}

static void Modbus_Write(void) {
    Modbus_TxPkt[0].function = 0x1;
    Modbus_TxPkt[0].slave_addr = 0x10;
    Modbus_TxPkt[0].data_num = 0x8;
    for (uint8_t i = 0; i < MODBUS_DATA_SIZE; i++)
    {
      Modbus_TxPkt[0].data_pck[0] += 0x10;
    }

    Modbus_CrcCalc(&Modbus_TxPkt[0]);

    UART_write(gUartHandle[CONFIG_UART0], &Modbus_TxTransmitCfg);
}

static void Modbus_Read(void) {
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
  Modbus_TxTransmitCfg.count = sizeof(MODBUS_PKT_T);

  Drivers_uartOpen();

  Modbus_TaskSt = READ_STATE;
  Modbus_Write();
}
/* Global functions */
void Modbus_Task(void *args);

void Uart1_RxCb(void)
{
    SemaphoreP_post(&gUartReadDoneSem);
}

void Uart1_TxCb(void)
{
    SemaphoreP_post(&gUartWriteDoneSem);
}

void Modbus_Init(void) {
    /* This task is created at highest priority, it should create more tasks and then delete itself */
    ModbusTask = xTaskCreateStatic(Modbus_Task,   /* Pointer to the function that implements the task. */
                                  "modbus_main", /* Text name for the task.  This is to facilitate debugging only. */
                                  MODBUS_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  MODBUS_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  ModbusTaskStack,  /* pointer to stack base */
                                  &ModbusTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(ModbusTask != NULL);

    /* Start the scheduler to start the tasks executing. */
}

void Modbus_Task(void *args) {
    Modbus_InitTask();

    for(;;) {

        switch(Modbus_TaskSt) {
        case WRITE_STATE:
        {
            SemaphoreP_pend(&gUartReadDoneSem, SystemP_WAIT_FOREVER);
            Modbus_Write();
            Modbus_TaskSt = READ_STATE;
        }
        break;

        case READ_STATE:
        {
            SemaphoreP_pend(&gUartWriteDoneSem, SystemP_WAIT_FOREVER); /* Wait till the write is finished */
            Modbus_Read();
            Modbus_TaskSt = WRITE_STATE;
        }
        break;

        default:
            break;
        }
        ClockP_usleep(MODBUS_TASK_WAIT);
    }
}
