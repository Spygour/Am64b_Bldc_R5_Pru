/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Ads8688/Ads8688.h"
#include "CurrentCtlr/CurrentCtlr.h"
#include "FreeRTOS.h"
#include "Igd/Igd.h"
#include "PruDriver/PruDriver.h"
#include "WatchdogService/Wd.h"
#include "task.h"
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <kernel/dpl/SemaphoreP.h>
#include <stdlib.h>


#define MAIN_TASK_PRI (configMAX_PRIORITIES - 1)
#define WD_TASK_PRI (configMAX_PRIORITIES - 2)
#define IGD_TASK_PRI (configMAX_PRIORITIES - 3)

#define SHARED_MEM_BUFFER_SIZE 128

#define MAIN_TASK_SIZE (4096)
#define WD_TASK_SIZE (512)
#define IGD_TASK_SIZE (1024)
StackType_t gMainTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t gMainTaskObj;
TaskHandle_t gMainTask;

StackType_t WdTaskStack[WD_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t WdTaskObj;
TaskHandle_t WdTask;

StackType_t IgdTaskStack[IGD_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t IgdTaskObj;
TaskHandle_t IgdTask;

/* semaphore used to indicate that the ISR has finished reading samples */
SemaphoreP_Object gAdcDataRecSem;

void freertos_main(void *args) {
  Ads8688_Init();

  CurrentCtlr_Init();

  Pru_InitCore();

  Igd_Init();

  for (;;) {
    Ads8688_Isr();

    CurrentCtlr_Isr();
  }

  vTaskDelete(NULL);
}

int main(void) {
  System_init();
  Board_init();
  Wd_Init();
  /* This task is created at highest priority, it should create more tasks and
   * then delete itself */
  gMainTask = xTaskCreateStatic(
      freertos_main,   /* Pointer to the function that implements the task. */
      "freertos_main", /* Text name for the task.  This is to facilitate
                          debugging only. */
      MAIN_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t
                          on 32b CPUs */
      NULL,            /* We are not using the task parameter. */
      MAIN_TASK_PRI,   /* task priority, 0 is lowest priority,
                              configMAX_PRIORITIES-1 is highest */
      gMainTaskStack,  /* pointer to stack base */
      &gMainTaskObj);  /* pointer to statically allocated task object memory */
  configASSERT(gMainTask != NULL);
  /* Wd task initiliazation */
  WdTask = xTaskCreateStatic(
      Wd_Task,      /* Pointer to the function that implements the task. */
      "wd_service", /* Text name for the task.  This is to facilitate debugging
                       only. */
      WD_TASK_SIZE, /* Stack depth in units of StackType_t typically uint32_t on
                       32b CPUs */
      NULL,         /* We are not using the task parameter. */
      WD_TASK_PRI,  /* task priority, 0 is lowest priority,
                           configMAX_PRIORITIES-1 is highest */
      WdTaskStack,  /* pointer to stack base */
      &WdTaskObj);  /* pointer to statically allocated task object memory */
  configASSERT(WdTask != NULL);

  /* Igd task initiliazation */
  IgdTask = xTaskCreateStatic(
      Igd_Task,      /* Pointer to the function that implements the task. */
      "igd_service", /* Text name for the task.  This is to facilitate debugging
                       only. */
      IGD_TASK_SIZE, /* Stack depth in units of StackType_t typically uint32_t
                       on 32b CPUs */
      NULL,          /* We are not using the task parameter. */
      IGD_TASK_PRI,  /* task priority, 0 is lowest priority,
                           configMAX_PRIORITIES-1 is highest */
      IgdTaskStack,  /* pointer to stack base */
      &IgdTaskObj);  /* pointer to statically allocated task object memory */
  configASSERT(IgdTask != NULL);
  
  /* Start the scheduler to start the tasks executing. */
  vTaskStartScheduler();

  return 0;
}

void De_Init(void) {
  Drivers_close();
  Board_deinit();
  System_deinit();
}
