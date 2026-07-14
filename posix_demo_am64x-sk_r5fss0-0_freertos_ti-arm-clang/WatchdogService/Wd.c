/* Includes */
#include "Wd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include <drivers/watchdog.h>
#include <string.h>


/* Definitions */
#define WD_TASK_WAIT (uint32_t)500000
/* Types */

/* Static Variables */
static Watchdog_Handle Wd_Handler;
static uint32_t Wd_Clear_cnt = 0;
/* Global Variables */
static volatile uint32_t Wd_Error = 0;

/* Local functions */
static void Wd_Nmi(Watchdog_Handle handle, void *callbackFxnArgs) {
  Wd_Error++;
  Watchdog_clear(Wd_Handler);
}

/* Global functions */
void Wd_Init(void) {
  Watchdog_Params Params_loc;
  Wd_Error = 0;
  Params_loc.callbackFxn = Wd_Nmi;
  Params_loc.callbackFxnArgs = NULL;
  Params_loc.resetMode = Watchdog_RESET_OFF;
  Params_loc.debugStallMode = Watchdog_DEBUG_STALL_OFF;
  Params_loc.windowSize = Watchdog_WINDOW_100_PERCENT;
  Params_loc.expirationTime = 1000; /* 1 sec */

  Wd_Handler = Watchdog_open(0, &Params_loc);
}

void Wd_Task(void *args) {
  for (;;) {
    Watchdog_clear(Wd_Handler);
    Wd_Clear_cnt++;

    ClockP_usleep(WD_TASK_WAIT);
  }
}
