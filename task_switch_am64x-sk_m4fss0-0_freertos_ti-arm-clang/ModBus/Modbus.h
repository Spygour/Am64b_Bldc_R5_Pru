#ifndef MODBUS_H
#define MODBUS_H
/* Includes */
#include <stdbool.h>
#include <stdint.h>

/* Definitions */
#define MODBUS_DATA_SIZE 8
/* Data Types*/
typedef struct
{
    uint8_t slave_addr;
    uint8_t function;
    uint8_t data_num;
    uint8_t data_pck[MODBUS_DATA_SIZE];
    uint8_t crc_l;
    uint8_t crc_h;
}MODBUS_PKT_T;
/* Local Variables */

/* Global variables */

/* Local functions */

/* Global functions */
extern void ModBus_Init(void);
#endif
