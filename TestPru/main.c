/**
 * main.c
 */
#include "stdint.h"
#include "am64x/pru_ecap.h"
#include "am64x/pru_cfg.h"
#include "am64x/pru_intc.h"
#include "am64x/pru_iep.h"
#include "intc_map_0.h"

#define HOST_INT (uint32_t)(1 << 31)
#define CONFIG_PRU_IPC0_CONFIG_MEM_OFFSET 0x0
#define CONFIG_PRU_IPC0_RX_INTR_ENABLE 1
#define CONFIG_PRU_IPC0_RX_EVENT 0x20
#define CONFIG_PRU_IPC0_TX_EVENT_CLR 17
#define CONFIG_PRU_IPC0_RX_EVENT_CLR 16

#define DOUBLEBUF_U_0 0
#define DOUBLEBUF_V_0 1
#define DOUBLEBUF_W_0 2
#define DOUBLEBUF_U_1 3
#define DOUBLEBUF_V_1 4
#define DOUBLEBUF_W_1 5
#define DOUBLEBUF_IDX 6
#define DEADTIME      7
#define ENABLE        8

#define PRU_PERIOD_ACTUAL 120000
#define PRU_PERIOD_HALF 60000

#define PRU_PWM_U_H  (uint32_t)(1 << 0)
#define PRU_PWM_V_H  (uint32_t)(1 << 1)
#define PRU_PWM_W_H  (uint32_t)(1 << 2)
#define PRU_PWM_U_L  (uint32_t)(1 << 3)
#define PRU_PWM_V_L  (uint32_t)(1 << 4)
#define PRU_PWM_W_L  (uint32_t)(1 << 5)

#define PRU_RELOAD_BIT  (uint32_t)1
#define PRU_START_U_H (uint32_t)(1 << 7)
#define PRU_END_U_H (uint32_t)(1 << 8)
#define PRU_START_V_H  (uint32_t)(1 << 9)
#define PRU_END_V_H (uint32_t)(1 << 10)
#define PRU_START_W_H (uint32_t)(1 << 11)
#define PRU_END_W_H (uint32_t)(1 << 12)

#define PRU_CLEAR PRU_RELOAD_BIT | PRU_START_U_H | PRU_END_U_H | \
                  PRU_START_V_H | PRU_END_V_H | PRU_START_W_H | PRU_END_W_H

#define PRU_START_U_L (uint32_t)(1 << 8)
#define PRU_END_U_L (uint32_t)(1 << 9)
#define PRU_START_V_L  (uint32_t)(1 << 10)
#define PRU_END_V_L (uint32_t)(1 << 11)
#define PRU_START_W_L (uint32_t)(1 << 12)
#define PRU_END_W_L (uint32_t)(1 << 13)


volatile register uint32_t __R30;
volatile register uint32_t __R31;

typedef enum
{
    ISR_WAIT,
    SEND_DATA
}PRU_MAIN_STATE;

typedef enum
{
    INIT_PHASE,
    LOW_PHASE,
    HIGH_PHASE,
    RESTART_PHASE
}PWM_MAIN_STATE;

typedef struct {
    uint16_t blockId;
    uint8_t  dataSize;
    uint8_t  noOfBuffers;
    uint16_t blockSize;
    uint16_t noOfBlocks;
    uint32_t bufferAddrs[128];
} Config_Mem_Struct;

Config_Mem_Struct MemArray;

extern uint32_t read_c28_value(void);
extern uint32_t read_c29_value(void);
extern uint32_t read_c30_value(void);

extern void enable_rx_isr(uint32_t value);

static PRU_MAIN_STATE pru_main_st = ISR_WAIT;

static PWM_MAIN_STATE pwm_main_st = INIT_PHASE;

static uint32_t pwm_doublebuf = 0;

static uint8_t pwm_enable = 0;

uint32_t Ecap_Capture_Time;

static uint32_t isr_cnt = 0;

void Ecap_Main(void)
{
    switch(pru_main_st)
    {
        case ISR_WAIT:
        {
            if ((__R31 & HOST_INT) == HOST_INT)
            {
                /* Clear the tx event */
                CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = CONFIG_PRU_IPC0_TX_EVENT_CLR;
                /* Start the timer */
                CT_ECAP.ECCTL2_ECCTL1_bit.TSCNTSTP = 1u;
                pru_main_st = SEND_DATA;
            }
        }
        break;

        case SEND_DATA:
        {
            if (CT_ECAP.ECFLG_ECEINT_bit.FLAG_CEVT1 == 1)
            {
                Ecap_Capture_Time = CT_ECAP.CAP1;
                /* Clear the Isr */
                CT_ECAP.ECCLR_bit.CEVT1 = 1U;
                /* Clear Global Isrs */
                CT_ECAP.ECCLR_bit.INT = 1u;
                MemArray.bufferAddrs[0] = Ecap_Capture_Time;
            
                /* Stop the timer */
                CT_ECAP.ECCTL2_ECCTL1_bit.TSCNTSTP = 0u;
                enable_rx_isr(CONFIG_PRU_IPC0_RX_EVENT);
                pru_main_st = ISR_WAIT;
            }
        }
        break;

        default:
        break;
    }
}

int main(void)
{
    /* Wait till the ack is done */
    while (CT_CFG.cgr_reg_bit.icss_stop_req == 1);
    CT_CFG.cgr_reg_bit.iep_clk_stop_ack = 0;

    CT_CFG.cgr_reg_bit.ecap_clk_stop_ack = 0;

    CT_CFG.cgr_reg_bit.uart_clk_stop_ack = 0;

    CT_CFG.cgr_reg_bit.intc_clk_stop_ack = 0;
    CT_CFG.cgr_reg_bit.icss_stop_ack = 0;
    while(CT_CFG.cgr_reg_bit.icss_stop_ack == 1);
    CT_CFG.cgr_reg_bit.icss_stop_ack = 0;
    /* Tell to master core that we are active */
    CT_CFG.cgr_reg_bit.icss_pwr_idle = 0;

    CT_CFG.cgr_reg_bit.uart_clk_en = 0;
    CT_CFG.cgr_reg_bit.ecap_clk_en = 1;

    CT_CFG.cgr_reg_bit.iep_clk_en = 1;

    /* Wait to get kick by the R5 */
    while ((__R31 & HOST_INT) == 0x0);
    /* Clear the tx event */
    CT_INTC.STATUS_CLR_INDEX_REG_bit.STATUS_CLR_INDEX = CONFIG_PRU_IPC0_TX_EVENT_CLR;

    CT_CFG.mii_rt_reg_bit.mii_rt_event_en = 0;

    /* Stop the timer for now */
    CT_ECAP.ECCTL2_ECCTL1_bit.TSCNTSTP = 0u;
    /* Capture mode */
    CT_ECAP.ECCTL2_ECCTL1_bit.CAP_APWM = 0u;
    /* ENABLE REGISTERS LOADS AT CAPTURE EVENT TIME */
    CT_ECAP.ECCTL2_ECCTL1_bit.CAPLDEN = 1U;
    /* Trigger at rising edge */
    CT_ECAP.ECCTL2_ECCTL1_bit.CAP1POL = 0U;

    /* Sync disable */
    CT_ECAP.ECCTL2_ECCTL1_bit.SYNCI_EN = 0u;

    /* Enable Capture isr and global isr */
    CT_ECAP.ECFLG_ECEINT_bit.EN_CEVT1 = 1u;

    /* Stop iep clock */
    CT_IEP1.global_cfg_reg_bit.cnt_enable = 0;
    CT_IEP1.global_cfg_reg_bit.default_inc = 1;
    CT_IEP1.cap_cfg_reg_bit.cap_en = 0;

    CT_IEP1.cmp_status_reg = PRU_CLEAR;

    /* Clear the IEP counter */
    CT_IEP1.count_reg0 = 0;
    CT_IEP1.count_reg1 = 0;
    /* Configure iep clock compare cfg */
    CT_IEP1.cmp_cfg_reg = 0x21F83;

    /* Set up the PWM3 */
    CT_CFG_EXT.pwm3_bit.pwm3_trip_cmp0_en = 1;
    CT_CFG_EXT.pwm3_bit.pwm3_trip_reset = 1;
    CT_CFG_EXT.pwm3_0 = 0x5;
    CT_CFG_EXT.pwm3_1 = 0x5;
    CT_CFG_EXT.pwm3_2 = 0x5;
    /* Configure the period, it should be the double the actual the period */
    CT_IEP1.cmp0_reg0 = PRU_PERIOD_HALF;
    CT_IEP1.cmp0_reg1 = PRU_PERIOD_HALF;

    /* High side signals */
    CT_IEP1.cmp7_reg0 = 0;

    CT_IEP1.cmp9_reg0 = 0;

    CT_IEP1.cmp11_reg0 = 0;

    CT_IEP1.cmp7_reg1 = 0;

    CT_IEP1.cmp9_reg1 = 0;

    CT_IEP1.cmp11_reg1 = 0;

    /* Low side signals */
    CT_IEP1.cmp8_reg0 = 0;

    CT_IEP1.cmp10_reg0 = 0;

    CT_IEP1.cmp12_reg0 = 0;

    CT_IEP1.cmp8_reg1 = 0;

    CT_IEP1.cmp10_reg1 = 0;

    CT_IEP1.cmp12_reg1 = 0;

    CT_IEP1_EXT.count_reset_val_reg0 = 0;
    CT_IEP1_EXT.count_reset_val_reg1 = 0;

    pwm_main_st = INIT_PHASE;
    CT_IEP1.global_cfg_reg_bit.cnt_enable = 1;
    isr_cnt = 0;
    pwm_enable = 0x0;
    pru_main_st = ISR_WAIT;
    while (1)
    {
        if (CT_IEP1.cmp_status_reg & PRU_RELOAD_BIT)
        {
            CT_IEP1.cmp_status_reg = PRU_RELOAD_BIT; /* Write one according with manual just clears one bit */
            switch(pwm_main_st)
            {

                case INIT_PHASE:
                {
                    if (isr_cnt == 1)
                    {
                        isr_cnt = 0;
                    }
                    else
                    {
                        isr_cnt++;
                        /* We want to have isr trigger period 2 times the main period */
                        if (MemArray.bufferAddrs[ENABLE] == 0x1)
                        {
                            pwm_enable = 0x1;
                            CT_CFG_EXT.pwm3_bit.pwm3_trip_reset = 0;

                            pwm_doublebuf = MemArray.bufferAddrs[6];
                            /* Send the isr to the R5 to calculate the next pwm values */
                            enable_rx_isr(CONFIG_PRU_IPC0_RX_EVENT);
                            if (pwm_doublebuf == 0)
                            {
                                /* High side signals */
                                CT_IEP1.cmp7_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_U_0] >> 1);

                                CT_IEP1.cmp9_reg1 = PRU_PERIOD_HALF -  (MemArray.bufferAddrs[DOUBLEBUF_V_0] >> 1);

                                CT_IEP1.cmp11_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_W_0] >> 1);
                            }
                            else
                            {
                                /* High side signals */
                                CT_IEP1.cmp7_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_U_1] >> 1);

                                CT_IEP1.cmp9_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_V_1] >> 1);

                                CT_IEP1.cmp11_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_W_1] >> 1);
                            }
                            /* Low side signals */
                            CT_IEP1.cmp8_reg1 = CT_IEP1.cmp7_reg1 - MemArray.bufferAddrs[DEADTIME];

                            CT_IEP1.cmp10_reg1 = CT_IEP1.cmp9_reg1 - MemArray.bufferAddrs[DEADTIME];

                            CT_IEP1.cmp12_reg1 = CT_IEP1.cmp11_reg1 - MemArray.bufferAddrs[DEADTIME];
                            pwm_main_st = HIGH_PHASE;
                        }
                        else
                        {
                            /* Send the isr to the R5 to calculate the next pwm values */
                            enable_rx_isr(CONFIG_PRU_IPC0_RX_EVENT);
                        }
                    }
                }
                break;

                case LOW_PHASE:
                {
                    pwm_doublebuf = MemArray.bufferAddrs[6];
                    /* Send the isr to the R5 to calculate the next pwm values */
                    enable_rx_isr(CONFIG_PRU_IPC0_RX_EVENT);
                    if (pwm_doublebuf == 0)
                    {
                        /* High side signals */
                        CT_IEP1.cmp7_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_U_0] >> 1);

                        CT_IEP1.cmp9_reg1 = PRU_PERIOD_HALF -  (MemArray.bufferAddrs[DOUBLEBUF_V_0] >> 1);

                        CT_IEP1.cmp11_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_W_0] >> 1);
                    }
                    else 
                    {
                        /* High side signals */
                        CT_IEP1.cmp7_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_U_1] >> 1);

                        CT_IEP1.cmp9_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_V_1] >> 1);

                        CT_IEP1.cmp11_reg1 = PRU_PERIOD_HALF - (MemArray.bufferAddrs[DOUBLEBUF_W_1] >> 1);
                    }
                    /* Low side signals */
                    CT_IEP1.cmp8_reg1 = CT_IEP1.cmp7_reg1 - MemArray.bufferAddrs[DEADTIME];

                    CT_IEP1.cmp10_reg1 = CT_IEP1.cmp9_reg1 - MemArray.bufferAddrs[DEADTIME];

                    CT_IEP1.cmp12_reg1 = CT_IEP1.cmp11_reg1 - MemArray.bufferAddrs[DEADTIME];
                    pwm_main_st = HIGH_PHASE;
                }
                break;

                case HIGH_PHASE:
                {
                    if (pwm_doublebuf == 0)
                    {
                        /* High side signals */
                        CT_IEP1.cmp7_reg1 = (MemArray.bufferAddrs[DOUBLEBUF_U_0] >> 1);

                        CT_IEP1.cmp9_reg1 = (MemArray.bufferAddrs[DOUBLEBUF_V_0] >> 1);

                        CT_IEP1.cmp11_reg1 = (MemArray.bufferAddrs[DOUBLEBUF_W_0] >> 1);
                    }
                    else 
                    {
                        /* High side signals */
                        CT_IEP1.cmp7_reg1 = (MemArray.bufferAddrs[DOUBLEBUF_U_1] >> 1);

                        CT_IEP1.cmp9_reg1 = (MemArray.bufferAddrs[DOUBLEBUF_V_1] >> 1);

                        CT_IEP1.cmp11_reg1 = (MemArray.bufferAddrs[DOUBLEBUF_W_1] >> 1);
                    }
                    /* Low side signals */
                    CT_IEP1.cmp8_reg1 = CT_IEP1.cmp7_reg1 + MemArray.bufferAddrs[DEADTIME];

                    CT_IEP1.cmp10_reg1 = CT_IEP1.cmp9_reg1 + MemArray.bufferAddrs[DEADTIME];

                    CT_IEP1.cmp12_reg1 = CT_IEP1.cmp11_reg1 + MemArray.bufferAddrs[DEADTIME];
                    if (MemArray.bufferAddrs[ENABLE] == 0x0)
                    {
                        pwm_main_st = RESTART_PHASE;
                    }
                    else 
                    {
                        pwm_main_st = LOW_PHASE;
                    }
                }
                break;

                case RESTART_PHASE:
                {
                    /* High side signals */
                    CT_IEP1.cmp7_reg1 = 0;

                    CT_IEP1.cmp9_reg1 = 0;

                    CT_IEP1.cmp11_reg1 = 0;

                    /* Low side signals */
                    CT_IEP1.cmp8_reg1 = 0;

                    CT_IEP1.cmp10_reg1 = 0;

                    CT_IEP1.cmp12_reg1 = 0;
                    CT_CFG_EXT.pwm3_bit.pwm3_trip_reset = 1;
                    pwm_main_st = INIT_PHASE;
                }
                break;

                default:
                break;
            }
        }

        if ((CT_IEP1.cmp_status_reg & PRU_START_U_H) && (pwm_enable == 0x1))
        {
            CT_IEP1.cmp_status_reg = PRU_START_U_H; /* Write one according with manual just clears one bit */
        }

        if ((CT_IEP1.cmp_status_reg & PRU_END_U_H)  && (pwm_enable == 0x1))
        {
            CT_IEP1.cmp_status_reg = PRU_END_U_H; /* Write one according with manual just clears one bit */
        }

        if ((CT_IEP1.cmp_status_reg & PRU_START_V_H)  && (pwm_enable == 0x1))
        {
            CT_IEP1.cmp_status_reg = PRU_START_V_H; /* Write one according with manual just clears one bit */
        }

        if ((CT_IEP1.cmp_status_reg & PRU_END_V_H)  && (pwm_enable == 0x1))
        {
            CT_IEP1.cmp_status_reg = PRU_END_V_H; /* Write one according with manual just clears one bit */
        }

        if ((CT_IEP1.cmp_status_reg & PRU_START_W_H)  && (pwm_enable == 0x1))
        {
            CT_IEP1.cmp_status_reg = PRU_START_W_H; /* Write one according with manual just clears one bit */
        }

        if ((CT_IEP1.cmp_status_reg & PRU_END_W_H)  && (pwm_enable == 0x1))
        {
            CT_IEP1.cmp_status_reg = PRU_END_W_H; /* Write one according with manual just clears one bit */
        }
    }
    __halt();
	return 0;
}
