/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "bsp_can.h"

/********************* Private MACRO Definition ******************************/
// #define TEST_DEMO
// #define DEBUG_FLAG_ADDR  0x03b00000
// #define WRITE_FLAG(val)  (*((volatile uint32_t *)DEBUG_FLAG_ADDR) = (val))
/********************* Private Structure Definition **************************/
/********************* Private Variable Definition ***************************/
static uint32_t count;
static uint8_t can_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
CANInstance *test_can0 = NULL;
CANInstance *test_can1 = NULL;
static CANInstance *s_can0 = NULL;
/********************* Private Function Definition ***************************/

static const char *CAN_ErrTypeName(uint32_t errType)
{
    switch (errType) {
    case 0U:
        return "BIT";
    case 1U:
        return "STUFF";
    case 2U:
        return "FORM";
    case 3U:
        return "ACK";
    case 4U:
        return "CRC";
    default:
        return "UNK";
    }
}

static void CAN_LogDiag(const char *tag)
{
    uint32_t ifg0;
    uint32_t ien0;
    uint32_t rec;
    uint32_t tec;
    uint32_t errCode;
    uint32_t errType;
    uint32_t errDir;
    uint32_t errPhase;
    uint32_t rxState;
    uint32_t txState;
    uint32_t retx0;
    uint32_t retx1;
    const char *errName;

#ifdef INTMUX0
    ifg0 = INTMUX0->INT_FLAG_GROUP[0];
    ien0 = INTMUX0->INT_ENABLE_GROUP[0];
#else
    ifg0 = INTMUX->INT_FLAG_GROUP[0];
    ien0 = INTMUX->INT_ENABLE_GROUP[0];
#endif

    rec = READ_REG(g_can0Dev.pReg->RXERRORCNT);
    tec = READ_REG(g_can0Dev.pReg->TXERRORCNT);
    errCode = READ_REG(g_can0Dev.pReg->ERROR_CODE);
    errType = (errCode & CAN_ERROR_CODE_ERROR_TYPE_MASK) >>
              CAN_ERROR_CODE_ERROR_TYPE_SHIFT;
    errDir = (errCode & CAN_ERROR_CODE_ERROR_DIRECTION_MASK) >>
             CAN_ERROR_CODE_ERROR_DIRECTION_SHIFT;
    errPhase = (errCode & CAN_ERROR_CODE_ERROR_PHASE_MASK) >>
               CAN_ERROR_CODE_ERROR_PHASE_SHIFT;
    rxState = (errCode & CAN_ERROR_CODE_RX_STATE_MASK) >>
              CAN_ERROR_CODE_RX_STATE_SHIFT;
    txState = (errCode & CAN_ERROR_CODE_TX_STATE_MASK) >>
              CAN_ERROR_CODE_TX_STATE_SHIFT;
    retx0 = READ_REG(g_can0Dev.pReg->AUTO_RETX_STATE0);
    retx1 = READ_REG(g_can0Dev.pReg->AUTO_RETX_STATE1);
    errName = (errCode == 0U) ? "NONE" : CAN_ErrTypeName(errType);

    HAL_DBG("%s: ISPR=0x%x IPSR=%d Out=%d Adp=%d Can=%d RxDisp=%d\n",
            tag, NVIC->ISPR[0], __get_IPSR(), g_isr_hit_count,
            g_can_adapter_hit_count, g_can_common_hit_count, g_can_rx_dispatch_count);
    HAL_DBG("CAN ST: IFG0=0x%x IEN0=0x%x MODE=0x%x STATE=0x%x STR=0x%x PM=0x%x ICSR=0x%x\n",
            ifg0, ien0, READ_REG(g_can0Dev.pReg->MODE), READ_REG(g_can0Dev.pReg->STATE),
            READ_REG(g_can0Dev.pReg->STR_STATE), __get_PRIMASK(), SCB->ICSR);
    HAL_DBG("CAN CFG: RXINT=0x%x ATF=0x%x ERRM=0x%x SCLK=%d\n",
            READ_REG(g_can0Dev.pReg->RXINT_CTRL), READ_REG(g_can0Dev.pReg->ATF_CTL),
            READ_REG(g_can0Dev.pReg->ERROR_MASK), HAL_CRU_ClkGetFreq(g_can0Dev.sclkID));
    HAL_DBG("CAN INT: LAST=0x%x INT=0x%x MASK=0x%x CMD=0x%x REC=%d TEC=%d\n",
            g_can_last_isr, READ_REG(g_can0Dev.pReg->INT), READ_REG(g_can0Dev.pReg->INT_MASK),
            READ_REG(g_can0Dev.pReg->CMD), rec, tec);
    HAL_DBG("CAN ERR: CODE=0x%x %s/%s phase=%s RXST=0x%x TXST=0x%x NOACK=%d RETX=%d TXERR=%d ARB=%d\n",
            errCode, errDir ? "RX" : "TX", errName,
            errPhase ? "DATA" : "ARB", rxState, txState,
            (int)((retx0 & CAN_AUTO_RETX_STATE0_NOACK_CNT_MASK) >> CAN_AUTO_RETX_STATE0_NOACK_CNT_SHIFT),
            (int)((retx0 & CAN_AUTO_RETX_STATE0_AUTO_RETX_CNT_MASK) >> CAN_AUTO_RETX_STATE0_AUTO_RETX_CNT_SHIFT),
            (int)((retx1 & CAN_AUTO_RETX_STATE1_TXERR_CNT_MASK) >> CAN_AUTO_RETX_STATE1_TXERR_CNT_SHIFT),
            (int)((retx1 & CAN_AUTO_RETX_STATE1_ARBIT_FAIL_CNT_MASK) >> CAN_AUTO_RETX_STATE1_ARBIT_FAIL_CNT_SHIFT));
}

// extern void app_rpmsg_send_loop_task(void);

/********************* Public Function Definition ****************************/


int main(void)
{
    /* HAL BASE Init */
    HAL_Init();

    /* BSP Init */
    BSP_Init();

    /* INTMUX Init */
    HAL_INTMUX_Init();

    /* UART Init */
    BSP_UART_Init();

    HAL_DBG("g_can0Dev.irqNum = %d\n", g_can0Dev.irqNum);

    CAN_Service_Init();

    __enable_irq();

    /* 回环自测时 rx_id 需要和 tx_id 一致，才能命中回调过滤 */
    CAN_Init_Config_s cfg0 = { .can_handle = g_can0Dev.pReg, .tx_id = 0x200, .rx_id = 0x201 };
    s_can0 = CAN_Register(&cfg0);
    if (s_can0 == NULL) {
        HAL_DBG_ERR("CAN_Register failed, stop.\n");
        while (1) {
            ;
        }
    }

    HAL_DBG("RK3506 CAN System Ready...\n");

    HAL_DBG("\n=======================================\n");
    HAL_DBG("Hello RK3506 MCU - UART3 is ALIVE!!!\n");
    HAL_DBG("CAN Build Tag: 2026-04-09-CAN-EXT-1M-HARDIRQ-NOSELFTEST-NOFILTER\n");
    HAL_DBG("=======================================\n");


#ifdef TEST_DEMO
    test_demo();
#endif

    while (1) {
    
        HAL_DBG("MCU is running, tick: %d\n", count++);
        
        static uint32_t tick = 0;
        uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, tick++};
        memcpy(s_can0->tx_buff, data, 8);
        s_can0->tx_len = 8;

        if (CAN_Transmit(s_can0, 100)) {
            CAN_LogDiag("TX OK");
        } else {
            CAN_LogDiag("TX FAIL");
        }

        HAL_DelayMs(1000);

    }
}

int entry(void)
{
    return main();
}
