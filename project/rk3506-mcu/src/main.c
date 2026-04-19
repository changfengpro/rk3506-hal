/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "bsp_rpmsg.h"
#include "bsp_can.h"

#include <string.h>

#define RPMSG_TEST
#define CAN_TEST

#define APP_RPMSG_LOCAL_EPT       0x4003U
#define APP_RPMSG_SERVICE_NAME    "rpmsg-mcu0-test"

#define APP_CAN_DEFAULT_TX_ID     0x200U
#define APP_CAN_RX_ID_ANY         CAN_ID_ANY
#define APP_CAN_DLC_MAX           8U
#define APP_CAN_QUEUE_DEPTH       16U

#define APP_CAN_RPMSG_MAGIC       0x314E4143U
#define APP_CAN_RPMSG_VERSION     1U
#define APP_CAN_DIR_MCU_TO_LINUX  0x01U
#define APP_CAN_DIR_LINUX_TO_MCU  0x02U

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t version;
    uint8_t direction;
    uint8_t dlc;
    uint8_t reserved;
    uint32_t can_id;
    uint8_t data[APP_CAN_DLC_MAX];
} AppCanRpmsgFrame;

typedef struct {
    AppCanRpmsgFrame frames[APP_CAN_QUEUE_DEPTH];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} AppCanFrameQueue;

#ifdef RPMSG_TEST
static RPMsgInstance *g_rpmsgIns;
#endif

#ifdef CAN_TEST
static CANInstance *g_canIns;
#endif

static AppCanFrameQueue g_canToRpmsgQueue;
static AppCanFrameQueue g_rpmsgToCanQueue;
static volatile uint32_t g_canToRpmsgDropCnt;
static volatile uint32_t g_rpmsgToCanDropCnt;
static volatile uint32_t g_canTxFailCnt;
static volatile uint32_t g_rpmsgTxFailCnt;

static uint8_t App_ClampCanDlc(uint8_t dlc)
{
    return (dlc > APP_CAN_DLC_MAX) ? APP_CAN_DLC_MAX : dlc;
}

static uint32_t App_IrqLock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    return primask;
}

static void App_IrqUnlock(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void App_CanQueueInit(AppCanFrameQueue *queue)
{
    if (queue == NULL) {
        return;
    }

    memset(queue, 0, sizeof(*queue));
}

static uint8_t App_CanQueuePush(AppCanFrameQueue *queue, const AppCanRpmsgFrame *frame)
{
    uint32_t primask;

    if ((queue == NULL) || (frame == NULL)) {
        return 0U;
    }

    primask = App_IrqLock();

    if (queue->count >= APP_CAN_QUEUE_DEPTH) {
        App_IrqUnlock(primask);
        return 0U;
    }

    queue->frames[queue->tail] = *frame;
    queue->tail = (uint8_t)((queue->tail + 1U) % APP_CAN_QUEUE_DEPTH);
    queue->count++;

    App_IrqUnlock(primask);

    return 1U;
}

static uint8_t App_CanQueuePop(AppCanFrameQueue *queue, AppCanRpmsgFrame *frame)
{
    uint32_t primask;

    if ((queue == NULL) || (frame == NULL)) {
        return 0U;
    }

    primask = App_IrqLock();

    if (queue->count == 0U) {
        App_IrqUnlock(primask);
        return 0U;
    }

    *frame = queue->frames[queue->head];
    queue->head = (uint8_t)((queue->head + 1U) % APP_CAN_QUEUE_DEPTH);
    queue->count--;

    App_IrqUnlock(primask);

    return 1U;
}

#ifdef CAN_TEST
static void App_CanCallback(CANInstance *ins)
{
    AppCanRpmsgFrame frame;
    uint8_t dlc;

    if (ins == NULL) {
        return;
    }

    dlc = App_ClampCanDlc(ins->rx_len);

    memset(&frame, 0, sizeof(frame));
    frame.magic = APP_CAN_RPMSG_MAGIC;
    frame.version = APP_CAN_RPMSG_VERSION;
    frame.direction = APP_CAN_DIR_MCU_TO_LINUX;
    frame.dlc = dlc;
    frame.can_id = ins->rx_msg_id;
    if (dlc > 0U) {
        memcpy(frame.data, ins->rx_buff, dlc);
    }

    if (App_CanQueuePush(&g_canToRpmsgQueue, &frame) == 0U) {
        g_canToRpmsgDropCnt++;
    }
}

static void App_CanInit(void)
{
    CAN_Init_Config_s config;

    memset(&config, 0, sizeof(config));
    config.can_handle = g_can0Dev.pReg;
    config.tx_id = APP_CAN_DEFAULT_TX_ID;
    config.rx_id = APP_CAN_RX_ID_ANY;
    config.can_module_callback = App_CanCallback;

    CANSetInterruptEnable(CAN_INTERRUPT_RX);
    g_canIns = CANRegister(&config);
    HAL_ASSERT(g_canIns != NULL);
    CANSetDLC(g_canIns, APP_CAN_DLC_MAX);
}
#endif

#ifdef RPMSG_TEST
static void App_RPMsgCallback(RPMsgInstance *ins)
{
    AppCanRpmsgFrame frame;

    if (ins == NULL) {
        return;
    }

    if (ins->rx_len != sizeof(AppCanRpmsgFrame)) {
        return;
    }

    memcpy(&frame, ins->rx_buff, sizeof(frame));

    if ((frame.magic != APP_CAN_RPMSG_MAGIC) ||
        (frame.version != APP_CAN_RPMSG_VERSION) ||
        (frame.direction != APP_CAN_DIR_LINUX_TO_MCU) ||
        (frame.dlc > APP_CAN_DLC_MAX)) {
        return;
    }

    if (App_CanQueuePush(&g_rpmsgToCanQueue, &frame) == 0U) {
        g_rpmsgToCanDropCnt++;
    }
}

static void App_RPMsgInit(void)
{
    RPMsg_Init_Config_s config;

    memset(&config, 0, sizeof(config));
    config.local_ept = APP_RPMSG_LOCAL_EPT;
    config.remote_ept = RPMSG_REMOTE_EPT_DYNAMIC;
    config.ept_name = APP_RPMSG_SERVICE_NAME;
    config.rpmsg_module_callback = App_RPMsgCallback;

    g_rpmsgIns = RPMsg_Register(&config);
    HAL_ASSERT(g_rpmsgIns != NULL);
}
#endif

static void App_TransmitCanToLinux(void)
{
#if defined(RPMSG_TEST)
    AppCanRpmsgFrame frame;

    if (g_rpmsgIns == NULL) {
        return;
    }

    if (RPMsg_IsLinkUp() == 0U) {
        return;
    }

    if ((g_rpmsgIns->remote_ept == RPMSG_REMOTE_EPT_DYNAMIC) &&
        (g_rpmsgIns->last_rx_src == RPMSG_REMOTE_EPT_DYNAMIC)) {
        return;
    }

    while (App_CanQueuePop(&g_canToRpmsgQueue, &frame) != 0U) {
        memcpy(g_rpmsgIns->tx_buff, &frame, sizeof(frame));
        RPMsg_SetTxLen(g_rpmsgIns, sizeof(frame));

        if (RPMsg_Transmit(g_rpmsgIns, 0U) == 0U) {
            g_rpmsgTxFailCnt++;
        }
    }
#endif
}

static void App_TransmitLinuxCanToBus(void)
{
#if defined(CAN_TEST)
    AppCanRpmsgFrame frame;
    uint8_t dlc;

    if (g_canIns == NULL) {
        return;
    }

    while (App_CanQueuePop(&g_rpmsgToCanQueue, &frame) != 0U) {
        dlc = App_ClampCanDlc(frame.dlc);

        g_canIns->tx_id = frame.can_id;
        memset(g_canIns->tx_buff, 0, sizeof(g_canIns->tx_buff));
        if (dlc > 0U) {
            memcpy(g_canIns->tx_buff, frame.data, dlc);
        }
        CANSetDLC(g_canIns, dlc);

        if (CANTransmit(g_canIns, 1U) == 0U) {
            g_canTxFailCnt++;
        }
    }
#endif
}

int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();

    App_CanQueueInit(&g_canToRpmsgQueue);
    App_CanQueueInit(&g_rpmsgToCanQueue);

#ifdef RPMSG_TEST
    RPMsg_Service_Init();
    App_RPMsgInit();
#endif

#ifdef CAN_TEST
    BSP_CAN_Init();
    App_CanInit();
#endif
    __enable_irq();

    while (1) {
#ifdef RPMSG_TEST
        RPMsg_Service_Poll();
#endif
        App_TransmitLinuxCanToBus();
        App_TransmitCanToLinux();
    }
}

int entry(void)
{
    return main();
}
