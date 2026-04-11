/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"

#include <stdlib.h>
#include <string.h>

#include "rpmsg_lite.h"
#include "rpmsg_ns.h"
#include "rpmsg_platform.h"

#define MASTER_ID                    ((uint32_t)0)
#define REMOTE_ID                    ((uint32_t)3)
#define RPMSG_HAL_REMOTE_TEST_EPT    0x4003U
#define RPMSG_HAL_REMOTE_TEST_NAME   "rpmsg-mcu0-test"
#define RPMSG_HAL_TEST_MSG           "Rockchip rpmsg linux test!"
#define RPMSG_LINUX_MEM_SIZE         (2UL * RL_VRING_OVERHEAD)
#define RPMSG_LINK_WAIT_TIMEOUT_MS   5000U
#define CAN_INT_ALL_MASK             0x000FFFFFU
#define RPMSG_PERIODIC_TX_ENABLE     0U

extern uint32_t __linux_share_rpmsg_start__[];
extern uint32_t __linux_share_rpmsg_end__[];

#define RPMSG_LINUX_MEM_BASE ((uint32_t)&__linux_share_rpmsg_start__)
#define RPMSG_LINUX_MEM_END  ((uint32_t)&__linux_share_rpmsg_end__)

struct rpmsg_block_t {
    uint32_t len;
    uint8_t buffer[RL_BUFFER_PAYLOAD_SIZE];
};

struct rpmsg_info_t {
    struct rpmsg_lite_instance *instance;
    struct rpmsg_lite_endpoint *ept;
    uint32_t cb_sta;
    void *private;
    uint32_t m_ept_id;
};

static struct rpmsg_info_t g_rpmsg_info;
static struct rpmsg_block_t g_rpmsg_block;

static void rpmsg_share_mem_check(void);
static void rpmsg_quiet_stale_irqs(void);
static void rpmsg_service_pending_mbox(void);
static void rpmsg_ns_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data);
static int32_t remote_ept_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv);
static int rpmsg_wait_link_up_with_timeout(struct rpmsg_info_t *info, uint32_t timeout_ms);
static void rpmsg_linux_test(void);
static void app_rpmsg_send_loop_task(void);

static void rpmsg_share_mem_check(void)
{
    if ((RPMSG_LINUX_MEM_BASE + RPMSG_LINUX_MEM_SIZE) > RPMSG_LINUX_MEM_END) {
        HAL_DBG_ERR("share memory size error!\n");
        while (1) {
            ;
        }
    }
}

static void rpmsg_quiet_stale_irqs(void)
{
#if defined(HAL_CANFD_MODULE_ENABLED)
#if defined(HAL_CRU_MODULE_ENABLED)
    HAL_CRU_ClkEnable(g_can0Dev.pclkGateID);
    HAL_CRU_ClkEnable(g_can1Dev.pclkGateID);
#endif
    WRITE_REG(CAN0->INT_MASK, CAN_INT_ALL_MASK);
    WRITE_REG(CAN0->INT, CAN_INT_ALL_MASK);
    WRITE_REG(CAN1->INT_MASK, CAN_INT_ALL_MASK);
    WRITE_REG(CAN1->INT, CAN_INT_ALL_MASK);
    HAL_INTMUX_DisableIRQ(CAN0_IRQn);
    HAL_INTMUX_DisableIRQ(CAN1_IRQn);
#endif
    HAL_NVIC_ClearPendingIRQ(INTMUX_OUT0_IRQn);
    HAL_NVIC_ClearPendingIRQ(INTMUX_OUT1_IRQn);
    HAL_NVIC_ClearPendingIRQ(INTMUX_OUT2_IRQn);
}

static void rpmsg_service_pending_mbox(void)
{
    if ((MBOX3->A2B_STATUS & 0x1U) != 0U) {
        (void)HAL_MBOX_IrqHandler((int)MAILBOX_BB_3_IRQn, MBOX3);
    }
}

static void rpmsg_ns_cb(uint32_t new_ept, const char *new_ept_name, uint32_t flags, void *user_data)
{
    (void)new_ept;
    (void)new_ept_name;
    (void)flags;
    (void)user_data;
}

static int32_t remote_ept_cb(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    struct rpmsg_info_t *info = (struct rpmsg_info_t *)priv;
    struct rpmsg_block_t *block;

    if ((info == NULL) || (payload == NULL)) {
        return RL_RELEASE;
    }

    block = (struct rpmsg_block_t *)info->private;
    if (block == NULL) {
        return RL_RELEASE;
    }

    info->m_ept_id = src;
    block->len = payload_len;
    if (block->len >= RL_BUFFER_PAYLOAD_SIZE) {
        block->len = RL_BUFFER_PAYLOAD_SIZE - 1U;
    }
    memcpy(block->buffer, payload, block->len);
    block->buffer[block->len] = '\0';
    info->cb_sta = 1U;

    (void)rpmsg_lite_send(info->instance,
                          info->ept,
                          info->m_ept_id,
                          RPMSG_HAL_TEST_MSG,
                          strlen(RPMSG_HAL_TEST_MSG) + 1U,
                          RL_BLOCK);

    return RL_RELEASE;
}

static int rpmsg_wait_link_up_with_timeout(struct rpmsg_info_t *info, uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    if ((info == NULL) || (info->instance == NULL)) {
        return -1;
    }

    while (rpmsg_lite_is_link_up(info->instance) != RL_TRUE) {
        rpmsg_service_pending_mbox();

        if (waited >= timeout_ms) {
            return -1;
        }

        HAL_DelayMs(1);
        waited++;
    }

    return 0;
}

static void rpmsg_linux_test(void)
{
    struct rpmsg_info_t *info = &g_rpmsg_info;
    struct rpmsg_block_t *block = &g_rpmsg_block;
    uint32_t ept_flags;
    void *ns_cb_data = NULL;

    rpmsg_share_mem_check();

    memset(info, 0, sizeof(*info));
    memset(block, 0, sizeof(*block));
    info->private = block;

    info->instance = rpmsg_lite_remote_init((void *)RPMSG_LINUX_MEM_BASE,
                                            RL_PLATFORM_SET_LINK_ID(MASTER_ID, REMOTE_ID),
                                            RL_NO_FLAGS);
    HAL_ASSERT(info->instance != NULL);

    __enable_irq();

    if (rpmsg_wait_link_up_with_timeout(info, RPMSG_LINK_WAIT_TIMEOUT_MS) != 0) {
        HAL_DBG_ERR("rpmsg link: 0\n");
        while (1) {
            HAL_DelayMs(1000);
        }
    }

    HAL_DBG("rpmsg link: 1\n");

    (void)rpmsg_ns_bind(info->instance, rpmsg_ns_cb, &ns_cb_data);

    info->ept = rpmsg_lite_create_ept(info->instance,
                                      RPMSG_HAL_REMOTE_TEST_EPT,
                                      remote_ept_cb,
                                      info);
    HAL_ASSERT(info->ept != NULL);

    ept_flags = RL_NS_CREATE;
    (void)rpmsg_ns_announce(info->instance,
                            info->ept,
                            RPMSG_HAL_REMOTE_TEST_NAME,
                            ept_flags);
}

static void app_rpmsg_send_loop_task(void)
{
#if (RPMSG_PERIODIC_TX_ENABLE == 0U)
    return;
#else
    static uint32_t tick_count = 0U;
    char send_buf[32];
    char num_str[16];
    int idx;
    int num_len;
    uint32_t temp_val;
    struct rpmsg_info_t *info = &g_rpmsg_info;
    int32_t ret;

    if ((info->instance == NULL) || (info->ept == NULL) || (info->cb_sta == 0U)) {
        return;
    }

    strcpy(send_buf, "MCU Tick: ");
    idx = (int)strlen(send_buf);

    temp_val = tick_count++;
    num_len = 0;
    if (temp_val == 0U) {
        num_str[num_len++] = '0';
    } else {
        while (temp_val > 0U) {
            num_str[num_len++] = (char)('0' + (temp_val % 10U));
            temp_val /= 10U;
        }
    }

    while (num_len > 0) {
        send_buf[idx++] = num_str[--num_len];
    }
    send_buf[idx] = '\0';

    ret = rpmsg_lite_send(info->instance,
                          info->ept,
                          info->m_ept_id,
                          send_buf,
                          strlen(send_buf) + 1U,
                          RL_DONT_BLOCK);
    if (ret != RL_SUCCESS) {
        HAL_DBG_ERR("rpmsg loop send error, ret=%d\n", ret);
    }
#endif
}

int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();

    rpmsg_quiet_stale_irqs();

    rpmsg_linux_test();

    while (1) {
        rpmsg_service_pending_mbox();
        app_rpmsg_send_loop_task();
        HAL_DelayMs(1000);
    }
}

int entry(void)
{
    return main();
}
