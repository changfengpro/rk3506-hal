/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 *
 * RPMsg BSP service: link bring-up, endpoint registration and TX/RX dispatch.
 */

#include "bsp_rpmsg.h"

#include <stdlib.h>
#include <string.h>

#include "rpmsg_platform.h"

extern uint32_t __linux_share_rpmsg_start__[];
extern uint32_t __linux_share_rpmsg_end__[];

#define RPMSG_MASTER_ID          0U
#define RPMSG_REMOTE_ID          3U
#define RPMSG_POOL_SIZE          (2UL * RL_VRING_OVERHEAD)

static struct rpmsg_lite_instance *s_rpmsgInstance = NULL;
static rpmsg_ns_handle s_rpmsgNsHandle = NULL;
static RPMsgInstance *s_rpmsgInstances[RPMSG_MX_REGISTER_CNT] = { NULL };
static uint8_t s_rpmsgInstanceIdx = 0U;
static uint8_t s_rpmsgServiceInitialized = 0U;

/**
 * @brief 默认 nameservice 回调。
 * @param new_ept 新 endpoint 地址。
 * @param new_ept_name endpoint 名称。
 * @param flags nameservice 标志。
 * @param user_data 用户私有参数。
 */
static void RPMsg_DefaultNsBindCallback(uint32_t new_ept,
                                        const char *new_ept_name,
                                        uint32_t flags,
                                        void *user_data);

/**
 * @brief 统一接收分发入口，将 payload 拷贝到实例缓冲区后回调业务层。
 * @param payload 接收负载。
 * @param payload_len 接收长度。
 * @param src 发送方 endpoint 地址。
 * @param priv 注册实例私有指针。
 * @return RL_RELEASE，表示 RPMsg-Lite 可立即释放 RX buffer。
 */
static int32_t RPMsg_RxCallback(void *payload, uint32_t payload_len, uint32_t src, void *priv);

/**
 * @brief 创建 endpoint 后向 Linux 侧做 nameservice announce。
 * @param instance 目标实例。
 */
static void RPMsg_AnnounceEndpoint(RPMsgInstance *instance);

/**
 * @brief 获取当前 RPMsg 共享内存基址。
 * @return 共享内存基址。
 */
static void *RPMsg_GetSharedMemoryBase(void);

/**
 * @brief 获取当前 RPMsg 共享内存长度。
 * @return 共享内存长度。
 */
static uint32_t RPMsg_GetSharedMemorySize(void);

/**
 * @brief 静默清理可能影响 RPMsg 的陈旧中断状态。
 */
static void RPMsg_QuietStaleIrqs(void);

/**
 * @brief 兜底处理 mailbox 中尚未消费的消息。
 */
static void RPMsg_ServicePendingMailbox(void);

/**
 * @brief 等待链路建立完成。
 * @param timeoutMs 超时时间，单位毫秒。
 * @return 0 表示成功，-1 表示超时。
 */
static int32_t RPMsg_WaitLinkUp(uint32_t timeoutMs);

/**
 * @brief 默认 nameservice 回调。
 * @param new_ept 新 endpoint 地址。
 * @param new_ept_name endpoint 名称。
 * @param flags nameservice 标志。
 * @param user_data 用户私有参数。
 */
static void RPMsg_DefaultNsBindCallback(uint32_t new_ept,
                                        const char *new_ept_name,
                                        uint32_t flags,
                                        void *user_data)
{
    (void)new_ept;
    (void)new_ept_name;
    (void)flags;
    (void)user_data;
}

/**
 * @brief 统一接收分发入口，将 payload 拷贝到实例缓冲区后回调业务层。
 * @param payload 接收负载。
 * @param payload_len 接收长度。
 * @param src 发送方 endpoint 地址。
 * @param priv 注册实例私有指针。
 * @return RL_RELEASE，表示 RPMsg-Lite 可立即释放 RX buffer。
 */
static int32_t RPMsg_RxCallback(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    RPMsgInstance *instance = (RPMsgInstance *)priv;

    if ((instance == NULL) || (payload == NULL)) {
        return RL_RELEASE;
    }

    if (payload_len > RPMSG_PAYLOAD_SIZE_MAX) {
        payload_len = RPMSG_PAYLOAD_SIZE_MAX;
    }

    if (payload_len > 0U) {
        memcpy(instance->rx_buff, payload, payload_len);
    }
    instance->rx_len = payload_len;
    instance->last_rx_src = src;

    /*
     * 若 remote_ept 尚未静态配置，则在第一次收到对端报文后自动学习，
     * 便于后续直接回复 Linux 动态分配出的 endpoint。
     */
    if (instance->remote_ept == RPMSG_REMOTE_EPT_DYNAMIC) {
        instance->remote_ept = src;
    }

    if (instance->rpmsg_module_callback != NULL) {
        instance->rpmsg_module_callback(instance);
    }

    return RL_RELEASE;
}

/**
 * @brief 创建 endpoint 后向 Linux 侧做 nameservice announce。
 * @param instance 目标实例。
 */
static void RPMsg_AnnounceEndpoint(RPMsgInstance *instance)
{
    int32_t ret;

    if ((instance == NULL) || (instance->ept == NULL)) {
        return;
    }

    if ((s_rpmsgInstance == NULL) || (s_rpmsgNsHandle == NULL)) {
        return;
    }

    if (instance->ept_name[0] == '\0') {
        return;
    }

    ret = rpmsg_ns_announce(s_rpmsgInstance, instance->ept, instance->ept_name, RL_NS_CREATE);
    if (ret != RL_SUCCESS) {
        HAL_DBG_ERR("RPMsg announce failed, ept=%u, ret=%d\n",
                    (unsigned int)instance->local_ept,
                    (int)ret);
    }
}

/**
 * @brief 获取当前 RPMsg 共享内存基址。
 * @return 共享内存基址。
 */
static void *RPMsg_GetSharedMemoryBase(void)
{
    return (void *)&__linux_share_rpmsg_start__;
}

/**
 * @brief 获取当前 RPMsg 共享内存长度。
 * @return 共享内存长度。
 */
static uint32_t RPMsg_GetSharedMemorySize(void)
{
    uintptr_t start = (uintptr_t)&__linux_share_rpmsg_start__;
    uintptr_t end = (uintptr_t)&__linux_share_rpmsg_end__;

    if (end <= start) {
        return 0U;
    }

    return (uint32_t)(end - start);
}

/**
 * @brief 静默清理可能影响 RPMsg 的陈旧中断状态。
 */
static void RPMsg_QuietStaleIrqs(void)
{
    HAL_INTMUX_ClearPendingIRQ(MAILBOX_BB_3_IRQn);
    HAL_NVIC_ClearPendingIRQ(INTMUX_OUT3_IRQn);
}

/**
 * @brief 兜底处理 mailbox 中尚未消费的消息。
 */
static void RPMsg_ServicePendingMailbox(void)
{
    if ((MBOX3->A2B_STATUS & 0x1U) != 0U) {
        (void)HAL_MBOX_IrqHandler((int)MAILBOX_BB_3_IRQn, MBOX3);
    }
}

/**
 * @brief 等待链路建立完成。
 * @param timeoutMs 超时时间，单位毫秒。
 * @return 0 表示成功，-1 表示超时。
 */
static int32_t RPMsg_WaitLinkUp(uint32_t timeoutMs)
{
    uint32_t waited = 0U;

    if (s_rpmsgInstance == NULL) {
        return -1;
    }

    while (rpmsg_lite_is_link_up(s_rpmsgInstance) != RL_TRUE) {
        RPMsg_ServicePendingMailbox();

        if (waited >= timeoutMs) {
            return -1;
        }

        HAL_DelayMs(1);
        waited++;
    }

    return 0;
}

/**
 * @brief 初始化 RPMsg-Lite 服务与共享内存链路。
 *
 * 当前默认按 RK3506 MCU(remote) <-> Linux(master) 拓扑初始化，
 * 并等待链路建立完成。
 *
 * @return 1 表示初始化成功，0 表示初始化失败（可由上层重试）。
 */
uint8_t BSP_RPMSG_Init(void)
{
    uint32_t shmemSize;
    void *shmemBase;

    if (s_rpmsgServiceInitialized != 0U) {
        return 1U;
    }

    if (s_rpmsgInstance == NULL) {
        shmemBase = RPMsg_GetSharedMemoryBase();
        shmemSize = RPMsg_GetSharedMemorySize();

        HAL_ASSERT(shmemBase != NULL);
        HAL_ASSERT(shmemSize >= RPMSG_POOL_SIZE);

        RPMsg_QuietStaleIrqs();

        s_rpmsgInstance = rpmsg_lite_remote_init(shmemBase,
                                                 RL_PLATFORM_SET_LINK_ID(RPMSG_MASTER_ID, RPMSG_REMOTE_ID),
                                                 RL_NO_FLAGS);
        if (s_rpmsgInstance == NULL) {
            HAL_DBG_ERR("rpmsg remote init failed\n");
            return 0U;
        }

        __enable_irq();
    }

    if (s_rpmsgNsHandle == NULL) {
        s_rpmsgNsHandle = rpmsg_ns_bind(s_rpmsgInstance, RPMsg_DefaultNsBindCallback, NULL);
        if (s_rpmsgNsHandle == NULL) {
            HAL_DBG_ERR("rpmsg ns bind failed\n");
            return 0U;
        }
    }

    if (RPMsg_WaitLinkUp(RPMSG_LINK_WAIT_TIMEOUT_MS) != 0) {
        HAL_DBG_ERR("rpmsg link wait timeout, continue init\n");
    } else {
        HAL_DBG("rpmsg link: 1\n");
    }

    s_rpmsgServiceInitialized = 1U;

    return 1U;
}



/**
 * @brief 查询 RPMsg 链路是否已建立。
 * @return 1 表示链路已建立，0 表示未建立。
 */
uint8_t RPMsg_IsLinkUp(void)
{
    if (s_rpmsgInstance == NULL) {
        return 0U;
    }

    return (rpmsg_lite_is_link_up(s_rpmsgInstance) == RL_TRUE) ? 1U : 0U;
}

/**
 * @brief 注册一个 RPMsg 业务实例（endpoint）。
 * @param config 注册参数。
 * @return 注册成功返回实例指针，失败返回 NULL。
 */
RPMsgInstance *RPMsg_Register(RPMsg_Init_Config_s *config)
{
    RPMsgInstance *instance;

    if ((config == NULL) || (s_rpmsgServiceInitialized == 0U) || (s_rpmsgInstance == NULL)) {
        return NULL;
    }

    if (s_rpmsgInstanceIdx >= RPMSG_MX_REGISTER_CNT) {
        HAL_DBG_ERR("RPMsg instance pool exhausted.\n");
        return NULL;
    }

    instance = (RPMsgInstance *)malloc(sizeof(RPMsgInstance));
    if (instance == NULL) {
        HAL_DBG_ERR("RPMsg malloc instance failed.\n");
        return NULL;
    }

    memset(instance, 0, sizeof(RPMsgInstance));

    instance->local_ept = config->local_ept;
    instance->remote_ept = config->remote_ept;
    instance->last_rx_src = RPMSG_REMOTE_EPT_DYNAMIC;
    instance->rpmsg_module_callback = config->rpmsg_module_callback;
    instance->id = config->id;

    if (config->ept_name != NULL) {
        strncpy(instance->ept_name, config->ept_name, RPMSG_EPT_NAME_LEN_MAX - 1U);
        instance->ept_name[RPMSG_EPT_NAME_LEN_MAX - 1U] = '\0';
    }

    instance->ept = rpmsg_lite_create_ept(s_rpmsgInstance,
                                          instance->local_ept,
                                          RPMsg_RxCallback,
                                          instance);
    if (instance->ept == NULL) {
        HAL_DBG_ERR("RPMsg create ept failed, local_ept=%u\n",
                    (unsigned int)instance->local_ept);
        free(instance);
        return NULL;
    }

    s_rpmsgInstances[s_rpmsgInstanceIdx++] = instance;

    RPMsg_AnnounceEndpoint(instance);

    return instance;
}

/**
 * @brief 设置实例的远端 endpoint 地址。
 * @param instance 目标实例。
 * @param remote_ept 远端 endpoint 地址。
 */
void RPMsg_SetRemoteEndpoint(RPMsgInstance *instance, uint32_t remote_ept)
{
    if (instance == NULL) {
        return;
    }

    instance->remote_ept = remote_ept;
}

/**
 * @brief 修改实例发送长度。
 * @param instance 目标实例。
 * @param length 发送长度，最大不超过 RL_BUFFER_PAYLOAD_SIZE。
 */
void RPMsg_SetTxLen(RPMsgInstance *instance, uint32_t length)
{
    if (instance == NULL) {
        return;
    }

    if (length > RPMSG_PAYLOAD_SIZE_MAX) {
        length = RPMSG_PAYLOAD_SIZE_MAX;
    }

    instance->tx_len = length;
}

/**
 * @brief 发送当前实例 tx_buff 中的数据。
 *
 * 若 remote_ept 为 RL_ADDR_ANY，则优先使用最近一次收到报文的 src 地址回复。
 *
 * @param instance 目标实例。
 * @param timeout_ms 发送等待超时，单位毫秒。
 * @return 1 表示发送成功，0 表示发送失败。
 */
uint8_t RPMsg_Transmit(RPMsgInstance *instance, uint32_t timeout_ms)
{
    int32_t ret;
    uint32_t remoteEpt;

    if ((instance == NULL) || (instance->ept == NULL) || (s_rpmsgInstance == NULL)) {
        return 0U;
    }

    if ((instance->tx_len == 0U) || (instance->tx_len > RPMSG_PAYLOAD_SIZE_MAX)) {
        return 0U;
    }

    remoteEpt = instance->remote_ept;
    if (remoteEpt == RPMSG_REMOTE_EPT_DYNAMIC) {
        remoteEpt = instance->last_rx_src;
    }

    if (remoteEpt == RPMSG_REMOTE_EPT_DYNAMIC) {
        HAL_DBG_ERR("RPMsg remote ept unresolved, local_ept=%u\n",
                    (unsigned int)instance->local_ept);
        return 0U;
    }

    ret = rpmsg_lite_send(s_rpmsgInstance,
                          instance->ept,
                          remoteEpt,
                          (char *)instance->tx_buff,
                          instance->tx_len,
                          timeout_ms);
    if (ret != RL_SUCCESS) {
        HAL_DBG_ERR("RPMsg send failed, local=%u, remote=%u, ret=%d\n",
                    (unsigned int)instance->local_ept,
                    (unsigned int)remoteEpt,
                    (int)ret);
        return 0U;
    }

    return 1U;
}
