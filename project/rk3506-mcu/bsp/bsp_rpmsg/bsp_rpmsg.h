#ifndef __BSP_RPMSG_H
#define __BSP_RPMSG_H

#include "hal_base.h"
#include "hal_bsp.h"
#include "rpmsg_lite.h"
#include "rpmsg_ns.h"

#define RPMSG_MX_REGISTER_CNT     8U
#define RPMSG_EPT_NAME_LEN_MAX    RL_NS_NAME_SIZE
#define RPMSG_PAYLOAD_SIZE_MAX    RL_BUFFER_PAYLOAD_SIZE
#define RPMSG_REMOTE_EPT_DYNAMIC  RL_ADDR_ANY
#define RPMSG_LINK_WAIT_TIMEOUT_MS 10000U

typedef struct RPMsgInstance RPMsgInstance;
typedef void (*RPMsg_Callback)(RPMsgInstance *ins);

struct RPMsgInstance {
    struct rpmsg_lite_endpoint *ept;
    uint32_t local_ept;
    uint32_t remote_ept;
    uint32_t last_rx_src;
    uint8_t tx_buff[RPMSG_PAYLOAD_SIZE_MAX];
    uint8_t rx_buff[RPMSG_PAYLOAD_SIZE_MAX];
    uint32_t tx_len;
    uint32_t rx_len;
    RPMsg_Callback rpmsg_module_callback;
    void *id;
    char ept_name[RPMSG_EPT_NAME_LEN_MAX];
};

typedef struct {
    uint32_t local_ept;
    uint32_t remote_ept;
    const char *ept_name;
    RPMsg_Callback rpmsg_module_callback;
    void *id;
} RPMsg_Init_Config_s;

/**
 * @brief 初始化 RPMsg-Lite 服务与共享内存链路。
 *
 * 当前默认按 RK3506 MCU(remote) <-> Linux(master) 拓扑初始化，
 * 并等待链路建立完成。
 */
void BSP_RPMSG_Init(void);

/**
 * @brief 查询 RPMsg 链路是否已建立。
 * @return 1 表示链路已建立，0 表示未建立。
 */
uint8_t RPMsg_IsLinkUp(void);

/**
 * @brief 注册一个 RPMsg 业务实例（endpoint）。
 * @param config 注册参数。
 * @return 注册成功返回实例指针，失败返回 NULL。
 */
RPMsgInstance *RPMsg_Register(RPMsg_Init_Config_s *config);

/**
 * @brief 设置实例的远端 endpoint 地址。
 * @param instance 目标实例。
 * @param remote_ept 远端 endpoint 地址。
 */
void RPMsg_SetRemoteEndpoint(RPMsgInstance *instance, uint32_t remote_ept);

/**
 * @brief 修改实例发送长度。
 * @param instance 目标实例。
 * @param length 发送长度，最大不超过 RL_BUFFER_PAYLOAD_SIZE。
 */
void RPMsg_SetTxLen(RPMsgInstance *instance, uint32_t length);

/**
 * @brief 发送当前实例 tx_buff 中的数据。
 *
 * 若 remote_ept 为 RL_ADDR_ANY，则优先使用最近一次收到报文的 src 地址回复。
 *
 * @param instance 目标实例。
 * @param timeout_ms 发送等待超时，单位毫秒。
 * @return 1 表示发送成功，0 表示发送失败。
 */
uint8_t RPMsg_Transmit(RPMsgInstance *instance, uint32_t timeout_ms);

#endif /* __BSP_RPMSG_H */
