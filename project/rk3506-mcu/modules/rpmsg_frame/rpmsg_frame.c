/*
 * @Description: 
 * @Author: changfengpro
 * @brief: 
 * @version: 
 * @Date: 2026-04-20 12:25:49
 * @LastEditors:  
 * @LastEditTime: 2026-04-23 17:14:39
 */

#include <stdlib.h>
#include <string.h>
#include "rpmsg_frame.h"
#include "rpmsg_platform.h"
#include "rpmsg_lite.h"      
#include "rpmsg_platform.h"
#include "hal_base.h"      

extern uint32_t __linux_share_rpmsg_start__[];
extern uint32_t __linux_share_rpmsg_end__[];


#define SHM_BASE_ADDR ((uint32_t)__linux_share_rpmsg_start__)

/* A7 和 M0 约定好的固定的端点端口号 */
#define M0_LOCAL_EPT_ADDR  (0x2023)
#define LINUX_DST_EPT_ADDR (0x2027)
#define RPMSG_MASTER_ID    (0U)
#define RPMSG_REMOTE_ID    (3U)
#define RPMSG_LINK_ID      RL_PLATFORM_SET_LINK_ID(RPMSG_MASTER_ID, RPMSG_REMOTE_ID)

#define RPMSG_LINKUP_WAIT_STEP_MS (10U)
#define RPMSG_LINKUP_TIMEOUT_MS   (3000U)

/* 静态上下文实例 */
static struct rpmsg_lite_instance rpmsg_ctxt;
static struct rpmsg_lite_ept_static_context rpmsg_ept_context;

/* 实例指针与端点指针 */
struct rpmsg_lite_instance *rpmsg_frame_instance = NULL;
struct rpmsg_lite_endpoint *rpmsg_frame_ept = NULL;
static uint8_t g_rpmsg_inited = 0U;

/* 全局变量：存放从 Linux 接收到的最新控制指令，供 1ms PID 中断读取 */
System_ControlCmd_s linux_control_cmd = {0};

/* ==========================================================
 * 1. 接收回调函数 (运行在 Mailbox 硬件中断上下文中)
 * ========================================================== */
static int32_t RPMsg_Frame_Recv_Callback(void *payload, uint32_t payload_len, uint32_t src, void *priv)
{
    /* 这里的 payload 是从 A7 发送过来的数据，长度为 payload_len */
    /* src 是发送端的地址，priv 是我们在创建端点时传入的上下文指针 */

    if(payload_len == sizeof(System_ControlCmd_s))
        memcpy(&linux_control_cmd, payload, payload_len);
    else
        HAL_DBG("RPMsg RX Len Error: %d\n", payload_len);

    return RL_RELEASE; // 处理完数据后，告诉 RPMsg-Lite 可以释放这个 Buffer 了
}

/* ==========================================================
 * 2. RPMsg 框架初始化函数
 * ========================================================== */
void RPMsg_Frame_Init(void)
{
    rpmsg_frame_instance = rpmsg_lite_remote_init((void *)SHM_BASE_ADDR, RPMSG_LINK_ID, RL_NO_FLAGS, &rpmsg_ctxt);
    if(rpmsg_frame_instance == NULL)
    {
        // 初始化失败，处理错误
        HAL_DBG("rpmsg_lite_remote_init failed\n");
        while(1);
    }

    while(!rpmsg_lite_is_link_up(rpmsg_frame_instance))
    {
        platform_time_delay(10); // 等待链接建立
        // 等待链接建立
    }
    HAL_DBG("link_up = 1\n");
    rpmsg_frame_ept = rpmsg_lite_create_ept(rpmsg_frame_instance, M0_LOCAL_EPT_ADDR, RPMsg_Frame_Recv_Callback, NULL, &rpmsg_ept_context);
    if(rpmsg_frame_ept == NULL)
    {
        // 创建端点失败，处理错误
        HAL_DBG("rpmsg_lite_create_ept failed\n");
        while(1);
    }

}

void RPMsg_Frame_Send_Telemetry(System_Telemetry_s *telemetry_data)
{
    int32_t ret;

    if (rpmsg_frame_instance == NULL || rpmsg_frame_ept == NULL) {
        return; // 未初始化则直接返回
    }

    ret = rpmsg_lite_send(rpmsg_frame_instance,
                          rpmsg_frame_ept,
                          LINUX_DST_EPT_ADDR,
                          (char *)telemetry_data,
                          sizeof(System_Telemetry_s),
                          RL_DONT_BLOCK);
    if (ret != RL_SUCCESS) {
        HAL_DBG("RPMsg send telemetry failed: %d\n", ret);
    }
}
