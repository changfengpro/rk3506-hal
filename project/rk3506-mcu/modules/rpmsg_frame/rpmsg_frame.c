/*
 * @Description: 
 * @Author: changfengpro
 * @brief: 
 * @version: 
 * @Date: 2026-04-20 12:25:49
 * @LastEditors:  
 * @LastEditTime: 2026-04-21 21:04:54
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

/* 静态上下文实例 */
static struct rpmsg_lite_instance rpmsg_ctxt;
static struct rpmsg_lite_ept_static_context rpmsg_ept_context;

/* 实例指针与端点指针 */
struct rpmsg_lite_instance *rpmsg_frame_instance = NULL;
struct rpmsg_lite_endpoint *rpmsg_frame_ept = NULL;

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
    rpmsg_frame_instance = rpmsg_lite_remote_init((void *)SHM_BASE_ADDR, RL_PLATFORM_HIGHEST_LINK_ID, RL_NO_FLAGS, &rpmsg_ctxt);
    if(rpmsg_frame_instance == NULL)
    {
        // 初始化失败，处理错误
        while(1);
    }

    while(!rpmsg_lite_is_link_up(rpmsg_frame_instance))
    {
        platform_time_delay(10); // 等待链接建立
        // 等待链接建立
    }
    HAL_DBG("link_up = 1");
    rpmsg_frame_ept = rpmsg_lite_create_ept(rpmsg_frame_instance, M0_LOCAL_EPT_ADDR, RPMsg_Frame_Recv_Callback, NULL, &rpmsg_ept_context);
    if(rpmsg_frame_ept == NULL)
    {
        // 创建端点失败，处理错误
        HAL_DBG("rpmsg_lite_create_ept failed");
        while(1);
    }

}

void RPMsg_Frame_Send_Telemetry(System_Telemetry_s *telemetry_data)
{
    if (rpmsg_frame_instance == NULL || rpmsg_frame_ept == NULL) {
        return; // 未初始化则直接返回
    }

    uint32_t tx_size = 0;

    // 1. 从共享内存池中直接申请一块发送 Buffer (Zero-Copy 机制)
    // 使用 RL_DONT_BLOCK非阻塞方式申请，如果没有可用 Buffer 则直接返回错误
    void *tx_buf = rpmsg_lite_alloc_tx_buffer(rpmsg_frame_instance, &tx_size, RL_DONT_BLOCK);

    if(tx_buf != NULL)
    {
        if(tx_size >= sizeof(System_Telemetry_s))
        {
            memcpy(tx_buf, telemetry_data, sizeof(System_Telemetry_s)); // 将数据直接填充到申请的发送 Buffer 中

            rpmsg_lite_send_nocopy(rpmsg_frame_instance, rpmsg_frame_ept, LINUX_DST_EPT_ADDR, tx_buf, sizeof(System_Telemetry_s));

            
        }
        else
        {
            RL_ASSERT(0); // 申请到的 Buffer 太小，无法发送数据，触发断言
        }
    }

    else
    {
        // 没有可用的发送 Buffer，可能是因为发送队列满了，可以选择丢弃这次数据或者记录日志
        // 这里我们选择丢弃这次数据并返回
        HAL_DBG("No available tx buffer, telemetry data dropped");
    }
}
