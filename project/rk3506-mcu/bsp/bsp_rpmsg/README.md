# bsp_rpmsg 使用说明

## 1. 模块作用

`bsp_rpmsg` 对 RPMsg-Lite 做了一层 MCU 侧封装，目标是让业务代码只关心 4 件事：

- 初始化 RPMsg 链路
- 注册本地 endpoint
- 在回调里处理收到的数据
- 按需向 Linux 侧发送数据

当前默认拓扑为：

- Linux 为 master
- RK3506 MCU 为 remote
- link id 固定为 `RL_PLATFORM_SET_LINK_ID(0, 3)`
- 共享内存来自链接脚本符号 `__linux_share_rpmsg_start__` 和 `__linux_share_rpmsg_end__`

## 2. 初始化流程

在主程序里先完成基础硬件初始化，再调用：

```c
RPMsg_Service_Init();
```

这个接口会完成下面几件事：

- 初始化 RPMsg-Lite remote 实例
- 等待 Linux 侧把 link 拉起
- 只打印链路结果

可能输出：

```text
rpmsg link: 1
```

或

```text
rpmsg link: 0
```

如果你的中断链路已经稳定，主循环里不需要轮询 RPMsg。

`RPMsg_Service_Poll()` 仅作为可选兜底接口，适合下面两类场景：

- 调试中断路径时临时辅助定位问题
- 某些极端情况下需要手动补消费 mailbox

可选调用方式如下：

```c
RPMsg_Service_Poll();
```

这个接口不会替代硬件中断，正常业务模式下可以不调用。

## 3. 注册 endpoint

使用 `RPMsg_Register()` 注册一个业务 endpoint。

示例：

```c
static RPMsgInstance *g_rpmsgIns;

static void App_RPMsgCallback(RPMsgInstance *ins)
{
    if (ins == NULL) {
        return;
    }

    /* ins->rx_buff 中是收到的数据，ins->rx_len 是长度 */
}

void App_RPMsgInit(void)
{
    RPMsg_Init_Config_s config;

    config.local_ept = 0x4003;
    config.remote_ept = RPMSG_REMOTE_EPT_DYNAMIC;
    config.ept_name = "rpmsg-mcu0-test";
    config.rpmsg_module_callback = App_RPMsgCallback;
    config.id = NULL;

    g_rpmsgIns = RPMsg_Register(&config);
    HAL_ASSERT(g_rpmsgIns != NULL);
}
```

参数说明：

- `local_ept`：MCU 本地 endpoint 地址
- `remote_ept`：Linux 远端 endpoint 地址
- `ept_name`：用于 nameservice announce 的服务名
- `rpmsg_module_callback`：收到数据后的业务回调
- `id`：业务私有指针，供上层自行使用

## 4. 动态远端地址

如果 Linux 侧 endpoint 地址不是固定值，建议把 `remote_ept` 设置为：

```c
RPMSG_REMOTE_EPT_DYNAMIC
```

这样 `bsp_rpmsg` 会在第一次收到 Linux 报文时，自动把发送方 `src` 记录到 `instance->remote_ept`，后续 `RPMsg_Transmit()` 就可以直接回复。

这意味着：

- 如果使用动态远端地址，必须由 Linux 先发第一帧，MCU 再回复
- 如果 MCU 需要主动首发，就要给 `remote_ept` 配固定值，不能使用 `RPMSG_REMOTE_EPT_DYNAMIC`

如果远端地址已知，也可以在初始化后手动指定：

```c
RPMsg_SetRemoteEndpoint(g_rpmsgIns, 0x400);
```

## 5. 接收数据

收到数据后，`bsp_rpmsg` 会先把 payload 拷贝到实例缓冲区，再调用业务回调：

- `instance->rx_buff`：接收数据缓存
- `instance->rx_len`：接收长度
- `instance->last_rx_src`：最近一次发送方 endpoint

建议回调里只做轻量处理，例如：

- 解析命令字
- 设置标志位
- 拷贝到业务缓冲区

不建议在回调中执行耗时很长的操作。

## 6. 发送数据

发送前先填充 `tx_buff`，再设置长度，最后调用发送接口。

示例：

```c
const char *msg = "hello linux";

memcpy(g_rpmsgIns->tx_buff, msg, strlen(msg) + 1U);
RPMsg_SetTxLen(g_rpmsgIns, strlen(msg) + 1U);

if (RPMsg_Transmit(g_rpmsgIns, RL_BLOCK) == 0U) {
    HAL_DBG_ERR("RPMsg send failed\n");
}
```

发送逻辑说明：

- 如果 `remote_ept` 是固定值，就直接发到该地址
- 如果 `remote_ept == RPMSG_REMOTE_EPT_DYNAMIC`，则优先回复最近一次接收到的 `src`
- 如果还没有收到过 Linux 报文，就无法解析远端地址，此时发送会失败

## 7. 推荐调用顺序

推荐的裸机主流程如下：

```c
int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();

    RPMsg_Service_Init();
    App_RPMsgInit();

    while (1) {
        /* 等待硬件中断唤醒后处理业务逻辑 */
        __WFI();
    }
}
```

## 8. Linux 侧配合建议

- Linux 侧 `ept_name` 要与 MCU announce 的名字一致
- 如果使用 `/dev/rpmsgX` 字符设备，建议保持文件描述符长期打开
- 不建议频繁使用 `echo xxx > /dev/rpmsgX` 做压力测试，因为它会不断打开和关闭 endpoint，容易触发 `msg received with no recipient`

更推荐使用常驻进程或你当前的 `rpmsg_init` 这类方式保持 endpoint 常开，再做读写。


附录1:


```c
/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 Rockchip Electronics Co., Ltd.
 */

#include "hal_bsp.h"
#include "hal_base.h"
#include "bsp_uart.h"
#include "bsp_rpmsg.h"

#include <string.h>

#define APP_RPMSG_LOCAL_EPT     0x4003U
#define APP_RPMSG_SERVICE_NAME  "rpmsg-mcu0-test"
#define APP_RPMSG_REPLY_MSG     "mcu ack"

static RPMsgInstance *g_rpmsgIns;

static void App_RPMsgCallback(RPMsgInstance *ins)
{
    uint32_t txLen;

    if (ins == NULL) {
        return;
    }

    txLen = (uint32_t)strlen(APP_RPMSG_REPLY_MSG) + 1U;
    memcpy(ins->tx_buff, APP_RPMSG_REPLY_MSG, txLen);
    RPMsg_SetTxLen(ins, txLen);
    (void)RPMsg_Transmit(ins, RL_BLOCK);    // 在回调函数中回复Linux
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

int main(void)
{
    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();

    RPMsg_Service_Init();
    App_RPMsgInit();

    while (1) {
        __WFI();
    }
}

int entry(void)
{
    return main();
}

```
