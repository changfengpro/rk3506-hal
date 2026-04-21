# HAL 问题累计记录

## 说明

- 本文件位于 hal 根目录，用于长期累计问题。
- 之后出现新问题，继续按时间追加，不覆盖历史记录。
- 每条记录建议包含：现象、影响、根因、修复、验证。

---

## [2026-04-16] RPMsg 与 CAN 同时运行时 RPMsg 回包丢失

### 现象

- MCU 工程同时打开 `RPMSG_TEST` 与 `CAN_TEST`。
- Linux 侧 `rpmsg_init` 持续打印 `[Linux TX] hello mcu`。
- Linux 侧长时间无 `[Linux RX] mcu ack`。

### 影响

- RPMsg 回调链路中断，Linux 到 MCU 单向可见，MCU 回复不可见。
- 并发联调时误判为 RPMsg 或 mailbox 本身不稳定。

### 根因

- `BSP_CAN_Init()` 调用了 `CANMaskBootEnabledNVICIrqs()`。
- 旧实现会遍历 `0..NUM_INTERRUPTS-1`，把所有内部 NVIC 中断都关闭并清 pending。
- RPMsg(remote) 依赖 `MAILBOX_BB_3_IRQn`，经 INTMUX 路由到 `INTMUX_OUT3_IRQn`。
- `INTMUX_OUT3_IRQn` 被一起关掉后，RPMsg mailbox 中断无法正常分发，回调不触发。

### 修复（治本）

- 文件：`project/rk3506-mcu/bsp/bsp_can/bsp_can.c`
- 将 `CANMaskBootEnabledNVICIrqs()` 从“全关内部 NVIC”改为“仅处理 CAN 相关中断”：
  - 仅对 `s_canDevs[]` 对应的 CAN 源中断执行 `HAL_INTMUX_DisableIRQ` 和 `HAL_INTMUX_ClearPendingIRQ`。
  - 只清理这些 CAN 源对应的 INTMUX OUT pending。
  - 不再全量关闭 `INTMUX_OUTx`，避免误伤 RPMsg mailbox 路径。

### 应用层临时补丁状态

- 临时补丁 `App_RPMsgRestoreIrqAfterCanInit()` 已删除。
- 现在依赖驱动层修复，不再在 `main.c` 里手动补中断。

### 验证要点

- 观察 Linux `rpmsg_init` 日志：应出现 `[Linux RX] mcu ack`。
- 并发下 RPMsg 与 CAN 可同时运行，不再需要手动开启 `INTMUX_OUT3`。

---

## [2026-04-16] CAN 单帧接收后回调持续刷屏

### 现象

- Linux 侧只执行一次 `cansend can0 201#...`。
- MCU 侧却持续进入 CAN 接收回调，串口反复打印 `can rx ok`。
- 现象在切回 `HAL_CANFD_Receive()` 后仍然存在，不是自定义接收函数独有问题。

### 影响

- 单帧会被上层业务重复处理。
- 容易误判为“中断线抖动”或“控制器不断重复收包”。
- 上层状态机、日志和调试判断都会被同一帧污染。

### 根因

- `project/rk3506-mcu/bsp/bsp_can/bsp_can.c` 的接收排空逻辑把 `STR_STATE.INTM_FRAME_CNT` 当成了“完整 CAN 帧数量”。
- 现场通过 J-Link 读取到 `INT=0x1`、`STR_STATE=0x00428204`，其中 `INTM_FRAME_CNT` 可达到 `33`，明显更像内部存储占用量而不是“33 帧”。
- 在这个误判下，`CANDrainRxFifo()` 会对同一接收存储内容重复执行 `HAL_CANFD_Receive()` 和上层分发，导致回调持续刷屏。
- 同时，之前稳定版本里的 RK3506 RX 配置在重构后丢失，现场寄存器回到默认值：
  - `RXINT_CTRL = 0x100`
  - `WAVE_FILTER_CFG = 0x308`
  - `ATF_CTL = 0x0`
- 这些默认值会让接收行为和已验证稳定版本不一致，进一步放大排查难度。

### 修复

- 文件：`project/rk3506-mcu/bsp/bsp_can/bsp_can.c`
- `CANDrainRxFifo()` 不再以 `INTM_FRAME_CNT == 0` 作为退出条件，改为检查 `CAN_STR_STATE_INTM_EMPTY_MASK`：
  - 只有内部接收存储区为空时才停止继续取包。
- 在 `CANInitController()` 中恢复之前稳定版本的 RK3506 RX 侧配置：
  - `WAVE_FILTER_CFG = 0`
  - `RXINT_CTRL = 1`
  - `ATF_CTL = ATF0_EN`

### 验证要点

- Linux 侧单次执行 `cansend can0 201#AABBCCDD`。
- MCU 侧接收回调只应触发一次，不应再连续刷 `can rx ok`。
- 通过 J-Link 复查运行中寄存器，确认 RX 配置不再回到默认值。

---

## [2026-04-16] HAL_CANFD_Receive/Transmit 对非 4 字节对齐 DLC 处理错误

### 现象

- 使用 `HAL_CANFD_Receive()` 后，`DLC=4/8` 的经典 CAN 报文接收正常。
- 但 `DLC=3/5/6/7` 的报文会出现尾字节异常，用户现场示例包括：
  - `AABBCC` 实际只收到 `BB CC`
  - `AABBCCDDEE` 实际只看到 `AA BB CC DD`
  - `AABBCCDDEEFF11` 实际字节错位
- 这类现象不是 `rx_buff` 拷贝问题，而是在 HAL 取包阶段就已经发生。

### 影响

- 只要经典 CAN 数据长度不是 4 的整数倍，应用层拿到的载荷就可能缺字节或错位。
- 即使上层 `bsp_can` 分发逻辑正确，业务层仍会误判协议、校验和状态位。
- 发送侧同样存在对应问题：尾部不足 4 字节的数据没有被完整写入 TX 寄存器。

### 根因

- 文件：`lib/hal/src/hal_canfd.c`
- `HAL_CANFD_Receive()` 使用 `for (i = 0; i < RxMsg->dlc; i += 4)`，每次固定展开 4 字节：
  - 对非 4 字节对齐 DLC，没有区分“完整 32bit 字”和“剩余字节”。
  - 结果是尾包字节在最后一个 word 上被错误解包，出现缺字节或错位。
- `HAL_CANFD_Transmit()` 也只写入 `TxMsg->dlc / 4` 个完整 32bit word：
  - 对尾部 `1~3` 个字节完全没有额外处理。
  - 因此发送非 4 字节对齐 DLC 时，最后几个字节根本没有下发到硬件寄存器。

### 修复

- 文件：`lib/hal/src/hal_canfd.c`
- `HAL_CANFD_Receive()` 改为分两段处理：
  - 先处理 `fullWords = dlc / 4`
  - 再按 `remBytes = dlc % 4` 单独解包最后一个不完整 word
- `HAL_CANFD_Transmit()` 同步修复：
  - 先写完整 32bit word
  - 再把最后 `1~3` 个字节按硬件字节序拼成一个 word 写入下一个 `FD_TXDATA[]`

### 验证要点

- 重新烧录后验证以下报文：
  - `cansend can0 201#AABBCC`
  - `cansend can0 201#AABBCCDDEE`
  - `cansend can0 201#AABBCCDDEEFF11`
  - `cansend can0 201#AABBCCDDEEFF1122`
- MCU 侧应分别收到完整且顺序正确的载荷：
  - `AA BB CC`
  - `AA BB CC DD EE`
  - `AA BB CC DD EE FF 11`
  - `AA BB CC DD EE FF 11 22`

---

## [2026-04-17] RK3506 CAN 单帧接收偶发触发中断风暴

### 现象

- USB2CAN 侧只发送一次 `cansend can0 201#AABBCCDD`。
- MCU 侧串口却持续刷 `can rx ok`，看起来像“单帧触发了无数次接收中断”。
- 同一套代码有时又只触发一次，现场表现不稳定。
- 用 J-Link 停机抓现场时，PC 常落在 `HAL_UART_SerialOutChar()`，说明 CPU 主要耗在重复打印接收日志。

### 影响

- 单次接收报文会被业务层重复处理。
- 串口日志被大量重复输出淹没，正常现象难以观察。
- 由于现象受上电状态和寄存器残留影响，容易误判为“USB2CAN 抖动”或“HAL_CANFD_Receive 仍然有问题”。

### 根因

- 文件：`project/rk3506-mcu/bsp/bsp_can/bsp_can.c`
- `CANInitController()` 里没有显式恢复 RK3506 RX 路径依赖的 3 个控制器本地寄存器：
  - `RXINT_CTRL`
  - `WAVE_FILTER_CFG`
  - `ATF_CTL`
- 因此程序行为依赖于 CAN 控制器当前残留值：
  - 如果寄存器碰巧保留了之前的“好值”，单帧接收表现正常。
  - 如果寄存器回到默认/坏值，就会出现 RX 存储区长期显示非空、接收中断反复重入。
- J-Link 抓到的坏现场寄存器为：
  - `CAN0->INT = 0x00000081`
  - `CAN0->STR_STATE = 0x0080FE04`
  - `CAN0->RXINT_CTRL = 0x00000100`
  - `CAN0->WAVE_FILTER_CFG = 0x00000308`
  - `CAN0->ATF_CTL = 0x00000000`
- 其中 `STR_STATE` 明显不合理：
  - `INTM_EMPTY = 0`
  - `INTM_LEFT_CNT = 254`
  - `INTM_FRAME_CNT = 64`
- 在这组寄存器状态下，单帧接收后内部 RX 存储一直显示“还有数据”，`RX_FINISH`/`RXSTR_FULL` 会持续出现，最终导致 `can rx ok` 无限刷屏。

### 修复

- 文件：`project/rk3506-mcu/bsp/bsp_can/bsp_can.c`
- 在 `CANInitController()` 的 `HAL_CANFD_Start()` 之后，显式写入稳定版本配置：
  - `WAVE_FILTER_CFG = 0`
  - `RXINT_CTRL = 1`
  - `ATF_CTL = ATF0_EN`
- 这样每次初始化都会把 RX 路径拉回已验证过的稳定状态，不再依赖寄存器残留值。

### 验证要点

- 用 J-Link 在坏现场手工把 3 个寄存器改为：
  - `RXINT_CTRL = 1`
  - `WAVE_FILTER_CFG = 0`
  - `ATF_CTL = ATF0_EN`
- 修改后，串口输出会立刻从“持续刷 `can rx ok`”收敛到正常节奏。
- 随后再次单发 `cansend can0 201#AABBCCDD`，MCU 侧只出现一次 `can rx ok`。
- J-Link 复查修复后寄存器状态：
  - `CAN0->INT = 0x00000000`
  - `CAN0->STR_STATE = 0x00000005`
  - `CAN0->RXINT_CTRL = 0x00000001`
  - `CAN0->WAVE_FILTER_CFG = 0x00000000`
  - `CAN0->ATF_CTL = 0x00000001`


## [2026-04-21] 重构rpmsg时无法打通端点

### 原因

- `rpmsg_frame_instance = rpmsg_lite_remote_init((void *)SHM_BASE_ADDR, RL_PLATFORM_HIGHEST_LINK_ID, RL_NO_FLAGS, &rpmsg_ctxt);`
-  `RL_PLATFORM_HIGHEST_LINK_ID` 应该修改为`#define RPMSG_LINK_ID      RL_PLATFORM_SET_LINK_ID(RPMSG_MASTER_ID, RPMSG_REMOTE_ID)`

- 修改前
```c
int main(void)
{
    uint8_t rpmsg_ready = 0U;

    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();
    RPMsg_Frame_Init(); 
    
    __enable_irq();

}
  ```

修改后
```c
int main(void)
{
    uint8_t rpmsg_ready = 0U;

    HAL_Init();
    BSP_Init();
    HAL_INTMUX_Init();
    BSP_UART_Init();
    
    __enable_irq();

    RPMsg_Frame_Init(); 
}
  ```

  区别在于rpmsg的初始化要开启中断，否则mailbox的通知无法触发，导致link_state=0
  
