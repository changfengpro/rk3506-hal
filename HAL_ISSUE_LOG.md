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
