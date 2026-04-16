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
