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

---

## [2026-04-18] 仅使用 BSP_RPMSG_Init 时 Linux 与 MCU RPMsg 不通信

### 现象

- 按要求放弃 `App_RPMsgTryInit()`，改为仅在启动阶段使用 `BSP_RPMSG_Init()`。
- Linux 侧 `rpmsg_frame` 能连接到 `/dev/rpmsg0`，但一段时间内 `rpmsg_fps.log` 显示 `TX/RX = 0`。
- J-Link 现场采样显示 RPMsg 服务状态长期未完成：`s_rpmsgServiceInitialized = 0`。

### 影响

- Linux 命令帧发出后 MCU 不回遥测，双方表现为“设备节点存在但业务不通”。
- 调试上容易误判为 mailbox 或 virtio 驱动异常，增加排障时间。

### 根因

- 关键根因 1：`HAL_ASSERT` 在当前构建配置下可能展开为空宏。
  - 代码若写成 `HAL_ASSERT(BSP_RPMSG_Init() == 1U)` / `HAL_ASSERT(App_RPMsgInit() == 1U)`，
    初始化调用本身会被优化掉（副作用丢失）。
  - 结果是 RPMsg 初始化链路未真实执行。
- 关键根因 2：启动阶段若严格等待 link-up，App 侧 endpoint 注册时机会被阻塞，
  在 Linux 启动时序波动时容易长时间看起来“不通”。

### 修复

- 文件：`project/rk3506-mcu/src/main.c`
  - 改为“先调用再断言”：
    - `rpmsgInitOk = BSP_RPMSG_Init(); HAL_ASSERT(rpmsgInitOk == 1U);`
    - `rpmsgModuleOk = App_RPMsgInit(); HAL_ASSERT(rpmsgModuleOk == 1U);`
  - 保留“只使用 BSP_RPMSG_Init”的初始化策略，未恢复 `App_RPMsgTryInit()` 轮询兜底。
- 文件：`project/rk3506-mcu/bsp/bsp_rpmsg/bsp_rpmsg.c`
  - `BSP_RPMSG_Init()` 调整为：先完成 `rpmsg_ns_bind`，再等待 link-up。
  - `RPMsg_WaitLinkUp` 超时只打印错误并继续初始化流程，不再阻塞启动。

### 验证要点（SSH + J-Link 闭环）

- 编译：`project/rk3506-mcu/GCC` 下 `make -j12` 通过。
- 烧录后 J-Link 采样（同一版固件）：
  - 修复前阶段可见 `s_rpmsgServiceInitialized = 0`。
  - Linux 持续发送命令帧后，`s_rpmsgServiceInitialized` 变为 `1`，`s_rpmsgInstanceIdx` 由 `0` 变 `1`。
- Linux 侧执行 `/root/rpmsg_frame ...` 后，`/root/rpmsg_fps.log` 出现稳定收发：
  - `TX` / `RX` 约 `95~97 fps`
  - `DROP = 0`

### 经验总结

- 在本工程配置中，`HAL_ASSERT` 不能包裹带副作用调用。
- 所有关键初始化应采用“显式调用 + 断言/日志分离”的写法，避免 release 构建被优化掉。

---

## [2026-04-18] RPMsg 初始联调不通（Linux 先启动时假连接导致 MCU 长期收不到命令）

### 现象

- 这是本轮 RPMsg 联调最开始遇到的首个问题。
- 按“先启动 Linux 侧 `/root/rpmsg_frame`，再启动 MCU”顺序测试 RPMsg。
- Linux 侧很快打印：
  - `[Linux] 已连接到 /dev/rpmsg0，监听 MCU(16387) -> LOCAL(1024)`
- 但 MCU 串口同时可见：
  - `rpmsg link wait timeout, continue init`
  - `RPMsg announce failed, ept=...`
- 此时 MCU 侧 `g_appRpmsgCmdCnt = 0`，`App_RPMsgCallback()` 不触发。
- Linux 侧旧版工具在现场可能表现为：
  - `rpmsg_fps.log` 长时间 `TX/RX = 0`
  - 或者保持一个“节点已连上但业务不通”的状态。

### 影响

- Linux 看起来“已有 `/dev/rpmsg0` 且已连接”，但该连接并不一定对应本次新启动的 MCU 实例。
- 调试时容易误判为 MCU 解包逻辑、CRC、mailbox 或 endpoint 地址配置错误。
- 现场如果只盯 `/dev/rpmsg0` 是否存在，很难第一时间定位到“stale endpoint / stale vring”问题。

### 根因

- Linux 先启动时，`rpmsg_frame` 可能连到旧的 RPMsg endpoint / 旧的 virtio 状态。
- 这个 stale 连接会让用户态看到“connected”，但业务侧并没有真正和当前这次启动的 MCU endpoint 建立闭环。
- 因此在坏现场中会出现两类表象：
  - Linux 侧读写看起来已打开，但 `TX/RX` 都不动。
  - MCU 侧 `g_appRpmsgCmdCnt`、`g_dbgLastRpmsgCommand`、`g_dbgLastRpmsgState` 长期不变。
- 本质上不是 `rpmsg_frame` 协议字段错误，而是 Linux 侧 endpoint / vring 状态没有和当前 MCU 实例对齐。

### 修复

- 文件：`/home/rmer/Project/Linux/vanxoak_rk3506_board_support/User/rpmsg_frame.c`
- 对 Linux 测试工具增加两类增强：
  - 新增逐帧遥测落盘：
    - `FRAME_DATA_LOG_FILE = "rpmsg_frame_data.log"`
    - 每收到一帧 `FrameTelemetry_t`，记录 `seq/timestamp/motor_count/motor_id/position/velocity/torque/temp/flags/raw hex`
  - 新增“假连接”自愈：
    - 增加 `tx_attempt_count`
    - 若 endpoint 已连接，且连续 `2` 秒存在发送尝试但没有任何有效接收，则自动：
      - `close_session()`
      - `force_kernel_rebind()`
      - 重新连接 endpoint
- 自愈触发日志为：
  - `[Recovery] endpoint connected but no valid RX for 2 s (tx_ok=0, tx_attempt=479), force rebind.`

### MCU 侧可观测性增强

- 文件：`project/rk3506-mcu/src/main.c`
- 为避免 `-O2` 下 `g_rpmsgIns` 在观察窗口显示 `<outofscope>`，增加调试变量：
  - `g_dbgRpmsgIns`
  - `g_dbgLastRpmsgCommand`
  - `g_dbgLastRpmsgState`
- 在 `App_RPMsgCallback()` 中：
  - 收到并成功解码命令帧后，先保存 `g_dbgLastRpmsgCommand`
  - 回包成功后，再保存 `g_dbgLastRpmsgState`
- 这样即使 IDE 无法直接观察 `g_rpmsgIns`，仍能从全局变量读取最后一次实际收发内容。

### 验证要点（先 Linux 后 MCU 场景）

- Linux 侧先启动：
  - `/root/rpmsg_frame -i 7 -p 65536 -v -12345 -t 321 -k 11 -d 22 -f 0xA55A`
- 在 MCU 尚未真正接上时，Linux 侧先复现坏现场：
  - `TX = 0`
  - `RX = 0`
  - `rpmsg_frame_data.log` 仅有头行，无业务帧
- 随后 Linux 侧自动触发自愈，日志应出现：
  - `endpoint connected but no valid RX for 2 s ... force rebind`
  - `驱动已刷新，Linux Vring 指针已重置`
- 自愈后恢复为稳定收发：
  - `TX` / `RX` 约 `434~436 fps`
  - `DROP = 0`
- Linux 侧逐帧日志成功记录 MCU 回包，例如：
  - `m0{id=7 pos=65536 vel=-12345 torque=321 temp=35 flags=0xA55A}`
- 与此同时，J-Link 读取 MCU 侧计数可见：
  - `g_appRpmsgCmdCnt` 持续增长
  - `g_appRpmsgTxCnt` 持续增长
  - `g_appRpmsgTxErrCnt = 0`

### 经验总结

- 在 RPMsg 联调中，“用户态看到 `/dev/rpmsg0` 已连接”不等于“当前 MCU 实例已经真正建立业务闭环”。
- 对“Linux 先起、MCU 后起”这类时序，最好让 Linux 侧工具具备 stale endpoint 检测与自动 `unbind/bind` 自愈能力。
- 若观察窗口里 `g_rpmsgIns` 显示 `<outofscope>`，优先改看显式保留的调试全局变量，而不是据此判断实例未创建。

---

## [2026-04-18] RPMsg 帧电机数量从 4 扩到 20 后 MCU 进入 HardFault

### 现象

- 将 `project/rk3506-mcu/modules/rpmsg_frame/rpmsg_frame.h` 中的
  `RPMSG_FRAME_MAX_MOTOR_CNT` 从 `4U` 改为 `20U` 后，Linux 与 MCU 重新联调时 MCU 会异常跑飞。
- 现场可见：
  - Linux 侧 `/root/rpmsg_frame` 能连接 `/dev/rpmsg0`，但早期长时间 `RX = 0`
  - MCU 侧业务不工作，J-Link 停机时可能落到 `HardFault_Handler / Default_Handler`
  - 旧版 `TestDemo.elf` 反汇编中，`App_RPMsgCallback()` 仍然表现为旧 4 路帧布局
- 关键现场证据：
  - 修复前 `App_RPMsgCallback()` 反汇编仍是 `cmp r3, #4`
  - 同时栈帧只有 `sub sp, #68`

### 影响

- Linux 下发 RPMsg 命令帧后，MCU 在接收/解包路径上就可能触发栈破坏并进入 HardFault。
- 由于 Linux 侧节点和 endpoint 可能仍然存在，表面上看像是“RPMsg 能连上但业务偶发不通”，容易误判成 payload、CRC 或 mailbox 问题。
- 修改帧定义后，如果仍依赖旧的增量构建结果，后续同类问题还会重复出现。

### 根因

- 真实根因不是 RPMsg payload 超限。
  - 当前 `FrameCommand_t` 与 `FrameTelemetry_t` 在 `20U` 配置下仍小于 `RL_BUFFER_PAYLOAD_SIZE`
- 根因是 MCU 构建系统缺少头文件依赖追踪：
  - `rpmsg_frame.h` 修改后，`rpmsg_frame.o` 重编了
  - 但 `main.o` 没有因为依赖该头文件而自动重编
- 结果导致同一版固件里出现 ABI / 结构体布局失配：
  - `RPMsgFrameGetCommandFrame()` 按新 `FrameCommand_t` 大小拷贝
  - `main.c` 中 `App_RPMsgCallback()` 里的局部变量 `FrameCommand_t command` 仍按旧 4 路大小分配栈空间
- 该失配直接导致：
  - 新帧数据拷贝覆盖旧栈缓冲区
  - 栈被破坏
  - MCU 进入 HardFault

### 修复

- 文件：`project/rk3506-mcu/GCC/Makefile`
- 为 MCU 工程补上头文件依赖生成与包含：
  - `CFLAGS += -MMD -MP`
  - `-include $(OBJS:.o=.d)`
- 随后执行一次真正的全量重编：
  - `make clean && make -j12`
- 这样当 `rpmsg_frame.h` 再次变更时，`main.o`、`rpmsg_frame.o` 等依赖对象都会同步重建，不再出现新旧帧布局混编。

### 验证要点

- 反汇编验证修复前后差异：
  - 修复前：`App_RPMsgCallback()` 中仍有 `cmp r3, #4`，且栈帧为 `sub sp, #68`
  - 修复后：变为 `cmp r3, #20`，且栈帧扩展为 `sub sp, #276`
- J-Link 在线检查：
  - 修复后 MCU 不再停在 `HardFault_Handler`
  - 现场一次停机采样显示 `IPSR = 000 (NoException)`，PC 落在正常延时路径 `HAL_TIMER_GetCount`
  - `g_rpmsgIns` 为非空有效地址
- Linux 实测收发恢复正常：
  - `/root/rpmsg_frame -i 7 -T HT04 -m TORQUE -p 1.5 -v -0.18837 -t 1.25`
  - `rpmsg_fps.log` 稳定出现：
    - `TX: 322 fps | RX: 258 fps | DROP: 0`
    - `TX: 260 fps | RX: 260 fps | DROP: 0`
- 临时探针版 Linux 工具还抓到了 MCU 实际回帧内容：
  - `seq=950 ts=1522139 motor_count=1 crc=0xDF5C`
  - `id=7 pos=98304 vel=-12345 torque=320 temp=35 flags=0x0000`

### 经验总结

- 在这个工程里，修改协议头文件后不能盲信增量编译结果，尤其是结构体直接参与跨模块拷贝时。
- 若现场出现“改了数组长度/帧定义后，逻辑看似能编过但 MCU 一跑就 HardFault”，应优先排查：
  - 是否有依赖该头文件的对象文件未重编
  - 是否存在同一固件内的新旧结构体布局混用
- 对协议类头文件，构建系统必须具备 `.d` 依赖文件追踪，否则很容易出现隐蔽 ABI 问题。

---

## [2026-04-18] RPMsg 与 CAN 并发时 CAN RX 不触发（INTMUX OUT0 分发链路被错误接管）

### 现象

- MCU 工程同时打开 `RPMSG_TEST` 与 `CAN_TEST`。
- Linux 板端 `/root/rpmsg_frame -i 7 -T HT04 -m TORQUE -p 1.0 -v -0.5 -t 1.25` 能稳定运行。
- Linux 侧 `rpmsg_fps.log` 长时间稳定显示 `TX/RX` 同步，约 `459~462 fps`，`DROP = 0`。
- 主机侧 `candump can0` 能持续看到 MCU 发出的 `0x200` 报文，说明 `CAN TX` 正常。
- 但主机侧单发 `cansend can0 201#...` 后，MCU 侧 `App_CanCallback()` 不进入，`g_appCanRxCnt` 不增长，看起来像“CAN 只能发不能收”。

### 影响

- RPMsg 与 CAN 并发联调时，表面上像是 RPMsg 正常、CAN 半瘫痪。
- 由于 `CAN TX` 明明正常，现场很容易先怀疑过滤器、波特率、USB2CAN 或业务回调，而不是中断分发本身。
- 如果只看总线波形，不看 MCU 内部计数，排障会反复绕圈。

### 根因

- 文件：`project/rk3506-mcu/bsp/bsp_can/bsp_can.c`
- 当时 `bsp_can` 接管了 `INTMUX_OUT0_IRQn` 等 `INTMUX_OUTx` 中断入口，改成走自定义 `CAN_INTMUX_OUTx_IRQHandler()`。
- 这层自定义入口内部再调用 `HAL_INTMUX_DirectDispatch()`，但现场固件实际走到的这条调用链没有把中断真正分发到 `CANINTMUXAdapter()`。
- 因此坏现场会出现非常迷惑的状态：
  - `CAN0->INT` 已经挂着 `RX_FINISH`
  - `INTMUX->INT_FLAG_GROUP` 已经置位
  - NVIC 侧对应 `INTMUX_OUT0_IRQn` 也 pending
  - 但 `CANINTMUXAdapter()` 和 `App_CanCallback()` 仍然不执行
- 同一轮联调中，还需要把 `CANAddFilter()` 真正落成 RK3506 的硬件 `ATF` 精确过滤，而不是空函数占位。这样每个 `CANInstance` 都能按 `rx_id` 分配独立的硬件过滤槽，接收行为与旧 bxCAN `IDLIST` 习惯保持一致。

### 修复

- 文件：`project/rk3506-mcu/bsp/bsp_can/bsp_can.c`
- 不再由 `bsp_can` 覆盖 `INTMUX_OUT0/1/2/3_IRQn` 的 NVIC handler。
- `CANConfigIntmuxOutputIRQ()` 不再调用 `HAL_NVIC_SetIRQHandler()` 强行接管 `INTMUX_OUTx`。
- 恢复 HAL 默认分发路径：
  - `HAL_INTMUX_OUTx_Handler() -> INTMUX_Dispatch()`
  - 再由 `HAL_INTMUX_SetIRQHandler(dev->irqNum, CANINTMUXAdapter, NULL)` 把源中断交给 CAN BSP
- `CANAddFilter()` 改为按控制器维护 RK3506 硬件 `ATF` 槽位：
  - `CAN0/CAN1` 各自维护 `filter_idx`
  - 每注册一个实例，就写入一组 `ATF[slot] / ATFM[slot]`
  - 清 `ATF_CTL` 对应 bit，启用该槽位
  - 实现对 `instance->rx_id` 的标准帧精确匹配

### 验证要点

- Linux 板端启动后，`/root/rpmsg_fps.log` 稳定出现：
  - `TX: 462 fps | RX: 462 fps | DROP: 0`
  - `TX: 460 fps | RX: 460 fps | DROP: 0`
- 主机侧 `candump -L can0,200:7FF` 持续看到 MCU 发包，例如：
  - `can0 200#A55A7B8401020304`
  - `can0 200#A55A7C8301020304`
- 主机侧执行：
  - `cansend can0 201#1122334455667788`
- 随后用 J-Link 读取 MCU 变量，可见：
  - `g_can_last_rx_data = 44 33 22 11 88 77 66 55`
  - `g_can_last_rx_dlc = 8`
  - `g_can_last_rx_std_id = 0x201`
  - `g_can_rx_irq_cnt` 增长
  - `g_appCanRxCnt` 增长
- 再执行：
  - `cansend can0 201#DEADBEEF01020304`
- J-Link 再次读取，可见：
  - `g_can_last_rx_data = EF BE AD DE 04 03 02 01`
  - `g_can_rx_irq_cnt` 与 `g_appCanRxCnt` 再次增长
- 这证明：
  - RPMsg 并发运行未受影响
  - CAN `TX` 持续正常
  - CAN `RX` 已经真正走通到 MCU 业务回调

### 经验总结

- 在 RK3506 这套 HAL 里，`INTMUX OUT` 中断不要被 BSP 私自接管，优先复用 HAL 默认 `HAL_INTMUX_OUTx_Handler()`。
- 判断 CAN 接收是否真的打通，不能只看总线和 pending 位，最好同时看 MCU 侧：
  - `g_can_last_rx_data`
  - `g_can_last_rx_std_id`
  - `g_can_rx_irq_cnt`
  - `g_appCanRxCnt`
- 修改固件后再做 J-Link 变量读取时，要以当前 `TestDemo.elf` 重新取地址，不能沿用旧版符号地址。
