# bsp_can

<p align='right'>rk3506-mcu project</p>

# 请注意使用 CAN 设备时务必保证总线只接入 2 个终端电阻

开发板通常已经带有一个终端电阻，部分电机、电调或分析仪也自带终端电阻。  
请确认整条总线最终只保留 2 个终端，否则会导致波形畸变、收发异常、丢帧或 ACK 错误。

## 使用说明

如果你希望新增一个基于 CAN 的 module，推荐在该 module 的结构体内部保存一个 `CANInstance *can_instance` 指针。  
模块初始化时调用 `CAN_Register()` 或兼容接口 `CANRegister()` 完成注册，之后：

- 发送前向 `tx_buff` 填入数据
- 通过 `CAN_SetDLC()` 或 `CANSetDLC()` 设置发送长度
- 调用 `CAN_Transmit()` 或 `CANTransmit()` 发送
- 如果注册时提供了 `can_module_callback`，收到匹配 `rx_id` 的报文后会自动回调

## 代码结构

- `.h` 文件提供外部接口、类型定义和宏
- `.c` 文件实现私有函数、收发流程和中断分发

当前实现风格参考通用 module-instance 模式：

- 使用全局实例表统一管理所有 CAN 实例
- 第一次注册实例时自动拉起 CAN 服务
- 接收中断统一从 FIFO 取包后，再按 `can_handle + rx_id` 分发给对应实例

## 类型定义

```c
#define CAN_MX_REGISTER_CNT 2U
#define DEVICE_CAN_CNT      2U

typedef struct CANInstance CANInstance;
typedef void (*CAN_Callback)(CANInstance *ins);

struct CANInstance {
    struct CAN_REG *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    uint8_t tx_buff[64];
    uint8_t rx_buff[64];
    uint8_t tx_len;
    uint8_t rx_len;
    CAN_Callback can_module_callback;
    void *id;
};

typedef struct {
    struct CAN_REG *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    CAN_Callback can_module_callback;
    void *id;
} CAN_Init_Config_s;
```

- `CAN_MX_REGISTER_CNT` 是当前最大 CAN 实例注册数量。这个值和总线负载有关，设备越多、频率越高，总线越容易拥塞。
- `DEVICE_CAN_CNT` 是当前 MCU 拥有的 CAN 硬件数量。RK3506 这里使用 `CAN0` 和 `CAN1`。
- `CANInstance` 是每个 module 持有的 CAN 实例，保存发送 ID、接收 ID、收发缓冲区、接收长度、回调函数以及可选的上层 `id` 指针。
- `tx_buff/rx_buff` 采用 64 字节缓冲，是因为底层硬件是 CANFD 控制器；但当前发送路径按经典 CAN 数据帧使用，推荐 `tx_len` 保持在 `1~8`。
- `id` 可选，用于把 CAN 实例和上层 module 对象关联起来。需要时可以在回调中强制转换回具体模块类型。

## 外部接口

```c
void CAN_Service_Init(void);
void CAN_SetInterruptEnable(uint32_t interrupt_mask);
CANInstance *CAN_Register(CAN_Init_Config_s *config);
void CAN_SetDLC(CANInstance *instance, uint8_t length);
uint8_t CAN_Transmit(CANInstance *instance, uint32_t timeout_ms);

void CANSetInterruptEnable(uint32_t interrupt_mask);
CANInstance *CANRegister(CAN_Init_Config_s *config);
void CANSetDLC(CANInstance *instance, uint8_t length);
uint8_t CANTransmit(CANInstance *instance, uint32_t timeout_ms);
```

- `CAN_Service_Init()` 用于显式初始化 CAN 服务，建议在 `CAN_Register()` 前调用。
- `CAN_SetInterruptEnable()` 用于配置 TX/RX 中断开关，支持按位组合 `CAN_INTERRUPT_RX` 和 `CAN_INTERRUPT_TX`。
- `CAN_Register()` 用于注册一个 CAN 实例。推荐在 module 初始化函数中调用。
- `CAN_SetDLC()` 用于设置发送帧长度，当前经典 CAN 模式下支持 `1~8`。
- `CAN_Transmit()` 用于发送一帧数据。调用前需要先填好 `tx_buff` 和 `tx_len`。
- `CANSetInterruptEnable / CANRegister / CANSetDLC / CANTransmit` 是为了兼容旧命名风格保留的别名接口，功能与带下划线接口一致。

中断配置宏如下：

```c
#define CAN_INTERRUPT_RX    0x01U
#define CAN_INTERRUPT_TX    0x02U
#define CAN_INTERRUPT_ALL   (CAN_INTERRUPT_RX | CAN_INTERRUPT_TX)
```

- 默认配置是 `CAN_INTERRUPT_RX`，也就是只打开 RX 中断。
- `static uint32_t s_canInterruptEnableMask = CAN_INTERRUPT_RX;`
- 如果希望同时打开 TX 和 RX 中断，请传入 `CAN_INTERRUPT_ALL`。
- 如果希望关闭 TX 中断但保留接收回调，传入 `CAN_INTERRUPT_RX` 即可。
- 如果希望关闭全部中断，可以传入 `0U`。此时发送接口仍会轮询 TX 状态，但接收回调不会再靠中断触发。

推荐在 `CAN_Service_Init()` 之前先设置中断策略；如果在初始化之后调用，驱动也会立即把新的屏蔽策略写回硬件。

常见用法：

```c
/* 默认行为：只开 RX 中断 */

CAN_SetInterruptEnable(CAN_INTERRUPT_RX);
CAN_Service_Init();
```

```c
/* 需要发送完成/失败也走中断时 */
CAN_SetInterruptEnable(CAN_INTERRUPT_ALL);
CAN_Service_Init();
```

```c
/* 已经初始化完成后，也可以动态切换 */
CAN_SetInterruptEnable(CAN_INTERRUPT_TX);
CAN_SetInterruptEnable(0U);
CAN_SetInterruptEnable(CAN_INTERRUPT_RX);
```

推荐的注册方式：

```c
CAN_Init_Config_s config = {
    .can_handle = g_can0Dev.pReg,
    .tx_id = 0x200,
    .rx_id = 0x201,
    .can_module_callback = MotorCallback,
    .id = motor_instance,
};
```

## 私有函数和变量

`.c` 文件中通过静态变量统一管理所有实例：

```c
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = { NULL };
static uint8_t idx = 0U;
```

主要私有函数包括：

- `CANServiceInit()`
- `CAN_SetInterruptEnable()`
- `CAN_ConfigPins()`
- `CAN_InitController()`
- `CAN_DrainRxFifo()`
- `CAN_DispatchRxMessage()`
- `CAN_Common_IRQHandler()`
- `CAN_INTMUX_Adapter()`

它们分别负责：

- 初始化 CAN 硬件、引脚、INTMUX 和中断
- 统一管理 TX/RX 中断的运行期开关
- 配置 RK3506 上 CAN0/CAN1 的 IOMUX/RMIO
- 初始化单个控制器的时钟、模式和中断屏蔽
- 在接收中断中尽快取空 FIFO
- 根据 `can_handle + rx_id` 将报文分发给实例
- 在 ISR 中记录发送完成/失败状态，并驱动接收路径

## 当前实现的接收分发方式

和 STM32 BxCAN 通过硬件过滤器直接按实例分流的做法不同，当前 RK3506 版本采用的是：

1. 硬件侧完成基础接收
2. ISR 从接收 FIFO 取出报文
3. 软件遍历 `can_instance[]`
4. 按 `can_handle + rx_id` 匹配实例
5. 命中后拷贝到实例的 `rx_buff` 并调用回调

这样做的好处是结构简单、可读性好，缺点是实例数量增多时回调分发开销会增加。  
如果后续总线负载持续升高，可以再考虑增加更细粒度的硬件过滤策略。

## 注意事项

- 当前发送路径使用的是标准数据帧，推荐 `DLC <= 8`。
- 默认只打开 RX 中断；如果没有显式开启 TX 中断，`CAN_Transmit()` 会改为轮询 TX 完成/失败状态。
- 如果总线上没有其它有效节点给 ACK，发送邮箱可能会被自动重发占住，最终导致发送超时。
- 如果某个任务周期要求非常严格，`timeout_ms` 不应设置过大，否则等待邮箱/等待发送结果的过程可能影响该任务的时序精度。
- 当前 `CAN_Register()` 会动态分配实例内存，如果后续实例数量固定且非常关注内存碎片，可以再改为静态实例池。
