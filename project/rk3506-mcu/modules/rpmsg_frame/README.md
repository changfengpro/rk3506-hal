# rpmsg_frame 使用说明

## 1. 模块职责

`rpmsg_frame` 是建立在 `bsp_rpmsg` 之上的业务帧层。

它只负责 3 件事：

- 接收 Linux 发来的原始 RPMsg payload
- 按 `FrameCommand_t` 做长度和 CRC 校验，并缓存成命令帧
- 把 App 层填写好的 `FrameTelemetry_t` 补上 `seq/timestamp/crc16` 后发回 Linux

它不负责：

- 初始化底层 RPMsg 链路
- 决定电机控制逻辑
- 生成电机状态数据，状态值由 `main.c` 等应用层填写

当前相关源码位置：

- MCU transport 层：`project/rk3506-mcu/bsp/bsp_rpmsg/`
- MCU 帧层：`project/rk3506-mcu/modules/rpmsg_frame/`
- MCU App 示例：`project/rk3506-mcu/src/main.c`
- Linux 对端工具：`/home/rmer/Project/Linux/vanxoak_rk3506_board_support/User/rpmsg_frame.c`

## 2. 当前帧定义

当前命令帧已经去掉 PID 相关字段，不再使用 `kp_q8 / kd_q8 / ctrl_flags`。

MCU 侧真实定义在 `project/rk3506-mcu/modules/rpmsg_frame/rpmsg_frame.h`：

```c
#define RPMSG_FRAME_MAX_MOTOR_CNT 4U
#define RPMSG_FRAME_Q8_SCALE 256L
#define RPMSG_FRAME_Q16_SCALE 65536L

typedef enum {
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
    LK9025,
    HT04,
} Motor_Type_e;

typedef enum {
    MOTOR_CONTROL_MODE_NONE = 0,
    MOTOR_CONTROL_MODE_TORQUE,
    MOTOR_CONTROL_MODE_VELOCITY,
    MOTOR_CONTROL_MODE_POSITION,
} Motor_Control_Mode_e;

typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    int32_t position_q16;
    int32_t velocity_q16;
    int16_t torque_q8;
    int16_t temperature_c;
    uint16_t status_flags;
} MotorState_t;

typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    uint8_t motor_type;
    uint8_t control_mode;
    int32_t target_position_q16;
    int32_t target_velocity_q16;
    int16_t target_torque_q8;
} MotorCmd_t;

typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t timestamp_ms;
    uint8_t motor_count;
    MotorState_t motors[RPMSG_FRAME_MAX_MOTOR_CNT];
    uint16_t crc16;
} FrameTelemetry_t;

typedef struct __attribute__((packed)) {
    uint16_t seq;
    uint32_t timestamp_ms;
    uint8_t motor_count;
    MotorCmd_t motors[RPMSG_FRAME_MAX_MOTOR_CNT];
    uint16_t crc16;
} FrameCommand_t;
```

注意：

- `packed` 不能删
- 字段顺序 MCU/Linux 必须完全一致
- `crc16` 默认放在结构体最后
- `motor_type` 和 `control_mode` 在线上传输时用 `uint8_t`
- `enum` 只是语义定义，不直接依赖 enum 的底层大小

推荐安全写法：

- 协议结构体里固定使用 `uint8_t motor_type` 和 `uint8_t control_mode`
- 业务代码读取后，如果需要 enum 语义，直接强转即可
- 组包时继续写入 `uint8_t` 原始值，不要把协议字段改成 enum 类型


## 3. Q8 / Q16 是什么意思

这里用的是定点数，不是浮点数。

- `Q16`：真实值 = 原始整数值 / 65536
- `Q8`：真实值 = 原始整数值 / 256

例子：

- 位置 `1.5` -> `1.5 * 65536 = 98304`
- 速度 `-0.25` -> `-0.25 * 65536 = -16384`
- 力矩 `2.25` -> `2.25 * 256 = 576`

这意味着后续你要发送小数完全没有问题，不需要把协议改成 `float`。
只要 Linux 发包前先把真实值乘以对应缩放系数，转成整数即可。

## 4. Linux 侧怎么发小数

Linux 对端文件：

```text
/home/rmer/Project/Linux/vanxoak_rk3506_board_support/User/rpmsg_frame.c
```

现在命令行工具已经支持两种输入方式：

- 整数字面量：按历史兼容方式，直接当作原始 Q 值发送
- 小数字面量：自动按 Q 格式换算后发送

也就是说：

- `-p 65536` 仍然表示直接发送原始 `Q16=65536`
- `-p 1.5` 表示发送真实值 `1.5`，工具内部会转成 `98304`

### 4.1 Linux 侧当前镜像结构

```c
typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    uint8_t motor_type;
    uint8_t control_mode;
    int32_t target_position_q16;
    int32_t target_velocity_q16;
    int16_t target_torque_q8;
} MotorCmd_t;

typedef struct {
    uint8_t motor_id;
    uint8_t motor_type;
    uint8_t control_mode;
    int32_t target_position_q16;
    int32_t target_velocity_q16;
    int16_t target_torque_q8;
    uint32_t period_ticks;
} CommandFrameConfig;
```

### 4.2 Linux 侧组包代码

```c
frame.motors[0].motor_id = cfg->motor_id;
frame.motors[0].motor_type = cfg->motor_type;
frame.motors[0].control_mode = cfg->control_mode;
frame.motors[0].target_position_q16 = cfg->target_position_q16;
frame.motors[0].target_velocity_q16 = cfg->target_velocity_q16;
frame.motors[0].target_torque_q8 = cfg->target_torque_q8;
```

### 4.3 Linux 侧命令行示例

传原始 Q 值：

```bash
/root/rpmsg_frame -i 7 -T HT04 -m TORQUE -p 65536 -v -12345 -t 321
```

传真实小数值：

```bash
/root/rpmsg_frame -i 7 -T HT04 -m TORQUE -p 1.5 -v -0.18837 -t 1.25
```

程序启动后会打印两套信息，便于核对：

- 真实值
- 实际发出去的 raw Q 值

例如：

```text
[Linux CFG] motor_id=7 type=HT04(5) mode=TORQUE(1) pos=1.500000(raw=98304) vel=-0.188370(raw=-12345) torque=1.250000(raw=320) period=1
```

### 4.4 Linux 编译命令

```bash
'/home/rmer/usr/local/gcc-arm-10.3-2021.07-x86_64-arm-none-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gcc' \
  -O2 -g -Wall -Wextra \
  -o /home/rmer/Project/Linux/vanxoak_rk3506_board_support/User/rpmsg_frame \
  /home/rmer/Project/Linux/vanxoak_rk3506_board_support/User/rpmsg_frame.c
```

## 5. MCU 侧怎么用

### 5.1 初始化

```c
static RPMsgFrameInstance *g_rpmsgIns;

static uint8_t App_RPMsgInit(void)
{
    RPMsgFrame_Init_Config_s config;

    memset(&config, 0, sizeof(config));
    config.local_ept = APP_RPMSG_LOCAL_EPT;
    config.remote_ept = RPMSG_REMOTE_EPT_DYNAMIC;
    config.ept_name = APP_RPMSG_SERVICE_NAME;
    config.state_motor_count = APP_RPMSG_MOTOR_CNT;
    config.command_callback = App_RPMsgCallback;

    g_rpmsgIns = RPMsgFrameInit(&config);
    if (g_rpmsgIns == NULL) {
        return 0U;
    }

    return 1U;
}
```

### 5.2 回调里读取命令并返回状态

```c
static void App_RPMsgCallback(RPMsgFrameInstance *ins)
{
    FrameCommand_t command;

    if ((ins == NULL) || (RPMsgFrameGetCommandFrame(ins, &command, 1U) == 0U)) {
        return;
    }

    App_RPMsgBuildStateFrame(ins, &command);
    (void)RPMsgFrameTransmitStateFrame(ins, HAL_GetTick(), RL_BLOCK);
}
```

### 5.3 业务代码里如何安全使用 enum

推荐在 MCU 业务代码里这样写：

```c
const MotorCmd_t *motorCmd = &command->motors[motorIdx];
Motor_Type_e motorType = (Motor_Type_e)motorCmd->motor_type;
Motor_Control_Mode_e controlMode = (Motor_Control_Mode_e)motorCmd->control_mode;

switch (controlMode) {
case MOTOR_CONTROL_MODE_TORQUE:
    break;
case MOTOR_CONTROL_MODE_VELOCITY:
    break;
case MOTOR_CONTROL_MODE_POSITION:
    break;
default:
    break;
}

(void)motorType;
```

这样做的好处是：

- 线上协议大小固定，MCU/Linux 不会因为 enum 大小不同而错位
- 业务代码里仍然能保留 enum 的可读性
- 后续新增控制模式时，只需要扩 `Motor_Control_Mode_e` 和业务分支

### 5.4 App 层填写状态帧

当前 `main.c` 示例：

```c
static void App_RPMsgBuildStateFrame(RPMsgFrameInstance *ins, const FrameCommand_t *command)
{
    FrameTelemetry_t *stateFrame;
    uint8_t motorIdx;
    uint8_t motorCount;

    stateFrame = RPMsgFrameGetStateFrame(ins);
    if (stateFrame == NULL) {
        return;
    }

    RPMsgFrameResetStateFrame(ins);

    motorCount = command->motor_count;
    if (motorCount > RPMSG_FRAME_MAX_MOTOR_CNT) {
        motorCount = RPMSG_FRAME_MAX_MOTOR_CNT;
    }

    stateFrame->motor_count = motorCount;

    for (motorIdx = 0U; motorIdx < motorCount; motorIdx++) {
        const MotorCmd_t *motorCmd = &command->motors[motorIdx];
        MotorState_t *motorState = &stateFrame->motors[motorIdx];

        motorState->motor_id = motorCmd->motor_id;
        motorState->position_q16 = motorCmd->target_position_q16;
        motorState->velocity_q16 = motorCmd->target_velocity_q16;
        motorState->torque_q8 = motorCmd->target_torque_q8;
        motorState->temperature_c = APP_RPMSG_TEMP_C;
        motorState->status_flags = 0U;
    }
}
```

## 6. 后续如果要改帧定义，应该怎么做

改帧定义时，MCU 和 Linux 必须一起改。

最少要同步看这 4 个位置：

- `project/rk3506-mcu/modules/rpmsg_frame/rpmsg_frame.h`
- `project/rk3506-mcu/src/main.c`
- `/home/rmer/Project/Linux/vanxoak_rk3506_board_support/User/rpmsg_frame.c`
- `project/rk3506-mcu/bsp/bsp_rpmsg/bsp_rpmsg.h`

### 6.1 先改 MCU 侧协议头文件

如果你要新增一个命令字段，例如 `current_limit_q8`：

```c
typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    uint8_t motor_type;
    uint8_t control_mode;
    int32_t target_position_q16;
    int32_t target_velocity_q16;
    int16_t target_torque_q8;
    int16_t current_limit_q8;   /* 新增字段 */
} MotorCmd_t;
```

如果你是要新增控制模式，例如 `CURRENT`：

```c
typedef enum {
    MOTOR_CONTROL_MODE_NONE = 0,
    MOTOR_CONTROL_MODE_TORQUE,
    MOTOR_CONTROL_MODE_VELOCITY,
    MOTOR_CONTROL_MODE_POSITION,
    MOTOR_CONTROL_MODE_CURRENT,   /* 新增模式 */
} Motor_Control_Mode_e;
```

这里要注意：

- 只扩 `Motor_Control_Mode_e` 的枚举值
- 不要把 `MotorCmd_t` 里的 `uint8_t control_mode` 改成 enum 类型
- 线上协议字段仍然保持 `uint8_t`
- 业务代码里如果要按枚举判断，直接强转成 `Motor_Control_Mode_e`

### 6.2 同步改 Linux 侧镜像结构

Linux 侧必须加在同样的位置：

```c
typedef struct __attribute__((packed)) {
    uint8_t motor_id;
    uint8_t motor_type;
    uint8_t control_mode;
    int32_t target_position_q16;
    int32_t target_velocity_q16;
    int16_t target_torque_q8;
    int16_t current_limit_q8;   /* 必须与 MCU 位置一致 */
} MotorCmd_t;
```

### 6.3 同步改 Linux 配置结构和组包逻辑

只改结构体还不够，还要把配置项真正写进帧：

```c
typedef struct {
    uint8_t motor_id;
    uint8_t motor_type;
    uint8_t control_mode;
    int32_t target_position_q16;
    int32_t target_velocity_q16;
    int16_t target_torque_q8;
    int16_t current_limit_q8;   /* 新增 */
    uint32_t period_ticks;
} CommandFrameConfig;
```

`send_command_frame()` 也要同步：

```c
frame.motors[0].motor_id = cfg->motor_id;
frame.motors[0].motor_type = cfg->motor_type;
frame.motors[0].control_mode = cfg->control_mode;
frame.motors[0].target_position_q16 = cfg->target_position_q16;
frame.motors[0].target_velocity_q16 = cfg->target_velocity_q16;
frame.motors[0].target_torque_q8 = cfg->target_torque_q8;
frame.motors[0].current_limit_q8 = cfg->current_limit_q8;   /* 新增 */
```

如果这个新字段需要从命令行传入，还要继续同步修改：

- `print_usage()`
- `parse_command_frame_config()`

如果你新增了控制模式，还要同步修改 Linux 侧这几处：

- `parse_control_mode()`
- `control_mode_to_string()`

### 6.4 同步改 MCU 应用层映射

`rpmsg_frame` 模块本身不决定业务含义，所以 App 层也要改。

例如你新增了 `current_limit_q8`，在 `main.c` 里要决定它怎么被使用：

```c
const MotorCmd_t *motorCmd = &command->motors[motorIdx];

motorState->motor_id = motorCmd->motor_id;
motorState->position_q16 = motorCmd->target_position_q16;
motorState->velocity_q16 = motorCmd->target_velocity_q16;
motorState->torque_q8 = motorCmd->target_torque_q8;
```

如果状态帧也要新增对应字段，就继续同步改 `MotorState_t` 和 `App_RPMsgBuildStateFrame()`。

### 6.5 如果改了服务名或 endpoint，也要同步两边

MCU 侧：

```c
#define APP_RPMSG_LOCAL_EPT     0x4003U
#define APP_RPMSG_SERVICE_NAME  "rpmsg-mcu0-test"
```

Linux 侧：

```c
#define RPMSG_SERVICE_NAME      "rpmsg-mcu0-test"
#define MCU_EPT_ADDR            0x4003U
#define LOCAL_EPT_ADDR          1024U
```

对应关系：

- `APP_RPMSG_SERVICE_NAME` <-> `RPMSG_SERVICE_NAME`
- `APP_RPMSG_LOCAL_EPT` <-> `MCU_EPT_ADDR`

### 6.6 最后一定要检查的点

- 两边结构体字段类型一致
- 两边字段顺序一致
- `packed` 保留
- `RPMSG_FRAME_MAX_MOTOR_CNT` 一致
- `crc16` 仍然在结构体最后
- Linux `sizeof(FrameCommand_t)` 和 MCU `sizeof(FrameCommand_t)` 一致
- Linux `sizeof(FrameTelemetry_t)` 和 MCU `sizeof(FrameTelemetry_t)` 一致

## 7. 常见错误

- 只改了 MCU，没改 Linux
- 只改了结构体，忘了改 `send_command_frame()`
- 新增了 enum，但 Linux 解析函数没同步加
- 误把在线传输字段改成直接传 enum 类型
- 去掉了 `packed`
- 把小数直接强转整数，忘了先乘 `Q8/Q16` 缩放系数

## 8. 一句话记忆

如果你后续要改 RPMsg 电机协议，最核心的原则只有一条：

MCU 的 `rpmsg_frame.h` 改什么，Linux 的 `rpmsg_frame.c` 就必须按同样顺序、同样类型、同样 `packed` 一起改。
