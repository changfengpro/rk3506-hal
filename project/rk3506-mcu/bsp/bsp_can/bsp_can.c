#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>

/* 与 hal_bsp.h 中定义保持一致 */
extern const struct HAL_CANFD_DEV g_can0Dev;
extern const struct HAL_CANFD_DEV g_can1Dev;

#define CAN_DEVICE_COUNT        2U
#define CAN_OUT_INDEX_INVALID   0xFFFFFFFFU

/* RK3506 有效中断位 [19:0] */
#define CAN_INT_VALID_MASK      0x000FFFFFU

#define CAN_TX_DONE_MASK        CAN_INT_TX_FINISH_INT_MASK
#define CAN_TX_FAIL_MASK        (CAN_INT_BUS_OFF_INT_MASK | \
                                 CAN_INT_ERROR_INT_MASK | \
                                 CAN_INT_TX_ARBIT_FAIL_INT_MASK | \
                                 CAN_INT_PASSIVE_ERROR_INT_MASK | \
                                 CAN_INT_OVERLOAD_INT_MASK | \
                                 CAN_INT_ERROR_WARNING_INT_MASK | \
                                 CAN_INT_AUTO_RETX_FAIL_INT_MASK)

/*
 * RK3506 接收主路径为 MFI/RXSTR_*，并非只看 RX_FINISH。
 * 纯硬件中断模式下，发送完成/失败也必须放行，由 ISR 负责更新状态。
 */
#define CAN_IRQ_UNMASK_BITS     (CAN_INT_MFI_INT_MASK | \
                                 CAN_INT_RX_FINISH_INT_MASK | \
                                 CAN_INT_RXSTR_FULL_INT_MASK | \
                                 CAN_INT_RXSTR_OVERFLOW_INT_MASK | \
                                 CAN_TX_DONE_MASK | \
                                 CAN_TX_FAIL_MASK)

#define CAN_RX_EVENT_MASK       (CAN_INT_MFI_INT_MASK | \
                                 CAN_INT_RX_FINISH_INT_MASK | \
                                 CAN_INT_RXSTR_FULL_INT_MASK | \
                                 CAN_INT_ISM_WTM_INT_MASK | \
                                 CAN_INT_RXSTR_TIMEOUT_INT_MASK)

#define CAN_SCLK_HZ             200000000U
#define CAN_SCLK_FALLBACK_HZ    24000000U
#define CAN_SCLK_MAX_VALID_HZ   600000000U
#define CAN_SCLK_24M_MIN_HZ     23000000U
#define CAN_SCLK_24M_MAX_HZ     25000000U
#define CAN_RX_DRAIN_LIMIT      16U
#define CAN_CLASSIC_DLC_MAX     8U

#ifndef CAN1_TX_BANK
#define CAN1_TX_BANK GPIO_BANK0
#endif
#ifndef CAN1_TX_PIN
#define CAN1_TX_PIN GPIO_PIN_B5 /* RM_IO13 */
#endif
#ifndef CAN1_RX_BANK
#define CAN1_RX_BANK GPIO_BANK0
#endif
#ifndef CAN1_RX_PIN
#define CAN1_RX_PIN GPIO_PIN_C2 /* RM_IO18 */
#endif

static const struct HAL_CANFD_DEV *const s_canDevs[CAN_DEVICE_COUNT] = {
    &g_can0Dev,
    &g_can1Dev,
};

static uint32_t s_canOutIndex[CAN_DEVICE_COUNT] = {
    CAN_OUT_INDEX_INVALID,
    CAN_OUT_INDEX_INVALID,
};

/* can instance ptrs storage, used for recv callback */
static CANInstance *can_instances[CAN_MX_REGISTER_CNT] = { NULL };
static uint8_t instance_idx = 0U;
static volatile uint32_t s_canTxDone[CAN_DEVICE_COUNT] = { 0U };
static volatile uint32_t s_canTxFail[CAN_DEVICE_COUNT] = { 0U };

/*
 * 这些计数用于保留当前已验证通过的中断链路实现形态。
 * 虽然默认不对外打印，但先保留在本地，避免修改已验证收包路径。
 */
static volatile uint32_t g_isr_hit_count = 0U;
static volatile uint32_t g_can_adapter_hit_count = 0U;
static volatile uint32_t g_can_common_hit_count = 0U;
static volatile uint32_t g_can_last_isr = 0U;
static volatile uint32_t g_can_rx_dispatch_count = 0U;

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief INTMUX 适配层，按中断源转发到对应 CAN 控制器。
 * @param irq 触发的 INTMUX 源中断号。
 * @param args 私有参数，当前未使用。
 * @return HAL_OK。
 */
static HAL_Status CAN_INTMUX_Adapter(uint32_t irq, void *args);

/**
 * @brief 配置 RK3506 上 CAN0/CAN1 的 IOMUX 与 RMIO 复用。
 */
static void CAN_ConfigPins(void);

/**
 * @brief 初始化单个 CAN 控制器。
 * @param dev CAN 控制器设备描述。
 * @param cfg HAL CANFD 初始化参数。
 */
static void CAN_InitController(const struct HAL_CANFD_DEV *dev, struct CANFD_CONFIG *cfg);

/**
 * @brief 将接收到的报文分发给已注册实例。
 * @param pReg 收包控制器基址。
 * @param rx_msg 接收到的报文。
 */
static void CAN_DispatchRxMessage(struct CAN_REG *pReg, const struct CANFD_MSG *rx_msg);

/**
 * @brief 尽快取空当前接收 FIFO 中的报文。
 * @param pReg CAN 控制器基址。
 */
static void CAN_DrainRxFifo(struct CAN_REG *pReg);

/**
 * @brief 给指定的 INTMUX 输出中断绑定实际 ISR。
 * @param outIrq INTMUX 输出中断号。
 */
static void CAN_BindIntmuxOutIRQHandler(IRQn_Type outIrq);

/**
 * @brief 配置当前 CAN 使用到的 INTMUX 输出中断。
 * @param outIrqs 输出中断数组。
 * @param outCount 数组中有效输出中断数量。
 */
static void CAN_ConfigIntmuxOutputIRQs(const IRQn_Type *outIrqs, uint32_t outCount);

/**
 * @brief 打开单路 CAN 的 INTMUX 源中断。
 * @param dev CAN 控制器设备描述。
 */
static void CAN_EnableIntmuxSourceIRQ(const struct HAL_CANFD_DEV *dev);

/**
 * @brief 应用运行期中断屏蔽策略。
 * @param dev CAN 控制器设备描述。
 */
static void CAN_ApplyRuntimeInterruptMask(const struct HAL_CANFD_DEV *dev);

/**
 * @brief 根据模式位补充 HAL 未覆盖的 CAN 模式设置。
 * @param pReg CAN 寄存器基址。
 * @param canfdMode CANFD 模式位。
 */
static void CAN_ApplyModeFlags(struct CAN_REG *pReg, uint32_t canfdMode);

/**
 * @brief 在 24MHz SCLK 场景下覆写 1Mbps 时序参数。
 * @param pReg CAN 寄存器基址。
 * @param sclkHz 当前 CAN SCLK 频率。
 */
static void CAN_ApplyNominalTiming1M(struct CAN_REG *pReg, uint32_t sclkHz);

/**
 * @brief 按 32bit 字边界整理 CAN 载荷字节序。
 * @param dst 目标缓冲区。
 * @param src 源缓冲区。
 * @param len 有效字节数。
 */
static void CAN_SwapBytesPerWord(uint8_t *dst, const uint8_t *src, uint8_t len);

/**
 * @brief 根据 CAN 寄存器基址查找控制器索引。
 * @param pReg CAN 寄存器基址。
 * @return 找到时返回控制器索引，失败时返回 CAN_DEVICE_COUNT。
 */
static uint32_t CAN_GetDeviceIndexByReg(struct CAN_REG *pReg);

/**
 * @brief 计算 INTMUX 输入中断对应的源号。
 * @param irq 原始中断号。
 * @return INTMUX 内部源号。
 */
static uint32_t CAN_GetIntmuxSource(uint32_t irq);

/**
 * @brief 根据 CAN 源中断号计算 INTMUX 输出中断号。
 * @param srcIrq CAN 源中断号。
 * @return 对应的 INTMUX 输出中断号。
 */
static IRQn_Type CAN_GetIntmuxOutIRQ(IRQn_Type srcIrq);

/**
 * @brief 计算 CAN 源中断对应的 INTMUX 输出索引。
 * @param srcIrq CAN 源中断号。
 * @return INTMUX 输出索引。
 */
static uint32_t CAN_GetIntmuxOutIndex(IRQn_Type srcIrq);

/**
 * @brief INTMUX 输出 0 中断入口。
 */
void INTMUX0_IRQHandler(void);

/**
 * @brief INTMUX 输出 1 中断入口。
 */
void INTMUX1_IRQHandler(void);

/**
 * @brief INTMUX 输出 2 中断入口。
 */
void INTMUX2_IRQHandler(void);

/**
 * @brief INTMUX 输出 3 中断入口。
 */
void INTMUX3_IRQHandler(void);

/**
 * @brief 计算 INTMUX 输入中断对应的源号。
 * @param irq 原始中断号。
 * @return INTMUX 内部源号。
 */
static uint32_t CAN_GetIntmuxSource(uint32_t irq)
{
    uint32_t intmuxStart = (uint32_t)NUM_INTERRUPTS + (uint32_t)INTMUX_IRQ_START_NUM;

    if (irq >= intmuxStart) {
        return irq - intmuxStart;
    }

    return irq;
}

/**
 * @brief 根据 CAN 源中断号计算 INTMUX 输出中断号。
 * @param srcIrq CAN 源中断号。
 * @return 对应的 INTMUX 输出中断号。
 */
static IRQn_Type CAN_GetIntmuxOutIRQ(IRQn_Type srcIrq)
{
    uint32_t intmuxSrcNum;

    intmuxSrcNum = (uint32_t)srcIrq - (uint32_t)NUM_INTERRUPTS - (uint32_t)INTMUX_IRQ_START_NUM;

    return (IRQn_Type)((uint32_t)INTMUX_OUT_IRQ_START_NUM +
                       (intmuxSrcNum / (uint32_t)INTMUX_NUM_INT_PER_OUT));
}

/**
 * @brief 计算 CAN 源中断对应的 INTMUX 输出索引。
 * @param srcIrq CAN 源中断号。
 * @return INTMUX 输出索引。
 */
static uint32_t CAN_GetIntmuxOutIndex(IRQn_Type srcIrq)
{
    IRQn_Type outIrq = CAN_GetIntmuxOutIRQ(srcIrq);

    return (uint32_t)outIrq - (uint32_t)INTMUX_OUT_IRQ_START_NUM;
}

/**
 * @brief 根据 CAN 寄存器基址查找控制器索引。
 * @param pReg CAN 寄存器基址。
 * @return 找到时返回控制器索引，失败时返回 CAN_DEVICE_COUNT。
 */
static uint32_t CAN_GetDeviceIndexByReg(struct CAN_REG *pReg)
{
    uint32_t i;

    for (i = 0U; i < CAN_DEVICE_COUNT; i++) {
        if (s_canDevs[i]->pReg == pReg) {
            return i;
        }
    }

    return CAN_DEVICE_COUNT;
}

/**
 * @brief 给指定的 INTMUX 输出中断绑定实际 ISR。
 * @param outIrq INTMUX 输出中断号。
 */
static void CAN_BindIntmuxOutIRQHandler(IRQn_Type outIrq)
{
    if (outIrq == INTMUX_OUT0_IRQn) {
        HAL_NVIC_SetIRQHandler(INTMUX_OUT0_IRQn, INTMUX0_IRQHandler);
    } else if (outIrq == INTMUX_OUT1_IRQn) {
        HAL_NVIC_SetIRQHandler(INTMUX_OUT1_IRQn, INTMUX1_IRQHandler);
    } else if (outIrq == INTMUX_OUT2_IRQn) {
        HAL_NVIC_SetIRQHandler(INTMUX_OUT2_IRQn, INTMUX2_IRQHandler);
    } else if (outIrq == INTMUX_OUT3_IRQn) {
        HAL_NVIC_SetIRQHandler(INTMUX_OUT3_IRQn, INTMUX3_IRQHandler);
    }
}

/**
 * @brief 配置当前 CAN 使用到的 INTMUX 输出中断。
 * @param outIrqs 输出中断数组。
 * @param outCount 数组中有效输出中断数量。
 */
static void CAN_ConfigIntmuxOutputIRQs(const IRQn_Type *outIrqs, uint32_t outCount)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)INTMUX_NUM_OUT_PER_CON; i++) {
        IRQn_Type outIrq = (IRQn_Type)((uint32_t)INTMUX_OUT_IRQ_START_NUM + i);

        HAL_NVIC_ClearPendingIRQ(outIrq);
        HAL_NVIC_DisableIRQ(outIrq);
    }

    for (i = 0U; i < outCount; i++) {
        HAL_NVIC_ClearPendingIRQ(outIrqs[i]);
        CAN_BindIntmuxOutIRQHandler(outIrqs[i]);
        HAL_NVIC_SetPriority(outIrqs[i], 1U, 0U);
        HAL_NVIC_EnableIRQ(outIrqs[i]);
    }
}

/**
 * @brief 打开单路 CAN 的 INTMUX 源中断。
 * @param dev CAN 控制器设备描述。
 */
static void CAN_EnableIntmuxSourceIRQ(const struct HAL_CANFD_DEV *dev)
{
    HAL_INTMUX_ClearPendingIRQ(dev->irqNum);
    HAL_INTMUX_EnableIRQ(dev->irqNum);
}

/**
 * @brief 应用运行期中断屏蔽策略。
 * @param dev CAN 控制器设备描述。
 */
static void CAN_ApplyRuntimeInterruptMask(const struct HAL_CANFD_DEV *dev)
{
    WRITE_REG(dev->pReg->INT_MASK, CAN_INT_VALID_MASK & ~CAN_IRQ_UNMASK_BITS);
}

/**
 * @brief 根据模式位补充 HAL 未覆盖的 CAN 模式设置。
 * @param pReg CAN 寄存器基址。
 * @param canfdMode CANFD 模式位。
 */
static void CAN_ApplyModeFlags(struct CAN_REG *pReg, uint32_t canfdMode)
{
    if ((canfdMode & CANFD_MODE_LOOPBACK) != 0U) {
        SET_BIT(pReg->MODE, CAN_MODE_RXSTX_MODE_MASK);
    } else {
        CLEAR_BIT(pReg->MODE, CAN_MODE_RXSTX_MODE_MASK);
    }
}

/**
 * @brief 在 24MHz SCLK 场景下覆写 1Mbps 时序参数。
 * @param pReg CAN 寄存器基址。
 * @param sclkHz 当前 CAN SCLK 频率。
 */
static void CAN_ApplyNominalTiming1M(struct CAN_REG *pReg, uint32_t sclkHz)
{
    /*
     * HAL 内置 BPS 参数按 200MHz 设计。
     * 当 sclk 落在 24MHz 档时，手工覆写 1Mbps 时序：
     * baud = sclk / (2 * (brq + 1) * (tseg1 + tseg2 + 3))
     * 取 brq=0, tseg1=8, tseg2=1 => 24MHz / 24 = 1MHz
     *
     * ACK delimiter 附近的接收错误对 ACK 槽同步较敏感，
     * 这里同时关闭 ACK_SLOT 的硬同步，避免在 ACK 段重新对齐。
     */
    if ((sclkHz >= CAN_SCLK_24M_MIN_HZ) && (sclkHz <= CAN_SCLK_24M_MAX_HZ)) {
        uint32_t nominal = (0U << CAN_FD_NOMINAL_BITTIMING_SJW_SHIFT) |
                           (0U << CAN_FD_NOMINAL_BITTIMING_BRQ_SHIFT) |
                           (8U << CAN_FD_NOMINAL_BITTIMING_TSEG1_SHIFT) |
                           (1U << CAN_FD_NOMINAL_BITTIMING_TSEG2_SHIFT);
        uint32_t data = CAN_FD_DATA_BITTIMING_ACKSLOT_SYNC_DIS_MASK |
                        (0U << CAN_FD_DATA_BITTIMING_SJW_SHIFT) |
                        (0U << CAN_FD_DATA_BITTIMING_BRQ_SHIFT) |
                        (8U << CAN_FD_DATA_BITTIMING_TSEG1_SHIFT) |
                        (1U << CAN_FD_DATA_BITTIMING_TSEG2_SHIFT);

        WRITE_REG(pReg->FD_NOMINAL_BITTIMING, nominal);
        WRITE_REG(pReg->FD_DATA_BITTIMING, data);
    }
}

/**
 * @brief 按 32bit 字边界整理 CAN 载荷字节序。
 * @param dst 目标缓冲区。
 * @param src 源缓冲区。
 * @param len 有效字节数。
 */
static void CAN_SwapBytesPerWord(uint8_t *dst, const uint8_t *src, uint8_t len)
{
    uint8_t base;

    for (base = 0U; base < len; base = (uint8_t)(base + 4U)) {
        uint8_t i;
        uint8_t remain = (uint8_t)(len - base);
        uint8_t chunk = (remain < 4U) ? remain : 4U;

        for (i = 0U; i < chunk; i++) {
            dst[base + i] = src[base + (chunk - 1U - i)];
        }
    }
}

/**
 * @brief 配置 RK3506 上 CAN0/CAN1 的 IOMUX 与 RMIO 复用。
 */
static void CAN_ConfigPins(void)
{
    /*
     * CAN0: TX->RM_IO19(GPIO0_C3), RX->RM_IO20(GPIO0_C4)
     * CAN1: TX->RM_IO13(GPIO0_B5), RX->RM_IO18(GPIO0_C2)
     */
    HAL_PINCTRL_SetIOMUX(GPIO_BANK0, GPIO_PIN_C3, PIN_CONFIG_MUX_FUNC7);
    HAL_PINCTRL_SetIOMUX(GPIO_BANK0, GPIO_PIN_C4, PIN_CONFIG_MUX_FUNC7);
    HAL_PINCTRL_SetIOMUX(CAN1_TX_BANK, CAN1_TX_PIN, PIN_CONFIG_MUX_FUNC7);
    HAL_PINCTRL_SetIOMUX(CAN1_RX_BANK, CAN1_RX_PIN, PIN_CONFIG_MUX_FUNC7);

    HAL_PINCTRL_SetRMIO(GPIO_BANK0, GPIO_PIN_C3, RMIO_CAN0_TX);
    HAL_PINCTRL_SetRMIO(GPIO_BANK0, GPIO_PIN_C4, RMIO_CAN0_RX);
    HAL_PINCTRL_SetRMIO(CAN1_TX_BANK, CAN1_TX_PIN, RMIO_CAN1_TX);
    HAL_PINCTRL_SetRMIO(CAN1_RX_BANK, CAN1_RX_PIN, RMIO_CAN1_RX);
}

/**
 * @brief 初始化单个 CAN 控制器。
 * @param dev CAN 控制器设备描述。
 * @param cfg HAL CANFD 初始化参数。
 */
static void CAN_InitController(const struct HAL_CANFD_DEV *dev, struct CANFD_CONFIG *cfg)
{
    uint32_t sclkHz = CAN_SCLK_FALLBACK_HZ;

#if defined(HAL_CRU_MODULE_ENABLED) && !defined(IS_FPGA)
    HAL_CRU_ClkEnable(dev->pclkGateID);
    HAL_CRU_ClkEnable(dev->sclkGateID);
    HAL_CRU_ClkSetFreq(dev->sclkID, CAN_SCLK_HZ);
    sclkHz = HAL_CRU_ClkGetFreq(dev->sclkID);
    if ((sclkHz == 0U) || (sclkHz > CAN_SCLK_MAX_VALID_HZ)) {
        HAL_CRU_ClkSetFreq(dev->sclkID, CAN_SCLK_FALLBACK_HZ);
    }
#endif

    HAL_CANFD_Init(dev->pReg, cfg);
    CAN_ApplyModeFlags(dev->pReg, cfg->canfdMode);
    CAN_ApplyNominalTiming1M(dev->pReg, sclkHz);
    /*
     * 关闭默认 RX 波形滤波，避免单边接收链路被 1Mbps 窄脉冲整形影响。
     * 当前现象是“MCU 发正常，外部发给 MCU 在 ACK delimiter 附近出错”，
     * 这是典型的接收侧敏感问题。
     */
    WRITE_REG(dev->pReg->WAVE_FILTER_CFG, 0U);
    WRITE_REG(dev->pReg->RXINT_CTRL, 1U);
    WRITE_REG(dev->pReg->ATF_CTL, CAN_ATF_CTL_ATF0_EN_MASK);
    HAL_CANFD_Start(dev->pReg);

    /* 初始化期屏蔽全部中断，清空历史状态后再放行 */
    WRITE_REG(dev->pReg->INT_MASK, CAN_INT_VALID_MASK);
    WRITE_REG(dev->pReg->INT, CAN_INT_VALID_MASK);

    /*
     * 这里保持为已验证可接收的中断链路写法：
     * 由 INTMUX 通过 irq 参数反查源设备，不依赖私有 args。
     */
    HAL_INTMUX_SetIRQHandler(dev->irqNum, CAN_INTMUX_Adapter, NULL);
}

/**
 * @brief 将接收到的报文分发给已注册实例。
 * @param pReg 收包控制器基址。
 * @param rx_msg 接收到的报文。
 */
static void CAN_DispatchRxMessage(struct CAN_REG *pReg, const struct CANFD_MSG *rx_msg)
{
    size_t i;

    for (i = 0U; i < instance_idx; i++) {
        uint8_t copyLen;
        uint8_t payload[64];

        if (can_instances[i] == NULL) {
            continue;
        }
        if (can_instances[i]->can_handle != pReg) {
            continue;
        }
        if (can_instances[i]->rx_id != rx_msg->stdId) {
            continue;
        }

        copyLen = rx_msg->dlc;
        if (copyLen > sizeof(can_instances[i]->rx_buff)) {
            copyLen = sizeof(can_instances[i]->rx_buff);
        }

        /*
         * HAL_CANFD 的 32bit 数据寄存器按大端字节拼包，
         * 在总线上观察会表现为每 4 字节逆序。这里统一转换回自然顺序。
         */
        CAN_SwapBytesPerWord(payload, rx_msg->data, copyLen);
        can_instances[i]->rx_len = copyLen;
        memcpy(can_instances[i]->rx_buff, payload, copyLen);
        g_can_rx_dispatch_count++;

        if (can_instances[i]->can_module_callback != NULL) {
            can_instances[i]->can_module_callback(can_instances[i]);
        }
    }
}

/**
 * @brief 尽快取空当前接收 FIFO 中的报文。
 * @param pReg CAN 控制器基址。
 */
static void CAN_DrainRxFifo(struct CAN_REG *pReg)
{
    uint32_t guard = 0U;

    while (guard++ < CAN_RX_DRAIN_LIMIT) {
        uint32_t frameCnt;
        uint32_t strState = READ_REG(pReg->STR_STATE);
        struct CANFD_MSG rx_msg = { 0 };

        frameCnt = (strState & CAN_STR_STATE_INTM_FRAME_CNT_MASK) >>
                   CAN_STR_STATE_INTM_FRAME_CNT_SHIFT;
        if (frameCnt == 0U) {
            break;
        }

        if (HAL_CANFD_Receive(pReg, &rx_msg) != HAL_OK) {
            break;
        }

        CAN_DispatchRxMessage(pReg, &rx_msg);
    }
}

/**
 * @brief 处理单个 CAN 控制器的收发中断。
 * @param pReg CAN 控制器基址。
 */
void CAN_Common_IRQHandler(struct CAN_REG *pReg)
{
    uint32_t devIdx;
    uint32_t txFailIsr = 0U;
    uint32_t isr = HAL_CANFD_GetInterrupt(pReg);

    g_can_common_hit_count++;
    g_can_last_isr = isr;
    devIdx = CAN_GetDeviceIndexByReg(pReg);

    if (devIdx < CAN_DEVICE_COUNT) {
        if ((isr & CAN_TX_DONE_MASK) != 0U) {
            s_canTxDone[devIdx] = 1U;
        }
        txFailIsr |= isr & (CAN_INT_BUS_OFF_INT_MASK |
                            CAN_INT_TX_ARBIT_FAIL_INT_MASK |
                            CAN_INT_AUTO_RETX_FAIL_INT_MASK);
        if ((isr & (CAN_INT_ERROR_INT_MASK |
                    CAN_INT_PASSIVE_ERROR_INT_MASK |
                    CAN_INT_OVERLOAD_INT_MASK |
                    CAN_INT_ERROR_WARNING_INT_MASK)) != 0U) {
            uint32_t errCode = READ_REG(pReg->ERROR_CODE);

            if ((errCode & CAN_ERROR_CODE_ERROR_DIRECTION_MASK) == 0U) {
                txFailIsr |= isr & (CAN_INT_ERROR_INT_MASK |
                                    CAN_INT_PASSIVE_ERROR_INT_MASK |
                                    CAN_INT_OVERLOAD_INT_MASK |
                                    CAN_INT_ERROR_WARNING_INT_MASK);
            }
        }
        if (txFailIsr != 0U) {
            s_canTxFail[devIdx] = txFailIsr;
        }
    }

    if ((isr & CAN_RX_EVENT_MASK) != 0U) {
        CAN_DrainRxFifo(pReg);
    }
}

/**
 * @brief INTMUX 适配层，按中断源转发到对应 CAN 控制器。
 * @param irq 触发的 INTMUX 源中断号。
 * @param args 私有参数，当前未使用。
 * @return HAL_OK。
 */
static HAL_Status CAN_INTMUX_Adapter(uint32_t irq, void *args)
{
    uint32_t i;
    uint32_t irqSrc;

    (void)args;
    g_can_adapter_hit_count++;

    irqSrc = CAN_GetIntmuxSource(irq);
    for (i = 0U; i < CAN_DEVICE_COUNT; i++) {
        uint32_t devSrc = CAN_GetIntmuxSource((uint32_t)s_canDevs[i]->irqNum);

        if (irqSrc == devSrc) {
            CAN_Common_IRQHandler(s_canDevs[i]->pReg);
            break;
        }
    }

    return HAL_OK;
}

/* 通过 HAL_NVIC_SetIRQHandler 动态绑定 */
/**
 * @brief INTMUX 输出 0 中断入口。
 */
void INTMUX0_IRQHandler(void)
{
    g_isr_hit_count++;
    HAL_INTMUX_DirectDispatch(0U);
}

/**
 * @brief INTMUX 输出 1 中断入口。
 */
void INTMUX1_IRQHandler(void)
{
    g_isr_hit_count++;
    HAL_INTMUX_DirectDispatch(1U);
}

/**
 * @brief INTMUX 输出 2 中断入口。
 */
void INTMUX2_IRQHandler(void)
{
    g_isr_hit_count++;
    HAL_INTMUX_DirectDispatch(2U);
}

/**
 * @brief INTMUX 输出 3 中断入口。
 */
void INTMUX3_IRQHandler(void)
{
    g_isr_hit_count++;
    HAL_INTMUX_DirectDispatch(3U);
}

/* ----------------------- extern callable function -----------------------*/

/**
 * @brief 初始化 CAN 控制器、引脚和中断路由。
 */
void CAN_Service_Init(void)
{
    uint32_t i;
    uint32_t j;
    uint32_t outCount = 0U;
    IRQn_Type outIrqs[CAN_DEVICE_COUNT];
    struct CANFD_CONFIG can_cfg = { 0 };

    can_cfg.canfdMode = 0U; /* 外总线模式（非回环） */
    can_cfg.bps = CANFD_BPS_1MBAUD;

    CAN_ConfigPins();

    for (i = 0U; i < CAN_DEVICE_COUNT; i++) {
        IRQn_Type outIrq;

        CAN_InitController(s_canDevs[i], &can_cfg);

        s_canOutIndex[i] = CAN_GetIntmuxOutIndex(s_canDevs[i]->irqNum);
        outIrq = CAN_GetIntmuxOutIRQ(s_canDevs[i]->irqNum);

        for (j = 0U; j < outCount; j++) {
            if (outIrqs[j] == outIrq) {
                break;
            }
        }
        if (j == outCount) {
            outIrqs[outCount++] = outIrq;
        }
    }

    CAN_ConfigIntmuxOutputIRQs(outIrqs, outCount);

    for (i = 0U; i < CAN_DEVICE_COUNT; i++) {
        CAN_EnableIntmuxSourceIRQ(s_canDevs[i]);
    }

    for (i = 0U; i < CAN_DEVICE_COUNT; i++) {
        CAN_ApplyRuntimeInterruptMask(s_canDevs[i]);
    }
}

/**
 * @brief 注册一个 CAN 业务实例。
 * @param config 注册参数。
 * @return 注册成功返回实例指针，失败返回 NULL。
 */
CANInstance *CAN_Register(CAN_Init_Config_s *config)
{
    CANInstance *ins;

    if ((config == NULL) || (instance_idx >= CAN_MX_REGISTER_CNT)) {
        return NULL;
    }

    ins = (CANInstance *)malloc(sizeof(CANInstance));
    if (ins == NULL) {
        return NULL;
    }

    memset(ins, 0, sizeof(CANInstance));
    ins->can_handle = config->can_handle;
    ins->tx_id = config->tx_id;
    ins->rx_id = config->rx_id;
    ins->tx_len = CAN_CLASSIC_DLC_MAX;
    ins->can_module_callback = config->can_module_callback;
    ins->id = config->id;

    can_instances[instance_idx++] = ins;

    return ins;
}

/**
 * @brief 修改 CAN 发送帧长度。
 * @param instance 目标 CAN 实例。
 * @param length 发送长度，经典 CAN 模式下支持 1~8。
 */
void CAN_SetDLC(CANInstance *instance, uint8_t length)
{
    if (instance == NULL) {
        return;
    }
    if ((length == 0U) || (length > CAN_CLASSIC_DLC_MAX)) {
        return;
    }

    instance->tx_len = length;
}

/**
 * @brief 发送一帧标准 CAN 数据帧。
 * @param instance 已注册的 CAN 实例。
 * @param timeout_ms 发送等待超时时间，单位毫秒。
 * @return 1 表示发送成功，0 表示发送失败或超时。
 */
uint8_t CAN_Transmit(CANInstance *instance, uint32_t timeout_ms)
{
    uint32_t devIdx;
    uint32_t start;
    struct CANFD_MSG tx_msg = { 0 };

    if ((instance == NULL) || (instance->tx_len > sizeof(tx_msg.data))) {
        return 0U;
    }

    devIdx = CAN_GetDeviceIndexByReg(instance->can_handle);
    if (devIdx >= CAN_DEVICE_COUNT) {
        return 0U;
    }

    tx_msg.stdId = instance->tx_id;
    tx_msg.ide = CANFD_ID_STANDARD;
    tx_msg.rtr = CANFD_RTR_DATA;
    tx_msg.fdf = CANFD_FORMAT;
    tx_msg.dlc = instance->tx_len;

    /*
     * 与接收路径对称：转换后线上字节顺序与上层缓冲一致。
     */
    CAN_SwapBytesPerWord(tx_msg.data, instance->tx_buff, instance->tx_len);

    /*
     * 清理上一次发送遗留标志，避免“旧 TX_FINISH”误判本次成功。
     */
    s_canTxDone[devIdx] = 0U;
    s_canTxFail[devIdx] = 0U;
    WRITE_REG(instance->can_handle->INT, CAN_TX_DONE_MASK | CAN_TX_FAIL_MASK);

    start = HAL_GetTick();
    while (HAL_CANFD_Transmit(instance->can_handle, &tx_msg) != HAL_OK) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return 0U;
        }
    }

    start = HAL_GetTick();
    while (1) {
        if (s_canTxDone[devIdx] != 0U) {
            s_canTxDone[devIdx] = 0U;
            return 1U;
        }
        if (s_canTxFail[devIdx] != 0U) {
            s_canTxFail[devIdx] = 0U;
            return 0U;
        }
        if ((HAL_GetTick() - start) > timeout_ms) {
            return 0U;
        }
    }
}

/* ----------------------- compatible old-style interfaces -----------------------*/

/**
 * @brief 兼容旧命名风格的注册接口。
 * @param config 注册参数。
 * @return 注册成功返回实例指针，失败返回 NULL。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    return CAN_Register(config);
}

/**
 * @brief 兼容旧命名风格的发送长度设置接口。
 * @param instance 目标 CAN 实例。
 * @param length 发送长度，经典 CAN 模式下支持 1~8。
 */
void CANSetDLC(CANInstance *instance, uint8_t length)
{
    CAN_SetDLC(instance, length);
}

/**
 * @brief 兼容旧命名风格的发送接口。
 * @param instance 已注册的 CAN 实例。
 * @param timeout_ms 发送等待超时时间，单位毫秒。
 * @return 1 表示发送成功，0 表示发送失败或超时。
 */
uint8_t CANTransmit(CANInstance *instance, uint32_t timeout_ms)
{
    return CAN_Transmit(instance, timeout_ms);
}
