/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2022 changfengpro
 */

#include "bsp_can.h"

#include <stdlib.h>
#include <string.h>

extern const struct HAL_CANFD_DEV g_can0Dev;
extern const struct HAL_CANFD_DEV g_can1Dev;

#define CAN_SCLK_HZ             200000000U
#define CAN_SCLK_FALLBACK_HZ    24000000U
#define CAN_SCLK_MAX_VALID_HZ   600000000U
#define CAN_SCLK_24M_MIN_HZ     23000000U
#define CAN_SCLK_24M_MAX_HZ     25000000U
#define CAN_RX_DRAIN_LIMIT      16U
#define CAN_CLASSIC_DLC_MAX     8U
#define CAN_ATF_SLOT_COUNT      5U
#define CAN_INTMUX_OUT_COUNT    INTMUX_NUM_OUT_PER_CON
#define CAN_INT_VALID_MASK      0x000FFFFFU

#define CAN_TX_DONE_MASK        CAN_INT_TX_FINISH_INT_MASK
#define CAN_TX_FAIL_MASK        (CAN_INT_BUS_OFF_INT_MASK | \
                                 CAN_INT_ERROR_INT_MASK | \
                                 CAN_INT_TX_ARBIT_FAIL_INT_MASK | \
                                 CAN_INT_PASSIVE_ERROR_INT_MASK | \
                                 CAN_INT_OVERLOAD_INT_MASK | \
                                 CAN_INT_ERROR_WARNING_INT_MASK | \
                                 CAN_INT_AUTO_RETX_FAIL_INT_MASK)

#define CAN_RX_IRQ_UNMASK_BITS  (CAN_INT_MFI_INT_MASK | \
                                 CAN_INT_RX_FINISH_INT_MASK | \
                                 CAN_INT_RXSTR_FULL_INT_MASK | \
                                 CAN_INT_RXSTR_OVERFLOW_INT_MASK)

#define CAN_TX_IRQ_UNMASK_BITS  (CAN_TX_DONE_MASK | CAN_TX_FAIL_MASK)

#define CAN_TX_EVENT_MASK       (CAN_TX_DONE_MASK | CAN_TX_FAIL_MASK)
#define CAN_RX_EVENT_MASK       (CAN_INT_MFI_INT_MASK | \
                                 CAN_INT_RX_FINISH_INT_MASK | \
                                 CAN_INT_RXSTR_FULL_INT_MASK | \
                                 CAN_INT_RXSTR_OVERFLOW_INT_MASK | \
                                 CAN_INT_ISM_WTM_INT_MASK | \
                                 CAN_INT_RXSTR_TIMEOUT_INT_MASK)

#ifndef CAN0_TX_BANK
#define CAN0_TX_BANK GPIO_BANK0
#endif
#ifndef CAN0_TX_PIN
#define CAN0_TX_PIN GPIO_PIN_C3 /* RM_IO19 */
#endif
#ifndef CAN0_RX_BANK
#define CAN0_RX_BANK GPIO_BANK0
#endif
#ifndef CAN0_RX_PIN
#define CAN0_RX_PIN GPIO_PIN_C4 /* RM_IO20 */
#endif

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

/* can instance ptrs storage, used for recv callback */
/* 在CAN产生接收中断会遍历数组,选出can_handle和rxid相同的实例,调用其回调函数 */
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = { NULL };
static uint8_t idx;

static const struct HAL_CANFD_DEV *const s_canDevs[DEVICE_CAN_CNT] = {
    &g_can0Dev,
    &g_can1Dev,
};

static uint8_t s_canControllerInitialized[DEVICE_CAN_CNT] = {
    0U,
    0U,
};
static uint8_t s_canServiceInitialized = 0U;
static uint8_t s_intmuxOutConfigured[CAN_INTMUX_OUT_COUNT] = {
    0U,
};
static uint32_t s_canInterruptEnableMask = CAN_INTERRUPT_ALL;
volatile uint32_t g_canSclkHz[DEVICE_CAN_CNT] = {
    0U,
    0U,
};

static HAL_Status CANINTMUXAdapter(uint32_t irq, void *args);

/**
 * @brief 屏蔽并清除启动阶段可能遗留的 CAN 相关中断状态。
 */
static void CANMaskBootEnabledNVICIrqs(void)
{
    uint32_t i;
    uint8_t outCleared[CAN_INTMUX_OUT_COUNT] = { 0U };
    uint32_t intmuxStart = (uint32_t)NUM_INTERRUPTS + (uint32_t)INTMUX_IRQ_START_NUM;

    for (i = 0U; i < DEVICE_CAN_CNT; i++) {
        uint32_t irq = (uint32_t)s_canDevs[i]->irqNum;
        uint32_t intmuxSrc = (irq >= intmuxStart) ? (irq - intmuxStart) : irq;
        uint32_t outIdx = intmuxSrc / (uint32_t)INTMUX_NUM_INT_PER_OUT;

        HAL_INTMUX_DisableIRQ(irq);
        HAL_INTMUX_ClearPendingIRQ(irq);

        if (outIdx < CAN_INTMUX_OUT_COUNT) {
            IRQn_Type outIrq = (IRQn_Type)((uint32_t)INTMUX_OUT_IRQ_START_NUM + outIdx);

            if (outCleared[outIdx] == 0U) {
                HAL_NVIC_ClearPendingIRQ(outIrq);
                outCleared[outIdx] = 1U;
            }
        }
    }
}

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 计算 INTMUX 输入中断对应的源号。
 * @param irq 原始中断号。
 * @return INTMUX 内部源号。
 */
static uint32_t CANGetIntmuxSource(uint32_t irq)
{
    uint32_t intmuxStart = (uint32_t)NUM_INTERRUPTS + (uint32_t)INTMUX_IRQ_START_NUM;

    if (irq >= intmuxStart) {
        return irq - intmuxStart;
    }

    return irq;
}

/**
 * @brief 计算 CAN 源中断对应的 INTMUX 输出索引。
 * @param irq 原始中断号。
 * @return INTMUX 输出索引。
 */
static uint32_t CANGetIntmuxOutIndex(uint32_t irq)
{
    return CANGetIntmuxSource(irq) / (uint32_t)INTMUX_NUM_INT_PER_OUT;
}

/**
 * @brief 根据 CAN 源中断号计算 INTMUX 输出中断号。
 * @param irq 原始中断号。
 * @return INTMUX 输出中断号。
 */
static IRQn_Type CANGetIntmuxOutIRQ(uint32_t irq)
{
    uint32_t outIdx = CANGetIntmuxOutIndex(irq);

    return (IRQn_Type)((uint32_t)INTMUX_OUT_IRQ_START_NUM + outIdx);
}

/**
 * @brief 根据 CAN 寄存器基址查找控制器索引。
 * @param can_handle CAN 控制器基址。
 * @return 找到时返回控制器索引，失败时返回 DEVICE_CAN_CNT。
 */
static uint32_t CANGetDeviceIndex(struct CAN_REG *can_handle)
{
    uint32_t i;

    for (i = 0U; i < DEVICE_CAN_CNT; i++) {
        if (s_canDevs[i]->pReg == can_handle) {
            return i;
        }
    }

    return DEVICE_CAN_CNT;
}

/**
 * @brief 根据 CAN 寄存器基址获取设备描述。
 * @param can_handle CAN 控制器基址。
 * @return 设备描述指针。
 */
static const struct HAL_CANFD_DEV *CANGetDevice(struct CAN_REG *can_handle)
{
    uint32_t devIdx = CANGetDeviceIndex(can_handle);

    if (devIdx >= DEVICE_CAN_CNT) {
        return NULL;
    }

    return s_canDevs[devIdx];
}

/**
 * @brief 获取当前运行期需要放行的中断位。
 * @return INT_MASK 中需要解屏蔽的中断位组合。
 */
static uint32_t CANGetRuntimeInterruptUnmaskBits(void)
{
    uint32_t unmaskBits = 0U;

    if ((s_canInterruptEnableMask & CAN_INTERRUPT_RX) != 0U) {
        unmaskBits |= CAN_RX_IRQ_UNMASK_BITS;
    }
    if ((s_canInterruptEnableMask & CAN_INTERRUPT_TX) != 0U) {
        unmaskBits |= CAN_TX_IRQ_UNMASK_BITS;
    }

    return unmaskBits;
}

/**
 * @brief 应用运行期中断屏蔽策略。
 * @param dev CAN 控制器设备描述。
 */
static void CANApplyInterruptMask(const struct HAL_CANFD_DEV *dev)
{
    uint32_t unmaskBits = CANGetRuntimeInterruptUnmaskBits();

    WRITE_REG(dev->pReg->INT_MASK, CAN_INT_VALID_MASK & ~unmaskBits);
}

/**
 * @brief 配置 RK3506 上 CAN0/CAN1 的 IOMUX 与 RMIO 复用。
 */
static void CANConfigPins(void)
{
    HAL_PINCTRL_SetIOMUX(CAN0_TX_BANK, CAN0_TX_PIN, PIN_CONFIG_MUX_FUNC7);
    HAL_PINCTRL_SetIOMUX(CAN0_RX_BANK, CAN0_RX_PIN, PIN_CONFIG_MUX_FUNC7);
    HAL_PINCTRL_SetIOMUX(CAN1_TX_BANK, CAN1_TX_PIN, PIN_CONFIG_MUX_FUNC7);
    HAL_PINCTRL_SetIOMUX(CAN1_RX_BANK, CAN1_RX_PIN, PIN_CONFIG_MUX_FUNC7);

    HAL_PINCTRL_SetRMIO(CAN0_TX_BANK, CAN0_TX_PIN, RMIO_CAN0_TX);
    HAL_PINCTRL_SetRMIO(CAN0_RX_BANK, CAN0_RX_PIN, RMIO_CAN0_RX);
    HAL_PINCTRL_SetRMIO(CAN1_TX_BANK, CAN1_TX_PIN, RMIO_CAN1_TX);
    HAL_PINCTRL_SetRMIO(CAN1_RX_BANK, CAN1_RX_PIN, RMIO_CAN1_RX);
}

/**
 * @brief 在 24MHz SCLK 场景下覆写 1Mbps 时序参数。
 * @param pReg CAN 控制器基址。
 * @param sclkHz 当前 CAN SCLK 频率。
 */
static void CANApplyNominalTiming1M(struct CAN_REG *pReg, uint32_t sclkHz)
{
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
 * @brief 初始化单个 CAN 控制器。
 * @param dev CAN 控制器设备描述。
 * @param cfg HAL CANFD 初始化参数。
 */
static void CANInitController(const struct HAL_CANFD_DEV *dev, struct CANFD_CONFIG *cfg)
{
    uint32_t devIdx = CANGetDeviceIndex(dev->pReg);
    uint32_t sclkHz = CAN_SCLK_FALLBACK_HZ;

    if (devIdx >= DEVICE_CAN_CNT) {
        return;
    }
    if (s_canControllerInitialized[devIdx] != 0U) {
        return;
    }

#if defined(HAL_CRU_MODULE_ENABLED) && !defined(IS_FPGA)
    HAL_CRU_ClkEnable(dev->pclkGateID);
    HAL_CRU_ClkEnable(dev->sclkGateID);
    HAL_CRU_ClkSetFreq(dev->sclkID, CAN_SCLK_HZ);
    sclkHz = HAL_CRU_ClkGetFreq(dev->sclkID);
    if ((sclkHz == 0U) || (sclkHz > CAN_SCLK_MAX_VALID_HZ)) {
        HAL_CRU_ClkSetFreq(dev->sclkID, CAN_SCLK_FALLBACK_HZ);
        sclkHz = HAL_CRU_ClkGetFreq(dev->sclkID);
    }
#endif
    g_canSclkHz[devIdx] = sclkHz;

    if (HAL_CANFD_Init(dev->pReg, cfg) != HAL_OK) {
        return;
    }
    CANApplyNominalTiming1M(dev->pReg, sclkHz);
    if (HAL_CANFD_Start(dev->pReg) != HAL_OK) {
        return;
    }

    /*
     * RK3506 RX path depends on these controller-local settings. If we leave
     * them at reset / residual values, a single received frame can keep
     * INTM non-empty and retrigger RX interrupts indefinitely.
     */
    WRITE_REG(dev->pReg->WAVE_FILTER_CFG, 0U);
    WRITE_REG(dev->pReg->RXINT_CTRL, 1U);
    WRITE_REG(dev->pReg->ATF_CTL, 0x1FU);

    WRITE_REG(dev->pReg->INT_MASK, CAN_INT_VALID_MASK);
    WRITE_REG(dev->pReg->INT, CAN_INT_VALID_MASK);

    HAL_INTMUX_SetIRQHandler(dev->irqNum, CANINTMUXAdapter, NULL);
    s_canControllerInitialized[devIdx] = 1U;
}

/**
 * @brief 为注册实例分配一个 RK3506 CAN 硬件接收过滤槽。
 *
 * @note 每个 CAN 控制器独立维护 ATF[5] 过滤槽。
 *       每注册一个实例，就占用一个槽位，并将该槽配置成
 *       “仅匹配 instance->rx_id 的标准帧”。
 *
 * @param instance can instance owned by specific module
 */
static void CANAddFilter(CANInstance *instance)
{
    static uint8_t can0_filter_idx = 0U;
    static uint8_t can1_filter_idx = 0U;
    uint8_t *filter_idx;
    uint32_t slot;
    uint32_t atfCtl;

    if ((instance == NULL) || (instance->can_handle == NULL)) {
        return;
    }

    if (instance->can_handle == g_can0Dev.pReg) {
        filter_idx = &can0_filter_idx;
    } else if (instance->can_handle == g_can1Dev.pReg) {
        filter_idx = &can1_filter_idx;
    } else {
        return;
    }

    slot = *filter_idx;
    if (slot >= CAN_ATF_SLOT_COUNT) {
        return;
    }

    /*
     * RK3576/RK3506 ATF list mode is closest to the old bxCAN IDLIST usage:
     * one hardware slot keeps two exact IDs. We write the same rx_id twice so
     * this slot only accepts the target standard ID.
     *
     * ATF_CTL bit semantics follow the Linux driver:
     * a bit value of 1 disables the corresponding filter slot.
     */
    WRITE_REG(instance->can_handle->ATF[slot], instance->rx_id & CAN_ATF0_ID_MASK);
    WRITE_REG(instance->can_handle->ATFM[slot],
              (instance->rx_id & CAN_ATFM0_ID_MASK) | CAN_ATFM0_MASK_MASK);

    atfCtl = READ_REG(instance->can_handle->ATF_CTL);
    atfCtl &= ~(1UL << slot);
    WRITE_REG(instance->can_handle->ATF_CTL, atfCtl);

    *filter_idx = (uint8_t)(slot + 1U);
}

/**
 * @brief 按 32bit 字边界整理 CAN 载荷字节序。
 * @param dst 目标缓冲区。
 * @param src 源缓冲区。
 * @param len 有效字节数。
 */
static void CANSwapBytesPerWord(uint8_t *dst, const uint8_t *src, uint8_t len)
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
 * @brief 将接收到的报文分发给已注册实例。
 * @param pReg 收包控制器基址。
 * @param rx_msg 接收到的报文。
 */
static void CANDispatchRxMessage(struct CAN_REG *pReg, const struct CANFD_MSG *rx_msg)
{
    uint32_t i;

    for (i = 0U; i < idx; i++) {
        uint8_t copyLen;

        if (can_instance[i] == NULL) {
            continue;
        }
        if (can_instance[i]->can_handle != pReg) {
            continue;
        }
        if (can_instance[i]->rx_id != rx_msg->stdId) {
            continue;
        }

        copyLen = rx_msg->dlc;
        if (copyLen > sizeof(can_instance[i]->rx_buff)) {
            copyLen = sizeof(can_instance[i]->rx_buff);
        }

        memset(can_instance[i]->rx_buff, 0, sizeof(can_instance[i]->rx_buff));
        CANSwapBytesPerWord(can_instance[i]->rx_buff, rx_msg->data, copyLen);
        can_instance[i]->rx_len = copyLen;

        if (can_instance[i]->can_module_callback != NULL) {
            can_instance[i]->can_module_callback(can_instance[i]);
        }
    }
}

volatile uint32_t g_can_cnt = 0U; /* for debug */
volatile uint32_t g_can_rx_irq_cnt = 0U; /* for debug */
volatile uint32_t g_can_last_rx_std_id = 0U; /* for debug */
volatile uint32_t g_can_last_rx_dlc = 0U; /* for debug */
volatile uint8_t g_can_last_rx_data[8] = { 0U }; /* for debug */
volatile uint32_t g_can_irq_entry_cnt = 0U; /* for debug */
volatile uint32_t g_can_last_isr = 0U; /* for debug */
volatile uint32_t g_can_last_str_state = 0U; /* for debug */

/**
 * @brief 尽快取空当前接收 FIFO 中的报文。
 * @param pReg CAN 控制器基址。
 */
static void CANDrainRxFifo(struct CAN_REG *pReg)
{
    uint32_t guard = 0U;

    while (guard++ < CAN_RX_DRAIN_LIMIT) {
        uint32_t strState = READ_REG(pReg->STR_STATE);
        struct CANFD_MSG rx_msg = { 0 };

        g_can_last_str_state = strState;

        /*
         * On RK3506 the INTM_FRAME_CNT field reflects storage occupancy,
         * not a literal number of complete CAN frames. Use the empty flag
         * to decide whether another packet can be popped safely.
         */
        if ((strState & CAN_STR_STATE_INTM_EMPTY_MASK) != 0U) {
            break;
        }

        if (HAL_CANFD_Receive(pReg, &rx_msg) != HAL_OK) {
            break;
        }

        if (rx_msg.ide == CANFD_ID_STANDARD) {
            uint8_t copyLen = rx_msg.dlc;

            g_can_rx_irq_cnt++;
            g_can_last_rx_std_id = rx_msg.stdId;
            g_can_last_rx_dlc = rx_msg.dlc;
            if (copyLen > sizeof(g_can_last_rx_data)) {
                copyLen = sizeof(g_can_last_rx_data);
            }
            memset((void *)g_can_last_rx_data, 0, sizeof(g_can_last_rx_data));
            memcpy((void *)g_can_last_rx_data, rx_msg.data, copyLen);
            CANDispatchRxMessage(pReg, &rx_msg);
        }
    }
}

/**
 * @brief 处理单个 CAN 控制器的发送中断。
 * @param pReg CAN 控制器基址。
 * @param isr 已读取的中断状态。
 */
__attribute__((noinline)) static void CANTxIRQHandler(struct CAN_REG *pReg, uint32_t isr)
{
    /* TX 完成/失败事件统一收敛到这里，便于后续扩展发送状态或回调。 */
    (void)pReg;
    (void)isr;

    g_can_cnt++;
}

/**
 * @brief 处理单个 CAN 控制器的接收中断。
 * @param pReg CAN 控制器基址。
 * @param isr 已读取的中断状态。
 */
__attribute__((noinline)) static void CANRxIRQHandler(struct CAN_REG *pReg, uint32_t isr)
{
    if ((isr & CAN_RX_EVENT_MASK) != 0U) {
        CANDrainRxFifo(pReg);
    }
}

/**
 * @brief 处理单个 CAN 控制器的收发中断。
 * @param pReg CAN 控制器基址。
 */
static void CANCommonIRQHandler(struct CAN_REG *pReg)
{
    uint32_t isr = HAL_CANFD_GetInterrupt(pReg);

    g_can_irq_entry_cnt++;
    g_can_last_isr = isr;

    if (((s_canInterruptEnableMask & CAN_INTERRUPT_TX) != 0U) &&
        ((isr & CAN_TX_EVENT_MASK) != 0U)) {
        CANTxIRQHandler(pReg, isr);
    }

    if (((s_canInterruptEnableMask & CAN_INTERRUPT_RX) != 0U) &&
        ((isr & CAN_RX_EVENT_MASK) != 0U)) {
        CANRxIRQHandler(pReg, isr);
    }
}

/**
 * @brief 配置当前 CAN 使用到的 INTMUX 输出中断。
 * @param srcIrq CAN 源中断号。
 */
static void CANConfigIntmuxOutputIRQ(uint32_t srcIrq)
{
    uint32_t outIdx = CANGetIntmuxOutIndex(srcIrq);
    IRQn_Type outIrq = CANGetIntmuxOutIRQ(srcIrq);
    uint32_t i;
    uint32_t startIrq;

    if (outIdx >= CAN_INTMUX_OUT_COUNT) {
        return;
    }

    if (s_intmuxOutConfigured[outIdx] != 0U) {
        return;
    }

    startIrq = (uint32_t)NUM_INTERRUPTS + (uint32_t)INTMUX_IRQ_START_NUM +
               outIdx * (uint32_t)INTMUX_NUM_INT_PER_OUT;
    for (i = 0U; i < (uint32_t)INTMUX_NUM_INT_PER_OUT; i++) {
        HAL_INTMUX_DisableIRQ(startIrq + i);
    }

    HAL_NVIC_ClearPendingIRQ(outIrq);
    HAL_NVIC_SetPriority(outIrq, 1U, 0U);
    HAL_NVIC_EnableIRQ(outIrq);
    s_intmuxOutConfigured[outIdx] = 1U;
}

/**
 * @brief INTMUX 适配层，按中断源转发到对应 CAN 控制器。
 * @param irq 触发的 INTMUX 源中断号。
 * @param args 私有参数，当前未使用。
 * @return HAL_Status。
 */
static HAL_Status CANINTMUXAdapter(uint32_t irq, void *args)
{
    uint32_t i;
    uint32_t irqSrc;

    (void)args;

    irqSrc = CANGetIntmuxSource(irq);
    for (i = 0U; i < DEVICE_CAN_CNT; i++) {
        uint32_t devSrc;

        if (s_canControllerInitialized[i] == 0U) {
            continue;
        }

        devSrc = CANGetIntmuxSource((uint32_t)s_canDevs[i]->irqNum);
        if (irqSrc == devSrc) {
            CANCommonIRQHandler(s_canDevs[i]->pReg);
            break;
        }
    }

    return HAL_OK;
}

/* ----------------------- two extern callable function ----------------------- */

/**
 * @brief 初始化 CAN 服务。
 */
void BSP_CAN_Init(void)
{
    uint32_t i;
    struct CANFD_CONFIG can_cfg = { 0 };

    if (s_canServiceInitialized != 0U) {
        return;
    }

    CANMaskBootEnabledNVICIrqs();

    can_cfg.canfdMode = 0U;
    can_cfg.bps = CANFD_BPS_1MBAUD;

    CANConfigPins();

    for (i = 0U; i < DEVICE_CAN_CNT; i++) {
        CANInitController(s_canDevs[i], &can_cfg);
        if (s_canControllerInitialized[i] == 0U) {
            continue;
        }
        CANConfigIntmuxOutputIRQ((uint32_t)s_canDevs[i]->irqNum);
        HAL_INTMUX_ClearPendingIRQ(s_canDevs[i]->irqNum);
        HAL_INTMUX_EnableIRQ(s_canDevs[i]->irqNum);
        CANApplyInterruptMask(s_canDevs[i]);
    }

    s_canServiceInitialized = 1U;
}

void CANSetInterruptEnable(uint32_t interrupt_mask)
{
    uint32_t i;

    s_canInterruptEnableMask = interrupt_mask & CAN_INTERRUPT_ALL;

    if (s_canServiceInitialized == 0U) {
        return;
    }

    for (i = 0U; i < DEVICE_CAN_CNT; i++) {
        if (s_canControllerInitialized[i] == 0U) {
            continue;
        }
        CANApplyInterruptMask(s_canDevs[i]);
    }
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *instance;
    uint32_t i;

    if (config == NULL) {
        return NULL;
    }

    if (s_canServiceInitialized == 0U) {
        return NULL;
    }
    if (idx >= CAN_MX_REGISTER_CNT) {
        return NULL;
    }
    if (CANGetDevice(config->can_handle) == NULL) {
        return NULL;
    }

    for (i = 0U; i < idx; i++) {
        if (can_instance[i] == NULL) {
            continue;
        }
        if ((can_instance[i]->rx_id == config->rx_id) &&
            (can_instance[i]->can_handle == config->can_handle)) {
            return NULL;
        }
    }

    instance = (CANInstance *)malloc(sizeof(CANInstance));
    if (instance == NULL) {
        return NULL;
    }

    memset(instance, 0, sizeof(CANInstance));
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->tx_len = CAN_CLASSIC_DLC_MAX;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);
    can_instance[idx++] = instance;

    return instance;
}

void CANSetDLC(CANInstance *instance, uint8_t length)
{
    if (instance == NULL) {
        return;
    }
    if ((length == 0U) || (length > CAN_CLASSIC_DLC_MAX)) {
        return;
    }

    instance->tx_len = length;
}

uint8_t CANTransmit(CANInstance *instance, uint32_t timeout_ms)
{
    struct CANFD_MSG tx_msg = { 0 };
    uint32_t start;

    if ((instance == NULL) || (instance->can_handle == NULL)) {
        return 0U;
    }
    if (instance->tx_len > sizeof(tx_msg.data)) {
        return 0U;
    }

    tx_msg.stdId = instance->tx_id;
    tx_msg.ide = CANFD_ID_STANDARD;
    tx_msg.rtr = CANFD_RTR_DATA;
    tx_msg.fdf = CANFD_FORMAT;
    tx_msg.dlc = instance->tx_len;
    CANSwapBytesPerWord(tx_msg.data, instance->tx_buff, instance->tx_len);

    start = HAL_GetTick();
    while (HAL_CANFD_Transmit(instance->can_handle, &tx_msg) != HAL_OK) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return 0U;
        }
    }

    return 1U;
}
