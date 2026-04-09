#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>

/* 与 hal_bsp.h 中定义保持一致 */
extern const struct HAL_CANFD_DEV g_can0Dev;
extern const struct HAL_CANFD_DEV g_can1Dev;

#define CAN_DEVICE_COUNT        2U
#define CAN_OUT_INDEX_INVALID   0xFFFFFFFFU

/* RK3506 有效中断位 [19:0] */
#define CAN_INT_VALID_MASK      (0x000FFFFFU)

#define CAN_TX_DONE_MASK        (CAN_INT_TX_FINISH_INT_MASK)
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

#define CAN_SCLK_HZ             (200000000U)
#define CAN_SCLK_FALLBACK_HZ    (24000000U)
#define CAN_SCLK_MAX_VALID_HZ   (600000000U)
#define CAN_SCLK_24M_MIN_HZ     (23000000U)
#define CAN_SCLK_24M_MAX_HZ     (25000000U)

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

static CANInstance *can_instances[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t instance_idx = 0;
static volatile uint32_t s_canTxDone[CAN_DEVICE_COUNT] = {0};
static volatile uint32_t s_canTxFail[CAN_DEVICE_COUNT] = {0};

volatile uint32_t g_isr_hit_count = 0;
volatile uint32_t g_can_adapter_hit_count = 0;
volatile uint32_t g_can_common_hit_count = 0;
volatile uint32_t g_can_last_isr = 0;
volatile uint32_t g_can_rx_dispatch_count = 0;

static HAL_Status CAN_INTMUX_Adapter(uint32_t irq, void *args);
static void CAN_ConfigPins(void);
static void CAN_InitController(const struct HAL_CANFD_DEV *dev, struct CANFD_CONFIG *cfg);
static void CAN_DispatchRxMessage(struct CAN_REG *pReg, const struct CANFD_MSG *rx_msg);
static void CAN_DrainRxFifo(struct CAN_REG *pReg);
static void CAN_BindIntmuxOutIRQHandler(IRQn_Type outIrq);
static void CAN_ConfigIntmuxOutputIRQs(const IRQn_Type *outIrqs, uint32_t outCount);
static void CAN_EnableIntmuxSourceIRQ(const struct HAL_CANFD_DEV *dev);
static void CAN_ApplyRuntimeInterruptMask(const struct HAL_CANFD_DEV *dev);
static void CAN_ApplyModeFlags(struct CAN_REG *pReg, uint32_t canfdMode);
static void CAN_ApplyNominalTiming1M(struct CAN_REG *pReg, uint32_t sclkHz);
static void CAN_SwapBytesPerWord(uint8_t *dst, const uint8_t *src, uint8_t len);
static uint32_t CAN_GetDeviceIndexByReg(struct CAN_REG *pReg);
static uint32_t CAN_GetIntmuxSource(uint32_t irq);
static IRQn_Type CAN_GetIntmuxOutIRQ(IRQn_Type srcIrq);
static uint32_t CAN_GetIntmuxOutIndex(IRQn_Type srcIrq);

void INTMUX0_IRQHandler(void);
void INTMUX1_IRQHandler(void);
void INTMUX2_IRQHandler(void);
void INTMUX3_IRQHandler(void);

static uint32_t CAN_GetIntmuxSource(uint32_t irq)
{
    uint32_t intmuxStart = (uint32_t)NUM_INTERRUPTS + (uint32_t)INTMUX_IRQ_START_NUM;

    if (irq >= intmuxStart) {
        return irq - intmuxStart;
    }

    return irq;
}

static IRQn_Type CAN_GetIntmuxOutIRQ(IRQn_Type srcIrq)
{
    uint32_t intmuxSrcNum;

    intmuxSrcNum = (uint32_t)srcIrq - (uint32_t)NUM_INTERRUPTS - (uint32_t)INTMUX_IRQ_START_NUM;

    return (IRQn_Type)((uint32_t)INTMUX_OUT_IRQ_START_NUM +
                       (intmuxSrcNum / (uint32_t)INTMUX_NUM_INT_PER_OUT));
}

static uint32_t CAN_GetIntmuxOutIndex(IRQn_Type srcIrq)
{
    IRQn_Type outIrq = CAN_GetIntmuxOutIRQ(srcIrq);

    return (uint32_t)outIrq - (uint32_t)INTMUX_OUT_IRQ_START_NUM;
}

static uint32_t CAN_GetDeviceIndexByReg(struct CAN_REG *pReg)
{
    uint32_t i;

    for (i = 0; i < CAN_DEVICE_COUNT; i++) {
        if (s_canDevs[i]->pReg == pReg) {
            return i;
        }
    }

    return CAN_DEVICE_COUNT;
}

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

static void CAN_ConfigIntmuxOutputIRQs(const IRQn_Type *outIrqs, uint32_t outCount)
{
    uint32_t i;

    for (i = 0; i < (uint32_t)INTMUX_NUM_OUT_PER_CON; i++) {
        IRQn_Type outIrq = (IRQn_Type)((uint32_t)INTMUX_OUT_IRQ_START_NUM + i);

        HAL_NVIC_ClearPendingIRQ(outIrq);
        HAL_NVIC_DisableIRQ(outIrq);
    }

    for (i = 0; i < outCount; i++) {
        HAL_NVIC_ClearPendingIRQ(outIrqs[i]);
        CAN_BindIntmuxOutIRQHandler(outIrqs[i]);
        HAL_NVIC_SetPriority(outIrqs[i], 1, 0);
        HAL_NVIC_EnableIRQ(outIrqs[i]);
    }
}

static void CAN_EnableIntmuxSourceIRQ(const struct HAL_CANFD_DEV *dev)
{
    HAL_INTMUX_ClearPendingIRQ(dev->irqNum);
    HAL_INTMUX_EnableIRQ(dev->irqNum);
}

static void CAN_ApplyRuntimeInterruptMask(const struct HAL_CANFD_DEV *dev)
{
    WRITE_REG(dev->pReg->INT_MASK, CAN_INT_VALID_MASK & ~CAN_IRQ_UNMASK_BITS);
}

static void CAN_ApplyModeFlags(struct CAN_REG *pReg, uint32_t canfdMode)
{
    if (canfdMode & CANFD_MODE_LOOPBACK) {
        SET_BIT(pReg->MODE, CAN_MODE_RXSTX_MODE_MASK);
    } else {
        CLEAR_BIT(pReg->MODE, CAN_MODE_RXSTX_MODE_MASK);
    }
}

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

static void CAN_SwapBytesPerWord(uint8_t *dst, const uint8_t *src, uint8_t len)
{
    uint8_t base;

    for (base = 0; base < len; base = (uint8_t)(base + 4U)) {
        uint8_t i;
        uint8_t remain = (uint8_t)(len - base);
        uint8_t chunk = (remain < 4U) ? remain : 4U;

        for (i = 0; i < chunk; i++) {
            dst[base + i] = src[base + (chunk - 1U - i)];
        }
    }
}

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

static void CAN_InitController(const struct HAL_CANFD_DEV *dev, struct CANFD_CONFIG *cfg)
{
    uint32_t sclkHz;

    sclkHz = CAN_SCLK_FALLBACK_HZ;

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

    HAL_INTMUX_SetIRQHandler(dev->irqNum, CAN_INTMUX_Adapter, NULL);
}

static void CAN_DispatchRxMessage(struct CAN_REG *pReg, const struct CANFD_MSG *rx_msg)
{
    size_t i;

    for (i = 0; i < instance_idx; i++) {
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
         * 在总线上观察会表现为每4字节逆序。这里统一转换回自然顺序。
         */
        CAN_SwapBytesPerWord(payload, rx_msg->data, copyLen);
        can_instances[i]->rx_len = copyLen;
        memcpy(can_instances[i]->rx_buff, payload, copyLen);
        g_can_rx_dispatch_count++;

        if (can_instances[i]->can_module_callback) {
            can_instances[i]->can_module_callback(can_instances[i]);
        }
    }
}

static void CAN_DrainRxFifo(struct CAN_REG *pReg)
{
    uint32_t guard = 0;

    while (guard++ < 16U) {
        uint32_t frameCnt;
        uint32_t strState = READ_REG(pReg->STR_STATE);
        struct CANFD_MSG rx_msg = {0};

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

void CAN_Common_IRQHandler(struct CAN_REG *pReg)
{
    uint32_t devIdx;
    uint32_t txFailIsr = 0U;
    uint32_t isr = HAL_CANFD_GetInterrupt(pReg);

    g_can_common_hit_count++;
    g_can_last_isr = isr;
    devIdx = CAN_GetDeviceIndexByReg(pReg);

    if (devIdx < CAN_DEVICE_COUNT) {
        if (isr & CAN_TX_DONE_MASK) {
            s_canTxDone[devIdx] = 1U;
        }
        txFailIsr |= isr & (CAN_INT_BUS_OFF_INT_MASK |
                            CAN_INT_TX_ARBIT_FAIL_INT_MASK |
                            CAN_INT_AUTO_RETX_FAIL_INT_MASK);
        if (isr & (CAN_INT_ERROR_INT_MASK |
                   CAN_INT_PASSIVE_ERROR_INT_MASK |
                   CAN_INT_OVERLOAD_INT_MASK |
                   CAN_INT_ERROR_WARNING_INT_MASK)) {
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

    if (isr & CAN_RX_EVENT_MASK) {
        CAN_DrainRxFifo(pReg);
    }
}

static HAL_Status CAN_INTMUX_Adapter(uint32_t irq, void *args)
{
    uint32_t i;
    uint32_t irqSrc;

    (void)args;
    g_can_adapter_hit_count++;

    irqSrc = CAN_GetIntmuxSource(irq);
    for (i = 0; i < CAN_DEVICE_COUNT; i++) {
        uint32_t devSrc = CAN_GetIntmuxSource((uint32_t)s_canDevs[i]->irqNum);

        if (irqSrc == devSrc) {
            CAN_Common_IRQHandler(s_canDevs[i]->pReg);
            break;
        }
    }

    return HAL_OK;
}

/* 通过 HAL_NVIC_SetIRQHandler 动态绑定 */
void INTMUX0_IRQHandler(void)
{
    g_isr_hit_count++;
    HAL_INTMUX_DirectDispatch(0);
}

void INTMUX1_IRQHandler(void)
{
    g_isr_hit_count++;
    HAL_INTMUX_DirectDispatch(1);
}

void INTMUX2_IRQHandler(void)
{
    g_isr_hit_count++;
    HAL_INTMUX_DirectDispatch(2);
}

void INTMUX3_IRQHandler(void)
{
    g_isr_hit_count++;
    HAL_INTMUX_DirectDispatch(3);
}

void CAN_Service_Init(void)
{
    uint32_t i;
    uint32_t j;
    uint32_t outCount = 0;
    IRQn_Type outIrqs[CAN_DEVICE_COUNT];
    struct CANFD_CONFIG can_cfg = {0};

    can_cfg.canfdMode = 0U; /* 外总线模式（非回环） */
    can_cfg.bps = CANFD_BPS_1MBAUD;

    CAN_ConfigPins();

    for (i = 0; i < CAN_DEVICE_COUNT; i++) {
        IRQn_Type outIrq;

        CAN_InitController(s_canDevs[i], &can_cfg);

        s_canOutIndex[i] = CAN_GetIntmuxOutIndex(s_canDevs[i]->irqNum);
        outIrq = CAN_GetIntmuxOutIRQ(s_canDevs[i]->irqNum);

        for (j = 0; j < outCount; j++) {
            if (outIrqs[j] == outIrq) {
                break;
            }
        }
        if (j == outCount) {
            outIrqs[outCount++] = outIrq;
        }
    }

    CAN_ConfigIntmuxOutputIRQs(outIrqs, outCount);

    for (i = 0; i < CAN_DEVICE_COUNT; i++) {
        CAN_EnableIntmuxSourceIRQ(s_canDevs[i]);
    }

    for (i = 0; i < CAN_DEVICE_COUNT; i++) {
        CAN_ApplyRuntimeInterruptMask(s_canDevs[i]);
    }
}

uint8_t CAN_Transmit(CANInstance *_instance, uint32_t timeout_ms)
{
    uint32_t devIdx;
    uint32_t start;
    struct CANFD_MSG tx_msg = {0};

    if ((_instance == NULL) || (_instance->tx_len > sizeof(tx_msg.data))) {
        return 0;
    }

    devIdx = CAN_GetDeviceIndexByReg(_instance->can_handle);
    if (devIdx >= CAN_DEVICE_COUNT) {
        return 0;
    }

    tx_msg.stdId = _instance->tx_id;
    tx_msg.ide = CANFD_ID_STANDARD;
    tx_msg.rtr = CANFD_RTR_DATA;
    tx_msg.fdf = CANFD_FORMAT;
    tx_msg.dlc = _instance->tx_len;

    /*
     * 与接收路径对称：转换后线上字节顺序与上层缓冲一致。
     */
    CAN_SwapBytesPerWord(tx_msg.data, _instance->tx_buff, _instance->tx_len);

    /*
     * 清理上一次发送遗留标志，避免“旧 TX_FINISH”误判本次成功。
     */
    s_canTxDone[devIdx] = 0U;
    s_canTxFail[devIdx] = 0U;
    WRITE_REG(_instance->can_handle->INT, CAN_TX_DONE_MASK | CAN_TX_FAIL_MASK);

    start = HAL_GetTick();
    while (HAL_CANFD_Transmit(_instance->can_handle, &tx_msg) != HAL_OK) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return 0;
        }
    }

    start = HAL_GetTick();
    while (1) {
        if (s_canTxDone[devIdx] != 0U) {
            s_canTxDone[devIdx] = 0U;
            return 1;
        }
        if (s_canTxFail[devIdx] != 0U) {
            s_canTxFail[devIdx] = 0U;
            return 0;
        }
        if ((HAL_GetTick() - start) > timeout_ms) {
            return 0;
        }
    }
}

CANInstance *CAN_Register(CAN_Init_Config_s *config)
{
    CANInstance *ins;

    if ((config == NULL) || (instance_idx >= CAN_MX_REGISTER_CNT)) {
        return NULL;
    }

    ins = (CANInstance *)malloc(sizeof(CANInstance));
    if (!ins) {
        return NULL;
    }

    memset(ins, 0, sizeof(CANInstance));
    ins->can_handle = config->can_handle;
    ins->tx_id = config->tx_id;
    ins->rx_id = config->rx_id;
    ins->tx_len = 8;
    ins->can_module_callback = config->can_module_callback;

    can_instances[instance_idx++] = ins;

    return ins;
}
