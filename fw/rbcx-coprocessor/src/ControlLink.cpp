/// Implements control frame exchange.
/// Frame byte layout: [0x00 LEN_BYTE COBS_DATA...]
/// COBS is used to avoid having zero bytes present on UART except as packet beginnings.
/// https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing

#include "FreeRTOS.h"

#include "message_buffer.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"

#include "Bsp.hpp"
#include "coproc_codec.h"
#include "coproc_link_parser.h"
#include "rbcx.pb.h"
#include "utils/ByteFifo.hpp"
#include <array>

static DMA_HandleTypeDef dmaRxHandle;
static DMA_HandleTypeDef dmaTxHandle;
static rb::CoprocCodec codec;
static rb::CoprocLinkParser<CoprocReq, &CoprocReq_msg, &codec> parser;
static ByteFifo<512> rxFifo;

// Encode TX frame in txEncodeBuf, push to txMessageBuf, move to txDmaBuf and send via DMA.
static std::array<uint8_t, codec.MaxFrameSize> txEncodeBuf;
static std::array<uint8_t, codec.MaxFrameSize> txDmaBuf;

static StaticMessageBuffer_t _txMessageBufStruct;
static uint8_t _txMessageBufStorage[512];
static MessageBufferHandle_t txMessageBuf;

void controlUartInit() {
    LL_USART_InitTypeDef init;
    LL_USART_StructInit(&init);
    init.BaudRate = 115200;
    init.DataWidth = LL_USART_DATAWIDTH_8B;
    init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    init.Parity = LL_USART_PARITY_NONE;
    init.StopBits = LL_USART_STOPBITS_1;
    init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    LL_USART_Init(controlUart, &init);
    LL_USART_Enable(controlUart);

    // UART RX runs indefinitely in circular mode
    dmaRxHandle.Instance = controlUartRxDmaChannel;
    dmaRxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dmaRxHandle.Init.Mode = DMA_CIRCULAR;
    dmaRxHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaRxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaRxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaRxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&dmaRxHandle);
    HAL_DMA_Start(&dmaRxHandle, uint32_t(&(controlUart->DR)),
        uint32_t(rxFifo.data()), rxFifo.size());
    LL_USART_EnableDMAReq_RX(controlUart);

    // UART TX burst is started ad hoc each time
    dmaTxHandle.Instance = controlUartTxDmaChannel;
    dmaTxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dmaTxHandle.Init.Mode = DMA_NORMAL;
    dmaTxHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaTxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaTxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaTxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&dmaTxHandle);
    HAL_NVIC_SetPriority(controlUartTxDmaIRQn, controlUartTxDmaIrqPrio, 0);
    HAL_NVIC_EnableIRQ(controlUartTxDmaIRQn);
    LL_USART_EnableDMAReq_TX(controlUart);

    txMessageBuf = xMessageBufferCreateStatic(sizeof(_txMessageBufStorage),
        _txMessageBufStorage, &_txMessageBufStruct);

    pinInit(
        controlUartTxPin, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    pinInit(controlUartRxPin, GPIO_MODE_AF_INPUT, GPIO_PULLUP,
        GPIO_SPEED_FREQ_HIGH);
}

bool controlLinkRx(CoprocReq& incoming) {
    int rxHead = rxFifo.size() - __HAL_DMA_GET_COUNTER(&dmaRxHandle);
    rxFifo.setHead(rxHead);

    while (rxFifo.hasData()) {
        if (parser.add(rxFifo.pop())) {
            incoming = parser.lastMessage();
            return true;
        }
    }
    return false;
}

void controlLinkTx(const CoprocStat& outgoing) {
    auto encodedSize = codec.encodeWithHeader(
        &CoprocStat_msg, &outgoing, txEncodeBuf.data(), txEncodeBuf.size());

    auto sendResult
        = xMessageBufferSend(txMessageBuf, txEncodeBuf.data(), encodedSize, 0);
    assert(sendResult != 0);

    HAL_NVIC_SetPendingIRQ(controlUartTxDmaIRQn);
}

extern "C" void CONTROLUART_TX_DMA_HANDLER() {
    HAL_DMA_IRQHandler(&dmaTxHandle);
    if (dmaTxHandle.State == HAL_DMA_STATE_READY) {
        BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
        auto len = xMessageBufferReceiveFromISR(txMessageBuf, txDmaBuf.data(),
            txDmaBuf.size(), &pxHigherPriorityTaskWoken);

        if (len > 0) {
            HAL_DMA_Start_IT(&dmaTxHandle, uint32_t(txDmaBuf.data()),
                uint32_t(&controlUart->DR), len);
        }
        portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
    }
}
