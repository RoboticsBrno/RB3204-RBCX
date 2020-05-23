/// Implements control frame exchange.
/// Frame byte layout: [0x00 LEN_BYTE COBS_DATA...]
/// COBS is used to avoid having zero bytes present on UART except as packet beginnings.
/// https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_ll_usart.h"

#include "Bsp.hpp"
#include "ByteFifo.hpp"
#include "coproc_codec.h"
#include "coproc_link_parser.h"
#include "rbcx.pb.h"
#include <array>

static DMA_HandleTypeDef dmaRxHandle;
static DMA_HandleTypeDef dmaTxHandle;
static ByteFifo<512> rxFifo;
static std::array<uint8_t, 257> txFrameBuf;
static rb::CoprocCodec codec;
static rb::CoprocLinkParser<EspMessage, &EspMessage_msg, &codec> parser;

void secondaryUartInit() {
    LL_USART_InitTypeDef init;
    LL_USART_StructInit(&init);
    init.BaudRate = 115200;
    init.DataWidth = LL_USART_DATAWIDTH_8B;
    init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    init.Parity = LL_USART_PARITY_NONE;
    init.StopBits = LL_USART_STOPBITS_1;
    init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    LL_USART_Init(secondaryUsart, &init);
    LL_USART_Enable(secondaryUsart);

    // UART RX runs indefinitely in circular mode
    dmaRxHandle.Instance = secondaryRxDmaChannel;
    dmaRxHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dmaRxHandle.Init.Mode = DMA_CIRCULAR;
    dmaRxHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaRxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaRxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaRxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&dmaRxHandle);
    HAL_DMA_Start(&dmaRxHandle, uint32_t(&(secondaryUsart->DR)), uint32_t(rxFifo.data()), rxFifo.size());
    LL_USART_EnableDMAReq_RX(secondaryUsart);

    // UART TX burst is started ad hoc each time
    dmaTxHandle.Instance = secondaryTxDmaChannel;
    dmaTxHandle.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dmaTxHandle.Init.Mode = DMA_NORMAL;
    dmaTxHandle.Init.MemInc = DMA_MINC_ENABLE;
    dmaTxHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    dmaTxHandle.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dmaTxHandle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&dmaTxHandle);
    LL_USART_EnableDMAReq_TX(secondaryUsart);

    pinInit(secondaryTx, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
    pinInit(secondaryRx, GPIO_MODE_AF_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);
}

bool controlLinkTxReady() {
    HAL_DMA_PollForTransfer(&dmaTxHandle, HAL_DMA_FULL_TRANSFER, 0);
    return dmaTxHandle.State == HAL_DMA_STATE_READY;
}

void controlLinkTx(const StmMessage &outgoing) {
     txFrameBuf[0] = 0x00;
     auto encodeResult = codec.encode(&StmMessage_msg, &outgoing, txFrameBuf.data() + 2, txFrameBuf.size() - 2);
     txFrameBuf[1] = (uint8_t)encodeResult;
     HAL_DMA_Start(&dmaTxHandle, uint32_t(txFrameBuf.data()), uint32_t(&secondaryUsart->DR), encodeResult + 2);
}

bool controlLinkRx(EspMessage &incoming) {
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
