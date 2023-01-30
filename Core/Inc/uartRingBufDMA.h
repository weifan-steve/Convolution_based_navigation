/************************************************************************************
FILE			:uartRingBufDMA.h
DESCRIPTION		:
*************************************************************************************/

#ifndef INC_UARTRINGBUFDMA_H_
#define INC_UARTRINGBUFDMA_H_

#define deck_uart &huart8
#define device_uart &huart7
#define pc_uart &huart4

//#define UART4 huart4
#define DMA_UART4 hdma_uart4_rx
//#define UART7 huart7
#define DMA_UART7 hdma_uart7_rx
//#define UART8 huart8
#define DMA_UART8 hdma_uart8_rx

/* Define the Size Here */
#define RxBuf_SIZE 256
#define MainBuf_SIZE 512

#endif /* INC_UARTRINGBUFDMA_H_ */
