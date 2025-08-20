#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f767xx.h"

/*
 * @def_group MODE_Types
 */
#define USART_MODE_TX						((uint32_t)(0x8))
#define USART_MODE_RX						((uint32_t)(0x4))
#define USART_MODE_TX_RX					((uint32_t)(0xC))


/*
 * @def_group WordLenght_Types
 */
#define USART_WORDLENGHT_8BIT				((uint32_t)(0x00000000))
#define USART_WORDLENGHT_7BIT				((uint32_t)(0x10000000))
#define USART_WORDLENGHT_9BIT				((uint32_t)(0x00001000))

/*
 * @def_group Parity_Modes
 */
#define USART_PARITY_NONE					((uint32_t)(0x000))
#define USART_PARITY_EVEN					((uint32_t)(0x400))
#define USART_PARITY_ODD					((uint32_t)(0x600))

/*
 * @def_group Stop_Modes
 */
#define USART_STOPBITS_ONE					((uint32_t)(0x0000))
#define USART_STOPBITS_HALF					((uint32_t)(0x1000))
#define USART_STOPBITS_TWO					((uint32_t)(0x2000))
#define USART_STOPBITS_ONE_HALF 			((uint32_t)(0x3000))

/*
 * @def_group OverSampling_Modes
 */
#define USART_OVERSAMPLING_BY_16			((uint32_t)(0x0000))
#define USART_OVERSAMPLING_BY_8				((uint32_t)(0x8000))

/*
 * @def_group HardwareFlowControl_Modes
 */

#define USART_HFC_NONE						((uint32_t)(0x0000))
#define USART_HFC_CTS_ENABLE				((uint32_t)(0x0200))
#define USART_HFC_RTS_ENABLE				((uint32_t)(0x0100))
#define USART_HFC_CTS_RTS_ENABLE			((uint32_t)(0x0300))



typedef struct
{
	uint32_t Mode;							// Transmisson and Reception modes 		@def_group MODE_Types
	uint32_t BaudRate;
	uint32_t WordLenght;					// 8 bits and 9 bits Word Lenght 		@def_group WordLenght_Types
	uint32_t Parity;						// Even And Odd parity modes 			@def_group Parity_Modes
	uint32_t StopBits;						// Stop Bits Modes						@def_group Stop_Modes
	uint32_t OverSampling;					// Over Sampling Modes					@def_group OverSampling_Modes
	uint32_t HardwareFlowControl;			// Hardware Flow Control Modes			@def_group HardwareFlowControl_Modes

}USART_InitTypedef_t;

typedef struct
{
	USART_TypeDef_t *Instance;
	USART_InitTypedef_t Init;

}USART_HandleTypedef_t;


void USART_Init(USART_HandleTypedef_t *USART_Handle);





#endif /* INC_USART_H_ */
