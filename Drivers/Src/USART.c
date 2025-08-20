#include "USART.h"

void USART_Init(USART_HandleTypedef_t *USART_Handle)
{
	uint32_t tempValue = 0;
	/*********************************** CR1 ***************************************/
	// Mode, OverSampling, Parity, WordLenght
	tempValue = USART_Handle->Instance->CR1;
	tempValue |= (USART_Handle->Init.Mode) | (USART_Handle->Init.OverSampling) | (USART_Handle->Init.Parity) | (USART_Handle->Init.WordLenght);
	USART_Handle->Instance->CR1 = tempValue;

	/*********************************** CR2 ***************************************/
	// StopBits
	tempValue = USART_Handle->Instance->CR2;

	tempValue &= ~(0x3U << USART_CR2_STOP);
	tempValue |= (USART_Handle->Init.StopBits);
	USART_Handle->Instance->CR2 = tempValue;

	/*********************************** CR3 ***************************************/
	// HardwareFlowControl
	tempValue = USART_Handle->Instance->CR3;
	tempValue |= (USART_Handle->Init.HardwareFlowControl);
	USART_Handle->Instance->CR3 = tempValue;

}
