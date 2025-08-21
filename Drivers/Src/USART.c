#include "USART.h"




/*
 * @brief 	USART_Init , Configures the USART peripherals
 *
 * @oaram	USART_Handle = User config structure
 *
 * @retreval None
 */
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






/*
 * @brief 	USART_TransmitData , Transmit data to the slave
 *
 * @oaram	USART_Handle = User config structure
 *
 * @oaram	pData = Data Address
 *
 * @oaram	dataSize = Data Lenght in byte
 *
 * @retreval None
 */
void USART_TransmitData(USART_HandleTypedef_t *USART_Handle, uint8_t *pData, uint16_t dataSize)
{
	uint16_t *data16Bits;

	if((USART_Handle->Init.WordLenght == USART_WORDLENGHT_9BIT) && (USART_Handle->Init.Parity == USART_PARITY_NONE))
	{
		data16Bits = (uint16_t*)pData;
	}
	else
	{
		data16Bits = NULL;
	}
	while(dataSize > 0)
	{
		while(!(USART_GetFlagStatus(USART_Handle,USART_TXE_FLAG)));
		if(data16Bits == NULL)
		{
			USART_Handle->Instance->TDR = (uint8_t)(*pData & 0xFFU);
			pData++;
			dataSize--;
		}
		else
		{
			USART_Handle->Instance->TDR =(uint16_t)(*data16Bits & (0x01FFU));
			data16Bits++;
			dataSize -=2;
		}
	}
	while(!(USART_GetFlagStatus(USART_Handle,USART_TC_FLAG)));


}






/*
 * @brief 	USART_PeriphCMD , Return the USART Flag Status
 *
 * @oaram	USART_Handle = User config structure
 *
 * @oaram	stateOfUSART = Enable or Disable
 *
 * @retreval NONE
 */
void USART_PeriphCMD(USART_HandleTypedef_t *USART_Handle, FunctionalState_t stateOfUSART)
{
	if(stateOfUSART == ENABLE)
	{
		USART_Handle->Instance->CR1 |= (0x1U << USART_CR1_UE);
	}
	else
	{
		USART_Handle->Instance->CR1 &= ~(0x1U << USART_CR1_UE);
	}
}







/*
 * @brief 	USART_GetFlagStatus , Return the USART Flag Status
 *
 * @oaram	USART_Handle = User config structure
 *
 * @oaram	FlagName = The name of the flag you want to return
 *
 * @retreval USART_FlagStatus_t
 */
USART_FlagStatus_t USART_GetFlagStatus(USART_HandleTypedef_t *USART_Handle, uint16_t FlagName)
{
	return((USART_Handle->Instance->ISR & FlagName) ? USART_FLAG_SET : USART_FLAG_RESET);
}





















































