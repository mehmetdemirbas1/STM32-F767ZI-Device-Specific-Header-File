#include "EXTI.h"


/*
 * @brief 	EXTI_Init for valid GPIO port and Line number
 *
 * @oaram	EXTI_InitStruct = User config structure
 *
 * @retreval None
 */
void EXTI_Init(EXTI_InitTypeDef_t *EXTI_InitStruct)
{
	uint32_t tempValue = 0;
	tempValue = (uint32_t)EXTI_BASE_ADDR;

	EXTI->IMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
	EXTI->EMR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

	if(EXTI_InitStruct->EXTI_LineCmd !=DISABLE)
	{
		tempValue += EXTI_InitStruct->Mode;
		*((volatile uint32_t*)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);

		tempValue = (uint32_t)EXTI_BASE_ADDR;

		EXTI->RTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);
		EXTI->FTSR &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

		if(EXTI_InitStruct->TriggerSelection == EXTI_TRIGGER_RF_EDGE)
		{
			EXTI->RTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
			EXTI->FTSR |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
		}
		else
		{
			tempValue += EXTI_InitStruct->TriggerSelection;
			*((volatile uint32_t*)tempValue) |= (0x1U << EXTI_InitStruct->EXTI_LineNumber);
		}
	}
	else
	{
		tempValue = (uint32_t)EXTI_BASE_ADDR;

		tempValue += EXTI_InitStruct->Mode;
		*((volatile uint32_t*)tempValue) &= ~(0x1U << EXTI_InitStruct->EXTI_LineNumber);

	}


}

/*
 * @brief 	EXTI_LineConfig , Configures the port and pin for SYSCFG
 * @oaram	EXTI_PortSource = User choose our EXTI Port Sources @def_group Port_Values
 *
 * @param	EXTI_LineSource = user choose our EXTI Line Sources @def_group Line_Values
 *
 * @retreval None
 */
void EXTI_LineConfig(uint8_t EXTI_PortSource, uint8_t EXTI_LineSource)
{
	uint32_t tempValue = 0;
    tempValue = SYSCFG->EXTI_CR[EXTI_LineSource >> 2U];
    tempValue &= ~(0xF << (EXTI_LineSource & 0x3U)*4);
    tempValue = (EXTI_PortSource <<(EXTI_LineSource & 0x3U)*4 );
    SYSCFG->EXTI_CR[EXTI_LineSource >> 2U] = tempValue;

}

/*
 * @brief 	NVIC_EnableInterrupt
 *
 * @oaram	IRQNumber = IRQ number of line
 *
 * @retreval None
 */
void NVIC_EnableInterrupt(IRQNumber_TypeDef_t IRQNumber)
{
	uint32_t tempValue = 0;

	tempValue = *((IRQNumber >>5U)+ NVIC_ISER0 );
	tempValue &= ~(0x1U << (IRQNumber & 0x1FU));
	tempValue |= (0x1U << (IRQNumber & 0x1FU));
	*((IRQNumber >>5U)+ NVIC_ISER0 ) = tempValue;
}











