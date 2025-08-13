#include "GPIO.h"

/*
 * @brief 	GPIO_Init , Configures the ports and pins
 * @oaram	GPIOx = GPIO Port Base Addresses
 *
 * @param	GPIO_ConfigStruct = User Config Structures
 *
 * @retreval None
 */

void GPIO_Init(GPIO_TypeDef_t *GPIOx, GPIO_InitTypeDef_t *GPIO_ConfigStruct)
{

	uint32_t position;
	uint32_t fakePosition = 0;
	uint32_t lastPosition = 0;
	for(position = 0; position <16; position++)
	{
		fakePosition = (0x1 << position);
		lastPosition = (uint32_t)(GPIO_ConfigStruct->PinNumbers)&fakePosition;
		if(fakePosition == lastPosition)
		{
			/* MODE CONFIG */
			uint32_t tempValue=0;
			tempValue = GPIOx->MODER;
			tempValue &= ~(0x3U << (position*2));
			tempValue |= (GPIO_ConfigStruct->Mode <<(position*2));
			GPIOx->MODER = tempValue;
			if(GPIO_ConfigStruct->Mode == GPIO_MODE_OUTPUT || GPIO_ConfigStruct->Mode == GPIO_MODE_AF)
			{
				/* OTYPE CONFIG */
				tempValue =GPIOx->OTYPER;
				tempValue &= ~(0x1U << position);
				tempValue |= (GPIO_ConfigStruct->Otype <<(position));
				GPIOx->OTYPER = tempValue;

				/* OSPEED CONFIG */
				tempValue = GPIOx->OSPEEDR;
				tempValue &= ~(0x3U << (position*2));
				tempValue |= (GPIO_ConfigStruct->Speed <<(position*2));
				GPIOx->OSPEEDR = tempValue;
			}
			/* PUPDR CONFIG */
			tempValue = GPIOx->PUPDR;
			tempValue &= ~(0x3U << (position*2));
			tempValue |= (GPIO_ConfigStruct->PuPd <<(position*2));
			GPIOx->PUPDR = tempValue;

			if(GPIO_ConfigStruct->Mode == GPIO_MODE_AF)
			{
				tempValue = GPIOx->AFR[position >> 3U];
				tempValue &= ~(0xFU << ((position & 0x7U)*4));
				tempValue |= (GPIO_ConfigStruct->AlernateFunction << ((position & 0x7U)*4));
				GPIOx->AFR[position >> 3U] = tempValue;
			}

		}

	}
}


/*
 * @brief 	GPIO_WritePin , Makes pins high or low
 * @oaram	GPIOx = GPIO Port Base Addresses
 *
 * @param	Pin_Number = GPIO Pin numbers 0-15
 *
 *  @param  PinState = GPIO_Pin_Set or GPIO_Pin_Reset
 * @retreval None
 */
void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, uint16_t Pin_Number, GPIO_PinState_t PinState)
{
	if(PinState == GPIO_Pin_Set)
	{
		GPIOx->BSSR = Pin_Number;
	}
	else
	{
		GPIOx->BSSR = (Pin_Number << 16U);
	}
}


/*
 * @brief 	GPIO_ReadPin , Reads the pin of GPIOx
 * @oaram	GPIOx = GPIO Port Base Addresses
 *
 * @param	Pin_Number = GPIO Pin numbers 0-15
 *
 * @retreval GPIO_PinState_t
 */
GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx,uint16_t Pin_Number)
{
	GPIO_PinState_t bitStatus = GPIO_Pin_Reset;
	if((GPIOx->IDR & Pin_Number)!= GPIO_Pin_Reset)
	{
		bitStatus = GPIO_Pin_Set;
	}
	return bitStatus;
}


/*
 * @brief 	GPIO_LockPin , Locks the pin of GPIOx
 * @oaram	GPIOx = GPIO Port Base Addresses
 *
 * @param	Pin_Number = GPIO Pin numbers 0-15
 *
 * @retreval None
 */
void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, uint16_t Pin_Number)
{
	uint32_t tempValue = (0x1U << 16U) | Pin_Number;
	GPIOx->LCKR = tempValue;
	GPIOx->LCKR = Pin_Number;
	GPIOx->LCKR = tempValue;
	tempValue = GPIOx->LCKR;

}


/*
 * @brief 	GPIO_TogglePin , Toggles the pin of GPIOx
 * @oaram	GPIOx = GPIO Port Base Addresses
 *
 * @param	Pin_Number = GPIO Pin numbers 0-15
 *
 * @retreval None
 */
void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx, uint16_t Pin_Number)
{
	uint32_t tempODRRegister = GPIOx->ODR;
	GPIOx->BSSR = ( (tempODRRegister & Pin_Number ) << 16U ) | (~tempODRRegister & Pin_Number);
}


































