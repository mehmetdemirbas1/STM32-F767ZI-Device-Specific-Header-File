#include "SPI.h"

/*
 * @brief 	SPI_Init , Configures the SPI parameters
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @retreval None
 */

void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle)
{
	uint32_t tempValue = 0;
	tempValue = SPI_Handle->Instance->SPI_CR1;

	tempValue |= (SPI_Handle->Init.Baudrate) | (SPI_Handle->Init.CPHA) | (SPI_Handle->Init.CPOL) | (SPI_Handle->Init.CRC_Length) | (SPI_Handle->Init.Mode) | \
			     (SPI_Handle->Init.BusConfig) | (SPI_Handle->Init.SSM_Cmd) | (SPI_Handle->Init.FrameFormat);
	SPI_Handle->Instance->SPI_CR1 = tempValue;
}

/*
 * @brief 	SPI_Periph_Cmd , Enable or disable SPI Peripherals
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @oaram	SPI_State = User selected enum ENABLE OR DISABLED
 *
 * @retreval None
 */
void SPI_Periph_Cmd(SPI_HandleTypeDef_t *SPI_Handle, FunctionalState_t SPI_State)
{
	if(ENABLE == SPI_State)
	{
		SPI_Handle->Instance->SPI_CR1 |=  (0x1U << SPI_CR1_SPE );
	}
	else
	{
		SPI_Handle->Instance->SPI_CR1 &= ~(0x1U << SPI_CR1_SPE );
	}
}
