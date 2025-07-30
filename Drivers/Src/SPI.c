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

