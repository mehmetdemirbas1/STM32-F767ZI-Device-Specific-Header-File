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


/*
 * @brief 	SPI_TransmitData , Transmit data to the slave
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @oaram	pData = Data Address
 *
 * @oaram	sizeOfData = Data Lenght in byte
 *
 * @retreval None
 */
void SPI_TransmitData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData )
{
	if(SPI_Handle->Init.CRC_Length == SPI_CRC_LENGTH_16BIT)
	{
		while(sizeOfData > 0)
		{
			if(SPI_GetFlagStatus(SPI_Handle, SPI_TXE_FLAG))
			{
				SPI_Handle->Instance->SPI_DR = *((uint16_t*)pData);
				pData += sizeof(uint16_t);
				sizeOfData -= 2;
			}


		}
	}
	else
	{
		while(sizeOfData > 0)
		{
			if(SPI_GetFlagStatus(SPI_Handle, SPI_TXE_FLAG))
			{
				SPI_Handle->Instance->SPI_DR = *pData;
				pData++;
				sizeOfData--;
			}

		}
	}
	while (SPI_GetFlagStatus(SPI_Handle, SPI_BUSY_FLAG));				 // wait for busy flag
}


/*
 * @brief 	SPI_GetFlagStatus , Return SPI SR register Flag status
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @oaram	SPI_Flag = Flag Name of SR Register
 *
 *
 * @retreval SPI_FlagStatus_t
 */
SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag)
{
	return (SPI_Handle->Instance->SPI_SR & SPI_Flag) ? SPI_FLAG_SET : SPI_FLAG_RESET;
}
























