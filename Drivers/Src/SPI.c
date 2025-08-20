#include "SPI.h"




/*
 * @brief 	SPI_CloseISR_Tx , Disables the interrupt for Tranmission
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @retreval None
 */
static void SPI_CloseISR_Tx(SPI_HandleTypeDef_t *SPI_Handle)
{
	SPI_Handle->Instance->SPI_CR2 &= ~(0x1U << SPI_CR2_TXEIE);
	SPI_Handle->TxDataSize = 0;
	SPI_Handle->pTxDataAddr = NULL;
	SPI_Handle->busStateTx = SPI_BUS_FREE;

}








/*
 * @brief 	SPI_CloseISR_Rx , Disables the interrupt for Recepiton
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @retreval None
 */
static void SPI_CloseISR_Rx(SPI_HandleTypeDef_t *SPI_Handle)
{
	SPI_Handle->Instance->SPI_CR2 &= ~(0x1U << SPI_CR2_RXNEIE);
	SPI_Handle->RxDataSize = 0;
	SPI_Handle->pRxDataAddr = NULL;
	SPI_Handle->busStateRx =SPI_BUS_FREE;
}








/*
 * @brief 	SPI_TransmitHelper_16Bits , Stores the user data into the DR register 16 bits
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @retreval None
 */
static void SPI_TransmitHelper_16Bits( SPI_HandleTypeDef_t *SPI_Handle)
{
	SPI_Handle->Instance->SPI_DR = *((uint16_t*)SPI_Handle->pTxDataAddr);
	SPI_Handle->pTxDataAddr += sizeof(uint16_t);
	SPI_Handle->TxDataSize -= 2;
	if(SPI_Handle->TxDataSize == 0)
	{
		SPI_CloseISR_Tx(SPI_Handle);
	}
}








/*
 * @brief 	SPI_TransmitHelper_8Bits , Stores the user data into the DR register 8 bits
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @retreval None
 */
static void SPI_TransmitHelper_8Bits( SPI_HandleTypeDef_t *SPI_Handle)
{
	SPI_Handle->Instance->SPI_DR = *((uint8_t*)SPI_Handle->pTxDataAddr);
	SPI_Handle->pTxDataAddr += sizeof(uint8_t);
	SPI_Handle->TxDataSize --;
	if(SPI_Handle->TxDataSize == 0)
	{
		SPI_CloseISR_Tx(SPI_Handle);
	}
}









/*
 * @brief 	SPI_TransmitHelper_8Bits , Reads the data register and stores into the user variable 8 bits
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @retreval None
 */
static void SPI_ReceiveHelper_8Bits(SPI_HandleTypeDef_t *SPI_Handle)
{
	*((uint8_t*)SPI_Handle->pRxDataAddr) =*((volatile uint8_t*)(&SPI_Handle->Instance->SPI_DR));
	SPI_Handle->pRxDataAddr += sizeof(uint8_t);
	SPI_Handle->RxDataSize --;
	if(SPI_Handle->RxDataSize == 0)
	{
		SPI_CloseISR_Rx(SPI_Handle);
	}

}









/*
 * @brief 	SPI_TransmitHelper_8Bits , Reads the data register and stores into the user variable 16 bits
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @retreval None
 */
static void SPI_ReceiveHelper_16Bits(SPI_HandleTypeDef_t *SPI_Handle)
{
	*((uint16_t*)SPI_Handle->pRxDataAddr) = (uint16_t)SPI_Handle->Instance->SPI_DR;
	SPI_Handle->pRxDataAddr += sizeof(uint16_t);
	SPI_Handle->RxDataSize -= 2;
	if(SPI_Handle->RxDataSize == 0)
	{
		SPI_CloseISR_Rx(SPI_Handle);
	}

}








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
 * @brief 	SPI_TransmitData_IT , Transmit data to the slave Interrupt method
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @oaram	pData = Data Address
 *
 * @oaram	sizeOfData = Data Lenght in byte
 *
 * @retreval None
 */
void SPI_TransmitData_IT(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData )
{
	SPI_BusStatus_t busState = SPI_Handle->busStateTx;

	if(busState != SPI_BUS_BUSY_TX)
	{
		SPI_Handle->pTxDataAddr = (uint8_t*)pData;
		SPI_Handle->TxDataSize = (uint16_t)sizeOfData;
		SPI_Handle->busStateTx = SPI_BUS_BUSY_TX;

		if(SPI_Handle->Instance->SPI_CR1 & (0x1U << SPI_CR1_CRCL))
		{
			SPI_Handle->TxISRFunction = SPI_TransmitHelper_16Bits;
		}
		else
		{
			SPI_Handle->TxISRFunction = SPI_TransmitHelper_8Bits;
		}

		SPI_Handle->Instance->SPI_CR2 |= (0x1U << SPI_CR2_TXEIE);

	}


}





/*
 * @brief 	SPI_ReceiveData_IT , Receive data to the slave Interrupt method
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @oaram	pBuffer = Buffer Address
 *
 * @oaram	sizeOfData = Data Lenght in byte
 *
 * @retreval None
 */
void SPI_ReceiveData_IT(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData )
{
	SPI_BusStatus_t busState = SPI_Handle->busStateRx;
	if(SPI_BUS_BUSY_RX != busState)
	{
		SPI_Handle->pRxDataAddr = (uint8_t*)pBuffer;
		SPI_Handle->RxDataSize  = (uint16_t)sizeOfData;
		SPI_Handle->busStateRx  = SPI_BUS_BUSY_RX;
		if(SPI_Handle->Instance->SPI_CR1 & (0x1 << SPI_CR1_CRCL))
		{
			SPI_Handle->RxISRFunction = SPI_ReceiveHelper_16Bits;
		}
		else
		{
			SPI_Handle->RxISRFunction = SPI_ReceiveHelper_8Bits;
		}
		SPI_Handle->Instance->SPI_CR2 |= (0x1U << SPI_CR2_RXNEIE);
	}
}






/*
 * @brief 	SPI_InterrupHandler ,
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @retreval None
 */
void SPI_InterrupHandler(SPI_HandleTypeDef_t *SPI_Handle)
{
	uint8_t interruptSource = 0;
	uint8_t interruptFlag = 0;

	interruptSource = SPI_Handle->Instance->SPI_CR2 & (0x1U << SPI_CR2_TXEIE);
	interruptFlag = SPI_Handle->Instance->SPI_SR & (0x1U << SPI_SR_TXE);

	if((interruptSource != 0)&& (interruptFlag !=0))
	{
		SPI_Handle->TxISRFunction(SPI_Handle);
	}

	interruptSource = SPI_Handle->Instance->SPI_CR2 & (0x1U << SPI_CR2_RXNEIE);
	interruptFlag = SPI_Handle->Instance->SPI_SR & (0x1U << SPI_SR_RXNE);

	if((interruptSource != 0)&& (interruptFlag !=0))
	{
		SPI_Handle->RxISRFunction(SPI_Handle);
	}
}






/*
 * @brief 	SPI_ReceiveData , Receive data from the Master
 *
 * @oaram	SPI_Handle = User config structure
 *
 * @oaram	pBuffer = Buffer Address
 *
 * @oaram	sizeOfData = Data Lenght in byte
 *
 * @retreval None
 */
void SPI_ReceiveData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData )
{
	if(SPI_Handle->Init.CRC_Length == SPI_CRC_LENGTH_16BIT)
	{
		while(sizeOfData > 0)
		{
			if(SPI_GetFlagStatus(SPI_Handle, SPI_RXNE_Flag))
			{
				*((uint16_t*)pBuffer) = (uint16_t)SPI_Handle->Instance->SPI_DR;
				pBuffer+=sizeof(uint16_t);
				sizeOfData -=2;
			}
		}

	}
	else
	{
		while(sizeOfData > 0)
		{
			if(SPI_GetFlagStatus(SPI_Handle, SPI_RXNE_Flag))
			{
				*pBuffer = *((volatile uint8_t*)&SPI_Handle->Instance->SPI_DR);
				pBuffer++;
				sizeOfData--;
			}
		}

	}
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

























