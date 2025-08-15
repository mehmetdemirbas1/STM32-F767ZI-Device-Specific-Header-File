#ifndef INC_SPI_H_
#define INC_SPI_H_
#include "stm32f767xx.h"


typedef enum
{
	SPI_BUS_FREE = 0x0U,
	SPI_BUS_BUSY_TX =0x1U,
	SPI_BUS_BUSY_RX =0x2U

}SPI_BusStatus_t;

/*
 * @def_group SPI_BaudRate Divisions
 */
#define SPI_BAUDRATE_DIV2				((uint32_t)(0x0) )
#define SPI_BAUDRATE_DIV4				((uint32_t)(0x8) )
#define SPI_BAUDRATE_DIV8				((uint32_t)(0x10))
#define SPI_BAUDRATE_DIV16				((uint32_t)(0x18))
#define SPI_BAUDRATE_DIV32				((uint32_t)(0x20))
#define SPI_BAUDRATE_DIV64				((uint32_t)(0x28))
#define SPI_BAUDRATE_DIV128				((uint32_t)(0x30))
#define SPI_BAUDRATE_DIV256				((uint32_t)(0x38))

/*
 * @def_group SPI_MODE
 */
#define SPI_MODE_SLAVE					((uint32_t)(0x0))
#define SPI_MODE_MASTER					((uint32_t)(0x4))

/*
 * @def_group SPI_CRC Length
 */
#define SPI_CRC_LENGTH_8BIT				((uint32_t)(0x0)  )
#define SPI_CRC_LENGTH_16BIT			((uint32_t)(0x800))

/*
 * @def_group SPI_SSM
 */
#define SPI_SSM_DISABLED				((uint32_t)(0x0)  )
#define SPI_SSM_ENABLED					((uint32_t)(0x300))

/*
 * @def_group SPI_CPHA Values
 */
#define SPI_CPHA_FIRST_EDGE				((uint32_t)(0x0)  )
#define SPI_CPHA_SECOND_EDGE			((uint32_t)(0x1)  )

/*
 * @def_group SPI_CPOL Values
 */
#define SPI_CPOL_LOW					((uint32_t)(0x0)  )
#define SPI_CPOL_HIGH					((uint32_t)(0x2)  )

/*
 * @def_group SPI_Frame Format
 */
#define SPI_FRAME_FORMAT_LSB			((uint32_t)(0x0)  )
#define SPI_FRAME_FORMAT_MSB			((uint32_t)(0x80) )

/*
 * @def_group SPI_Bus Config Values
 */
#define SPI_BUS_FULLDUPLEX				((uint32_t)(0x0)  )
#define SPI_BUS_RECEIVE_ONLY			((uint32_t)(0x400))
#define SPI_BUS_HALFDUPLEX_T			((uint32_t)(0xC000))
#define SPI_BUS_HALFDUPLEX_R			((uint32_t)(0x8000))


typedef enum
{
	SPI_FLAG_RESET = 0x0U,
	SPI_FLAG_SET = !SPI_FLAG_RESET

}SPI_FlagStatus_t;


typedef struct
{
	uint32_t Mode;								// SPI Mode Definations 		@def_group SPI_MODE
	uint32_t CPHA;								// CPHA Values For SPI  		@def_group SPI_CPHA Values
	uint32_t CPOL;								// CPOL Values For SPI  		@def_group SPI_CPOL Values
	uint32_t Baudrate;							// SPI BaudRate Values 			@def_group SPI_BaudRate Divisions
	uint32_t SSM_Cmd;							// SPI SSM Formal Definations	@def_group SPI_SSM
	uint32_t CRC_Length;						// SPI CRC Lenght Definations 	@def_group SPI_CRC Length
	uint32_t BusConfig;							// Bus Config Values For SPI 	@def_group SPI_Bus Config Values
	uint32_t FrameFormat;						// SPI Frame Format Values 		@def_group SPI_Frame Format

}SPI_InitTypeDef_t;



typedef struct __SPI_HandleTypeDef_t
{

	SPI_typeDef_t *Instance;
	SPI_InitTypeDef_t Init;
	uint8_t* pTxDataAddr;
	uint16_t TxDataSize;
	uint8_t busStateTx;
	void (*TxISRFunction)(struct __SPI_HandleTypeDef_t *SPI_Handle);


}SPI_HandleTypeDef_t;









void SPI_Init(SPI_HandleTypeDef_t *SPI_Handle);
void SPI_Periph_Cmd(SPI_HandleTypeDef_t *SPI_Handle, FunctionalState_t SPI_State);
void SPI_TransmitData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData );
void SPI_ReceiveData(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData );
SPI_FlagStatus_t SPI_GetFlagStatus(SPI_HandleTypeDef_t *SPI_Handle, uint16_t SPI_Flag);
void SPI_TransmitData_IT(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pData, uint16_t sizeOfData );
void SPI_ReceiveData_IT(SPI_HandleTypeDef_t *SPI_Handle, uint8_t *pBuffer, uint16_t sizeOfData );
void SPI_InterrupHandler(SPI_HandleTypeDef_t *SPI_Handle);

#endif /* INC_SPI_H_ */
