#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f767xx.h"

/*
 * @def_group GPIO_Pins
 */

#define GPIO_PIN_0					(uint16_t)(0x0001)
#define GPIO_PIN_1					(uint16_t)(0x0002)
#define GPIO_PIN_2					(uint16_t)(0x0004)
#define GPIO_PIN_3					(uint16_t)(0x0008)
#define GPIO_PIN_4					(uint16_t)(0x0010)
#define GPIO_PIN_5					(uint16_t)(0x0020)
#define GPIO_PIN_6					(uint16_t)(0x0040)
#define GPIO_PIN_7					(uint16_t)(0x0080)
#define GPIO_PIN_8					(uint16_t)(0x0100)
#define GPIO_PIN_9					(uint16_t)(0x0200)
#define GPIO_PIN_10					(uint16_t)(0x0400)
#define GPIO_PIN_11					(uint16_t)(0x0800)
#define GPIO_PIN_12 				(uint16_t)(0x1000)
#define GPIO_PIN_13					(uint16_t)(0x2000)
#define GPIO_PIN_14					(uint16_t)(0x4000)
#define GPIO_PIN_15					(uint16_t)(0x8000)
#define GPIO_PIN_ALL				(uint16_t)(0xFFFF)

/*
 * @def_group GPIO_Modes
 */
#define GPIO_MODE_INPUT							(0x0U)
#define GPIO_MODE_OUTPUT						(0x1U)
#define GPIO_MODE_AF							(0x2U)
#define GPIO_MODE_ANALOG						(0x3U)

/*
 * @def_group GPIO_Otype_Modes
 */
#define GPIO_OTYPE_PUSH_PULL					(0x0U)
#define GPIO_OTYPE_OPEN_DRAIN					(0x1U)

/*
 * @def_group GPIO_Pupd_Modes
 */
#define GPIO_PUPD_NOPULL						(0x0U)
#define GPIO_PUPD_PULL_UP						(0x1U)
#define GPIO_PUPD_PULL_DOWN						(0x2U)

/*
 * @def_group GPIO_Speed_Modes
 */
#define GPIO_SPEED_LOW							(0x0U)
#define GPIO_SPEED_MEDIUM						(0x1U)
#define GPIO_SPEED_HIGH							(0x2U)
#define GPIO_SPEED_VERY_HIGH					(0x3U)

/*
 * @def_group GPIO_AF_Modes
 */



typedef enum
{
	GPIO_Pin_Reset = 0x0U,
	GPIO_Pin_Set   = !GPIO_Pin_Reset

}GPIO_PinState_t;

typedef struct
{
	uint32_t PinNumbers;													// GPIO Pin numbers @def_group GPIO_Pins
	uint32_t Mode;															// GPIO Pin numbers @def_group GPIO_Modes
	uint32_t Otype;															// GPIO Pin numbers @def_group GPIO_Otype_Modes
	uint32_t PuPd;															// GPIO Pin numbers @def_group GPIO_Pupd_Modes
	uint32_t Speed;															// GPIO Pin numbers @def_group GPIO_Speed_Modes
	uint32_t AlernateFunction;												// GPIO Pin numbers @def_group GPIO_AF_Modes


}GPIO_InitTypeDef_t;

void GPIO_Init(GPIO_TypeDef_t *GPIOx, GPIO_InitTypeDef_t *GPIO_ConfigStruct);
void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, uint16_t Pin_Number, GPIO_PinState_t PinState);
GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx,uint16_t Pin_Number);
void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, uint16_t Pin_Number);
void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx, uint16_t Pin_Number);










#endif /* INC_GPIO_H_ */
