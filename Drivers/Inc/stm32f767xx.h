#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

#include <stdint.h>
#include <string.h>

/*
 * MPU Defines
 */
#define NVIC_ISER0							((uint32_t*)(0xE000E100))

#define SET_BIT(REG,BIT)					((REG) |=  (BIT))
#define RESET_BIT(REG,BIT)					((REG) &= ~(BIT))
#define READ_BIT(REG,BIT)					((REG) &   (BIT))
#define UNUSED(x)							(void)x

typedef enum
{
	DISABLE = 0x0U,
	ENABLE = !DISABLE
}FunctionalState_t;

/*
 * IRQ Numbers of MCU  ==> Vector Table
 */
typedef enum
{
	EXTI0_IRQNumber = 6,
	EXTI1_IRQNumber = 7,
	EXTI2_IRQNumber = 8,
	EXTI3_IRQNumber = 9,
	EXTI4_IRQNumber = 10

}IRQNumber_TypeDef_t;


/*
 *  Memory Base Address
 */

#define FLASH_BASE_ADDR						(0x08000000UL) 							// Flash Base Address
#define SRAM1_BASE_ADDR						(0x20020000UL) 							// SRAM1 Base Address
#define SRAM2_BASE_ADDR						(0x2007C000UL) 							// SRAM2 Base Address

/*
 *  Peripheral Base Addresses
 */

#define PERIPH_BASE_ADDR					(0x40000000UL)  						// Base addresses for all peripherals

#define APB1_BASE_ADDR						(PERIPH_BASE_ADDR) 						// APB1 Bus Domain Base Address
#define APB2_BASE_ADDR						(PERIPH_BASE_ADDR + 0x10000UL) 			// APB2 Bus Domain Base Address
#define AHB1_BASE_ADDR						(PERIPH_BASE_ADDR + 0x20000UL) 			// AHB1 Bus Domain Base Address
#define AHB2_BASE_ADDR						(PERIPH_BASE_ADDR + 0x10000000UL) 		// AHB2 Bus Domain Base Address
#define AHB3_BASE_ADDR						(0xA0000000UL) 							// AHB3 Bus Domain Base Address




/*
 *  APB1 Peripherals Base Addresses
 */

#define TIM2_BASE_ADDR						(APB1_BASE_ADDR)						// TIM2 Base Address
#define TIM3_BASE_ADDR						(APB1_BASE_ADDR + 0x0400UL)				// TIM3 Base Address
#define TIM4_BASE_ADDR						(APB1_BASE_ADDR + 0x0800UL)				// TIM4 Base Address
#define TIM5_BASE_ADDR						(APB1_BASE_ADDR + 0x0C00UL)				// TIM5 Base Address
#define TIM6_BASE_ADDR						(APB1_BASE_ADDR + 0x1000UL)				// TIM6 Base Address
#define TIM7_BASE_ADDR						(APB1_BASE_ADDR + 0x1400UL)				// TIM7 Base Address
#define TIM12_BASE_ADDR						(APB1_BASE_ADDR + 0x1800UL)				// TIM12 Base Address
#define TIM13_BASE_ADDR						(APB1_BASE_ADDR + 0x1C00UL)				// TIM13 Base Address
#define TIM14_BASE_ADDR						(APB1_BASE_ADDR + 0x2000UL)				// TIM14 Base Address
#define LPTIM1_BASE_ADDR					(APB1_BASE_ADDR + 0x2400UL)				// LPTIM1 Base Address
#define RTC_BKP_BASE_ADDR					(APB1_BASE_ADDR + 0x2C00UL)				// RTC & BKP Registers Base Address
#define WWDG_BASE_ADDR						(APB1_BASE_ADDR + 0x3000UL)				// WWDG Base Address
#define CAN3_BASE_ADDR						(APB1_BASE_ADDR + 0x3400UL)				// CAN3 Base Address
#define SPI2_BASE_ADDR						(APB1_BASE_ADDR + 0x3800UL)				// SAPI2 / I2S2 Base Address
#define SPI3_BASE_ADDR						(APB1_BASE_ADDR + 0x3C00UL)				// SAPI3 / I2S3 Base Address
#define SPDIFRX_BASE_ADDR					(APB1_BASE_ADDR + 0x4000UL)				// SPDIFRX Base Address
#define USART2_BASE_ADDR					(APB1_BASE_ADDR + 0x4400UL)				// USART2 Base Address
#define USART3_BASE_ADDR					(APB1_BASE_ADDR + 0x4800UL)				// USART3 Base Address
#define UART4_BASE_ADDR						(APB1_BASE_ADDR + 0x4C00UL)				// UART4 Base Address
#define UART5_BASE_ADDR						(APB1_BASE_ADDR + 0x5000UL)				// UART5 Base Address
#define I2C1_BASE_ADDR						(APB1_BASE_ADDR + 0x5400UL)				// I2C1 Base Address
#define I2C2_BASE_ADDR						(APB1_BASE_ADDR + 0x5800UL)				// I2C2 Base Address
#define I2C3_BASE_ADDR						(APB1_BASE_ADDR + 0x5C00UL)				// I2C3 Base Address
#define I2C4_BASE_ADDR						(APB1_BASE_ADDR + 0x6000UL)				// I2C4 Base Address
#define CAN1_BASE_ADDR						(APB1_BASE_ADDR + 0x6400UL)				// CAN1 Base Address
#define CAN2_BASE_ADDR						(APB1_BASE_ADDR + 0x6800UL)				// CAN2 Base Address
#define HDMI_BASE_ADDR						(APB1_BASE_ADDR + 0x6C00UL)				// HDMI-CEC Base Address
#define PWR_BASE_ADDR						(APB1_BASE_ADDR + 0x7000UL)				// PWR Base Address
#define DAC_BASE_ADDR						(APB1_BASE_ADDR + 0x7400UL)				// DAC Base Address
#define UART7_BASE_ADDR						(APB1_BASE_ADDR + 0x7800UL)				// UART7 Base Address
#define UART8_BASE_ADDR						(APB1_BASE_ADDR + 0x7C00UL)				// UART8 Base Address





/*
 *  APB2 Peripherals Base Addresses
 */

#define TIM1_BASE_ADDR						(APB2_BASE_ADDR)						// TIM1 Base Address
#define TIM8_BASE_ADDR						(APB2_BASE_ADDR + 0x0400UL)				// TIM8 Base Address
#define USART1_BASE_ADDR					(APB2_BASE_ADDR + 0x1000UL)				// USART1 Base Address
#define USART6_BASE_ADDR					(APB2_BASE_ADDR + 0x1400UL)				// USART6 Base Address
#define SDMMC2_BASE_ADDR					(APB2_BASE_ADDR + 0x1C00UL)				// SDMMC2 Base Address
#define ADC1_2_3_BASE_ADDR					(APB2_BASE_ADDR + 0x2000UL)				// ADC1 - ADC2 - ADC3 Base Address
#define SDMMC1_BASE_ADDR					(APB2_BASE_ADDR + 0x2C00UL)				// SDMMC1 Base Address
#define SPI1_BASE_ADDR						(APB2_BASE_ADDR + 0x3000UL)				// SPI1 Base Address
#define SPI4_BASE_ADDR						(APB2_BASE_ADDR + 0x3400UL)				// SPI4 Base Address
#define SYSCFG_BASE_ADDR					(APB2_BASE_ADDR + 0x3800UL)				// SYSCFG Base Address
#define EXTI_BASE_ADDR						(APB2_BASE_ADDR + 0x3C00UL)				// EXTI Base Address
#define TIM9_BASE_ADDR						(APB2_BASE_ADDR + 0x4000UL)				// TIM9 Base Address
#define TIM10_BASE_ADDR						(APB2_BASE_ADDR + 0x4400UL)				// TIM10 Base Address
#define TIM11_BASE_ADDR						(APB2_BASE_ADDR + 0x4800UL)				// TIM11 Base Address
#define SPI5_BASE_ADDR						(APB2_BASE_ADDR + 0x5000UL)				// SPI5 Base Address
#define SPI6_BASE_ADDR						(APB2_BASE_ADDR + 0x5400UL)				// SPI6 Base Address
#define SAI1_BASE_ADDR						(APB2_BASE_ADDR + 0x5800UL)				// SAI1 Base Address
#define SAI2_BASE_ADDR						(APB2_BASE_ADDR + 0x5C00UL)				// SAI2 Base Address
#define LCD_TFT_BASE_ADDR					(APB2_BASE_ADDR + 0x6800UL)				// LCD-TFT Base Address
#define DSI_BASE_ADDR						(APB2_BASE_ADDR + 0x6C00UL)				// DSI Host Base Address
#define DFSDM1_BASE_ADDR					(APB2_BASE_ADDR + 0x7400UL)				// DFSDM1 Base Address
#define MDIOS_BASE_ADDR						(APB2_BASE_ADDR + 0x7800UL)				// MDIOS Base Address





/*
 *  AHB1 Peripherals Base Addresses
 */

#define GPIOA_BASE_ADDR						(AHB1_BASE_ADDR)						// GPIOA Base Address
#define GPIOB_BASE_ADDR						(AHB1_BASE_ADDR + 0x0400UL)				// GPIOB Base Address
#define GPIOC_BASE_ADDR						(AHB1_BASE_ADDR + 0x0800UL)				// GPIOC Base Address
#define GPIOD_BASE_ADDR						(AHB1_BASE_ADDR + 0x0C00UL)				// GPIOD Base Address
#define GPIOE_BASE_ADDR						(AHB1_BASE_ADDR + 0x1000UL)				// GPIOE Base Address
#define GPIOF_BASE_ADDR						(AHB1_BASE_ADDR + 0x1400UL)				// GPIOF Base Address
#define GPIOG_BASE_ADDR						(AHB1_BASE_ADDR + 0x1800UL)				// GPIOG Base Address
#define GPIOH_BASE_ADDR						(AHB1_BASE_ADDR + 0x1C00UL)				// GPIOH Base Address
#define GPIOI_BASE_ADDR						(AHB1_BASE_ADDR + 0x2000UL)				// GPIOI Base Address
#define GPIOJ_BASE_ADDR						(AHB1_BASE_ADDR + 0x2400UL)				// GPIOJ Base Address
#define GPIOK_BASE_ADDR						(AHB1_BASE_ADDR + 0x2800UL)				// GPIOK Base Address
#define CRC_BASE_ADDR						(AHB1_BASE_ADDR + 0x3000UL)				// CRC Base Address
#define RCC_BASE_ADDR						(AHB1_BASE_ADDR + 0x3800UL)				// RCC Base Address
#define FIR_BASE_ADDR						(AHB1_BASE_ADDR + 0x3C00UL)				// Flash interface register Base Address
#define BKPSRAM_BASE_ADDR					(AHB1_BASE_ADDR + 0x4000UL)				// BKPSRAM Base Address
#define DMA1_BASE_ADDR						(AHB1_BASE_ADDR + 0x6000UL)				// DMA1 Base Address
#define DMA2_BASE_ADDR						(AHB1_BASE_ADDR + 0x6400UL)				// DMA2 Base Address
#define ETHERNET_BASE_ADDR					(AHB1_BASE_ADDR + 0x8000UL)				// ETHERNET MAC Base Address






/*
 *  AHB2 Peripherals Base Addresses
 */

#define USB_OTG_BASE_ADDR					(AHB2_BASE_ADDR)						// USB OTG FS Base Address
#define DCMI_BASE_ADDR						(AHB2_BASE_ADDR + 0x50000UL)			// DCMI Base Address
#define JPEG_BASE_ADDR						(AHB2_BASE_ADDR + 0x51000UL)			// JPEG Base Address
#define CRYP_BASE_ADDR						(AHB2_BASE_ADDR + 0x60000UL)			// CRYP Base Address
#define HASH_BASE_ADDR						(AHB2_BASE_ADDR + 0x60400UL)			// HASH Base Address
#define RNG_BASE_ADDR						(AHB2_BASE_ADDR + 0x60800UL)			// RNG Base Address




/*
 *  AHB3 Peripherals Base Addresses
 */

#define FMC_BASE_ADDR						(AHB3_BASE_ADDR)						// FMC control register Base Address
#define QUADSPI_BASE_ADDR					(AHB3_BASE_ADDR + 0x1000UL)				// QUADSPI control register Base Address

/*
 *  Peripherla Structure Definations
 */
typedef struct
{

	volatile uint32_t MODER;														// GPIO port mode register 					Address offset:0x00
	volatile uint32_t OTYPER;														// GPIO port output type register 			Address offset: 0x04
	volatile uint32_t OSPEEDR;														// GPIO port speed register 				Address offset: 0x08
	volatile uint32_t PUPDR;														// GPIO port push-pull register 			Address offset: 0x0C
	volatile uint32_t IDR;															// GPIO port input data register 			Address offset: 0x10
	volatile uint32_t ODR;															// GPIO port output data register 			Address offset: 0x14
	volatile uint32_t BSSR;															// GPIO port bit set & reset register 		Address offset: 0x18
	volatile uint32_t LCKR;															// GPIO port configuration lock register 	Address offset: 0x1C
	volatile uint32_t AFR[2];														// GPIO port alternate function register 	Address offset: 0x20

}GPIO_TypeDef_t;

typedef struct
{
	volatile uint32_t CR;															// RCC clock control register 										Address offset: 0x00
	volatile uint32_t PLLCFGR;														// RCC PLL configuration register									Address offset: 0x04
	volatile uint32_t CFGR;															// RCC clock configuration register									Address offset: 0x08
	volatile uint32_t CIR;															// RCC clock interrupt register										Address offset: 0x0C
	volatile uint32_t AHB1RSTR;														// RCC AHB1 peripheral reset register								Address offset: 0x10
	volatile uint32_t AHB2RSTR;														// RCC AHB2 peripheral reset register								Address offset: 0x14
	volatile uint32_t AHB3RSTR;														// RCC AHB3 peripheral reset register								Address offset: 0x18
	volatile uint32_t RESERVED1;													// RESERVED
	volatile uint32_t APB1RSTR;														// RCC APB1 peripheral reset register 								Address offset: 0x20
	volatile uint32_t APB2RSTR;														// RCC APB2 peripheral reset register 								Address offset: 0x24
	volatile uint32_t RESERVED2;													// RESERVED
	volatile uint32_t RESERVED3;													// RESERVED
	volatile uint32_t AHB1ENR;														// RCC AHB1 peripheral clock register								Address offset: 0x30
	volatile uint32_t AHB2ENR;														// RCC AHB2 peripheral clock register								Address offset: 0x34
	volatile uint32_t AHB3ENR;														// RCC AHB3 peripheral clock register								Address offset: 0x38
	volatile uint32_t RESERVED4;													// RESERVED
	volatile uint32_t APB1ENR;														// RCC APB1 peripheral clock enable register						Address offset: 0x40
	volatile uint32_t APB2ENR;														// RCC APB2 peripheral clock enable register						Address offset: 0x44
	volatile uint32_t RESERVED5;													// RESERVED
	volatile uint32_t RESERVED6;													// RESERVED
	volatile uint32_t AHB1LPENR;													// RCC AHB1 peripheral clock enable in low-power mode register		Address offset: 0x50
	volatile uint32_t AHB2LPENR;													// RCC AHB2 peripheral clock enable in low-power mode register		Address offset: 0x54
	volatile uint32_t AHB3LPENR;													// RCC AHB3 peripheral clock enable in low-power mode register		Address offset: 0x58
	volatile uint32_t RESERVED7;													// RESERVED
	volatile uint32_t APB1LPENR;													// RCC APB1 peripheral clock enable in low-power mode register		Address offset: 0x60
	volatile uint32_t APB2LPENR;													// RCC APB2 peripheral clock enable in low-power mode register		Address offset: 0x64
	volatile uint32_t RESERVED8;													// RESERVED
	volatile uint32_t RESERVED9;													// RESERVED
	volatile uint32_t BDCR;															// RCC backup domain control register								Address offset: 0x70
	volatile uint32_t CSR;															// RCC clock control & status register								Address offset: 0x74
	volatile uint32_t RESERVED10;													// RESERVED
	volatile uint32_t RESERVED11;													// RESERVED
	volatile uint32_t SSCGR;														// RCC spread spectrum clock generation register					Address offset: 0x80
	volatile uint32_t PLLI2SCFGR;													// PLLI2S configuration register									Address offset: 0x84
	volatile uint32_t PLLSAICFGR;													// RCC PLLSAI configuration register								Address offset: 0x88
	volatile uint32_t DCKCFGR1;														// RCC dedicated clocks configuration register 1					Address offset: 0x8C
	volatile uint32_t DCKCFGR2;														// RCC dedicated clocks configuration register 2					Address offset: 0x90

}RCC_TypeDef_t;

typedef struct
{
	volatile uint32_t MEMRMP;														// SYSCFG memory remap register										Address offset:	0x00
	volatile uint32_t PMC;															// SYSCFG peripheral mode configuration register					Address offset:	0x04
	volatile uint32_t EXTI_CR[4];													// SYSCFG external interrupt configuration register 1				Address offset:	0x08
	volatile uint32_t CBR;															// SYSCFG Class B register 											Address offset:	0x1C
	volatile uint32_t Reserved;
	volatile uint32_t CMPCR;														// SYSCFG Compensation cell control register						Address offset:	0x20

}SYSCFG_TypeDef_t;

typedef struct
{
	volatile uint32_t IMR;															// EXTI Interrupt mask register										Address offser: 0x00
	volatile uint32_t EMR;															// EXTI Event mask register											Address offser: 0x04
	volatile uint32_t RTSR;															// EXTI Rising trigger selection register							Address offser: 0x08
	volatile uint32_t FTSR;															// EXTI Falling trigger selection register							Address offser: 0x0C
	volatile uint32_t SWIER;														// EXTI Software interrupt event register							Address offser: 0x10
	volatile uint32_t PR;															// EXTI Pending register											Address offser: 0x14
}EXTI_TypeDef_t;

#define GPIOA							((GPIO_TypeDef_t*)(GPIOA_BASE_ADDR))
#define GPIOB							((GPIO_TypeDef_t*)(GPIOB_BASE_ADDR))
#define GPIOC							((GPIO_TypeDef_t*)(GPIOC_BASE_ADDR))
#define GPIOD							((GPIO_TypeDef_t*)(GPIOD_BASE_ADDR))
#define GPIOE							((GPIO_TypeDef_t*)(GPIOE_BASE_ADDR))
#define GPIOF							((GPIO_TypeDef_t*)(GPIOF_BASE_ADDR))
#define GPIOG							((GPIO_TypeDef_t*)(GPIOG_BASE_ADDR))
#define GPIOH							((GPIO_TypeDef_t*)(GPIOH_BASE_ADDR))
#define GPIOI							((GPIO_TypeDef_t*)(GPIOI_BASE_ADDR))
#define GPIOJ							((GPIO_TypeDef_t*)(GPIOJ_BASE_ADDR))
#define GPIOK							((GPIO_TypeDef_t*)(GPIOK_BASE_ADDR))

#define RCC								((RCC_TypeDef_t*) (RCC_BASE_ADDR)  )

#define SYSCFG							((SYSCFG_TypeDef_t*)(SYSCFG_BASE_ADDR))
#define EXTI							((EXTI_TypeDef_t*)(EXTI_BASE_ADDR) )

/*
 * Bit Definations
 */
#define RCC_AHB1ENR_GPIOAEN_Pos			(0U)										// RCC AHB1ENR register GPIOAEN bit Position
#define RCC_AHB1ENR_GPIOAEN_Msk			(0x1U << RCC_AHB1ENR_GPIOAEN_Pos)			// RCC AHB1ENR register GPIOAEN bit Mask
#define RCC_AHB1ENR_GPIOAEN				(RCC_AHB1ENR_GPIOAEN_Msk)					// RCC AHB1ENR register GPIOAEN Macro

#define RCC_AHB1ENR_GPIOBEN_Pos			(1U)										// RCC AHB1ENR register GPIOBEN bit Position
#define RCC_AHB1ENR_GPIOBEN_Msk			(0x1U << RCC_AHB1ENR_GPIOBEN_Pos)			// RCC AHB1ENR register GPIOBEN bit Mask
#define RCC_AHB1ENR_GPIOBEN				(RCC_AHB1ENR_GPIOBEN_Msk)					// RCC AHB1ENR register GPIOBEN Macro

#define RCC_AHB1ENR_GPIOCEN_Pos			(2U)										// RCC AHB1ENR register GPIOCEN bit Position
#define RCC_AHB1ENR_GPIOCEN_Msk			(0x1U << RCC_AHB1ENR_GPIOCEN_Pos)			// RCC AHB1ENR register GPIOCEN bit Mask
#define RCC_AHB1ENR_GPIOCEN				(RCC_AHB1ENR_GPIOCEN_Msk)					// RCC AHB1ENR register GPIOCEN Macro

#define RCC_AHB1ENR_GPIODEN_Pos			(3U)										// RCC AHB1ENR register GPIODEN bit Position
#define RCC_AHB1ENR_GPIODEN_Msk			(0x1U << RCC_AHB1ENR_GPIODEN_Pos)			// RCC AHB1ENR register GPIODEN bit Mask
#define RCC_AHB1ENR_GPIODEN				(RCC_AHB1ENR_GPIODEN_Msk)					// RCC AHB1ENR register GPIODEN Macro

#define RCC_AHB1ENR_GPIOEEN_Pos			(4U)										// RCC AHB1ENR register GPIOEEN bit Position
#define RCC_AHB1ENR_GPIOEEN_Msk			(0x1U << RCC_AHB1ENR_GPIOEEN_Pos)			// RCC AHB1ENR register GPIOEEN bit Mask
#define RCC_AHB1ENR_GPIOEEN				(RCC_AHB1ENR_GPIOEEN_Msk)					// RCC AHB1ENR register GPIOEEN Macro

#define RCC_AHB1ENR_GPIOFEN_Pos			(5U)										// RCC AHB1ENR register GPIOFEN bit Position
#define RCC_AHB1ENR_GPIOFEN_Msk			(0x1U << RCC_AHB1ENR_GPIOFEN_Pos)			// RCC AHB1ENR register GPIOFEN bit Mask
#define RCC_AHB1ENR_GPIOFEN				(RCC_AHB1ENR_GPIOFEN_Msk)					// RCC AHB1ENR register GPIOFEN Macro

#define RCC_AHB1ENR_GPIOGEN_Pos			(6U)										// RCC AHB1ENR register GPIOGEN bit Position
#define RCC_AHB1ENR_GPIOGEN_Msk			(0x1U << RCC_AHB1ENR_GPIOGEN_Pos)			// RCC AHB1ENR register GPIOGEN bit Mask
#define RCC_AHB1ENR_GPIOGEN				(RCC_AHB1ENR_GPIOGEN_Msk)					// RCC AHB1ENR register GPIOGEN Macro

#define RCC_AHB1ENR_GPIOHEN_Pos			(7U)										// RCC AHB1ENR register GPIOHEN bit Position
#define RCC_AHB1ENR_GPIOHEN_Msk			(0x1U << RCC_AHB1ENR_GPIOHEN_Pos)			// RCC AHB1ENR register GPIOHEN bit Mask
#define RCC_AHB1ENR_GPIOHEN				(RCC_AHB1ENR_GPIOHEN_Msk)					// RCC AHB1ENR register GPIOHEN Macro

#define RCC_AHB1ENR_GPIOIEN_Pos			(8U)										// RCC AHB1ENR register GPIOIEN bit Position
#define RCC_AHB1ENR_GPIOIEN_Msk			(0x1U << RCC_AHB1ENR_GPIOIEN_Pos)			// RCC AHB1ENR register GPIOIEN bit Mask
#define RCC_AHB1ENR_GPIOIEN				(RCC_AHB1ENR_GPIOIEN_Msk)					// RCC AHB1ENR register GPIOIEN Macro

#define RCC_AHB1ENR_GPIOJEN_Pos			(9U)										// RCC AHB1ENR register GPIOJEN bit Position
#define RCC_AHB1ENR_GPIOJEN_Msk			(0x1U << RCC_AHB1ENR_GPIOJEN_Pos)			// RCC AHB1ENR register GPIOJEN bit Mask
#define RCC_AHB1ENR_GPIOJEN				(RCC_AHB1ENR_GPIOJEN_Msk)					// RCC AHB1ENR register GPIOJEN Macro

#define RCC_AHB1ENR_GPIOKEN_Pos			(10U)										// RCC AHB1ENR register GPIOKEN bit Position
#define RCC_AHB1ENR_GPIOKEN_Msk			(0x1U << RCC_AHB1ENR_GPIOKEN_Pos)			// RCC AHB1ENR register GPIOKEN bit Mask
#define RCC_AHB1ENR_GPIOKEN				(RCC_AHB1ENR_GPIOKEN_Msk)					// RCC AHB1ENR register GPIOKEN Macro

#define RCC_APB1ENR_SYSCFG_Pos			(14U)										// RCC APB2ENR register SYSCFG bit position
#define RCC_APB1ENR_SYSCFG_Msk			(0x1U << RCC_APB1ENR_SYSCFG_Pos)			// RCC APB2ENR register SYSCFG bit mask
#define RCC_APB1ENR_SYSCFG				(RCC_APB1ENR_SYSCFG_Msk)					// RCC APB2ENR register SYSCFG macro





























#include "RCC.h"
#include "GPIO.h"
#include "EXTI.h"

#endif /* INC_STM32F767XX_H_ */
