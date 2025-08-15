#include "stm32f767xx.h"

static void GPIO_LedConfig();
static void GPIO_ButtonInterruptConfig();
static void SPI_Config();
static void SPI_GPIO_Config();

SPI_HandleTypeDef_t SPI_Handle;

void EXTI0_IRQHandler()
{
	if(EXTI->PR & 0x1)
	{
		EXTI->PR |= (0x1U << 0U);
		GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_14, GPIO_Pin_Set);
	}
}

void SPI1_IRQHandler()
{
	SPI_InterrupHandler(&SPI_Handle);
}

int main(void)
{
	GPIO_LedConfig();
	GPIO_ButtonInterruptConfig();
	SPI_GPIO_Config();
	SPI_Config();

	char msg[]= "Mehmet Demirbas";

	for(;;)
	{
		SPI_TransmitData_IT(&SPI_Handle, (uint8_t*)msg, strlen(msg));

	}
}

static void GPIO_LedConfig()
{
	RCC_GPIOB_CLK_ENABLE();
	RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef_t GPIO_LedStruct = {0};
	GPIO_LedStruct.PinNumbers = GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_14;
	GPIO_LedStruct.Mode = GPIO_MODE_OUTPUT;
	GPIO_LedStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_LedStruct.Otype = GPIO_OTYPE_PUSH_PULL;
	GPIO_LedStruct.PuPd = GPIO_PUPD_NOPULL;

	GPIO_Init(GPIOB, &GPIO_LedStruct);	//Config
	memset(&GPIO_LedStruct,0,sizeof(GPIO_LedStruct));


	GPIO_LedStruct.PinNumbers = 13;
	GPIO_LedStruct.Mode = GPIO_MODE_INPUT;
	GPIO_LedStruct.PuPd = GPIO_PUPD_PULL_DOWN;

	GPIO_Init(GPIOC, &GPIO_LedStruct);
}
static void GPIO_ButtonInterruptConfig()
{
	EXTI_InitTypeDef_t EXTI_InitStruct = {0};
	RCC_SYSCFG_CLK_ENABLE();
	EXTI_LineConfig(EXTI_PortSource_GPIOC, EXTI_LineSource_0);


	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.Mode = EXTI_MODE_INTERRUPT;
	EXTI_InitStruct.TriggerSelection = EXTI_TRIGGER_RISING_EDGE;
	EXTI_InitStruct.EXTI_LineNumber = EXTI_LineSource_0;

	EXTI_Init(&EXTI_InitStruct);
	NVIC_EnableInterrupt(EXTI0_IRQNumber);
}



static void SPI_Config()
{
	RCC_SPI1_CLK_ENABLE();


	SPI_Handle.Instance = SPI1;
	SPI_Handle.Init.Baudrate = SPI_BAUDRATE_DIV8;
	SPI_Handle.Init.BusConfig = SPI_BUS_FULLDUPLEX;
	SPI_Handle.Init.CPHA = SPI_CPHA_FIRST_EDGE;
	SPI_Handle.Init.CPOL = SPI_CPOL_LOW;
	SPI_Handle.Init.CRC_Length = SPI_CRC_LENGTH_8BIT;
	SPI_Handle.Init.FrameFormat = SPI_FRAME_FORMAT_MSB;
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI_Handle.Init.SSM_Cmd = SPI_SSM_ENABLED;

	SPI_Init(&SPI_Handle);
	NVIC_EnableInterrupt(SPI1_IRQNumber);

	SPI_Periph_Cmd(&SPI_Handle, ENABLE);



}
static void SPI_GPIO_Config()
{
	RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef_t GPIO_InitStruct = {0};

	GPIO_InitStruct.PinNumbers = GPIO_PIN_5 | GPIO_PIN_7;  //PA5= SCK PA7= MOSI
	GPIO_InitStruct.Mode = GPIO_MODE_AF;
	GPIO_InitStruct.Otype = GPIO_OTYPE_PUSH_PULL;
	GPIO_InitStruct.PuPd = GPIO_PUPD_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.AlernateFunction = GPIO_AF_5;

	GPIO_Init(GPIOA, &GPIO_InitStruct);
}



























