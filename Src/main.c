#include "stm32f767xx.h"

static void GPIO_LedConfig();
static void GPIO_LockControl();
static void GPIO_ButtonInterruptConfig();

void EXTI0_IRQHandler()
{
	if(EXTI->PR & 0x1)
	{
		EXTI->PR |= (0x1U << 0U);
		GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_14, GPIO_Pin_Set);
	}
}

int main(void)
{
	GPIO_LedConfig();
	//GPIO_LockPin(GPIOC, GPIO_PIN_13);
	//GPIO_LockControl();
	GPIO_ButtonInterruptConfig();


	for(;;)
	{

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

	GPIO_Init(GPIOB, &GPIO_LedStruct);
	memset(&GPIO_LedStruct,0,sizeof(GPIO_LedStruct));

/*
	GPIO_LedStruct.PinNumbers = GPIO_PIN_13;
	GPIO_LedStruct.Mode = GPIO_MODE_INPUT;
	GPIO_LedStruct.PuPd = GPIO_PUPD_PULL_DOWN;

	GPIO_Init(GPIOC, &GPIO_LedStruct);
*/
}

/*
static void GPIO_LockControl()
{
	GPIO_InitTypeDef_t GPIO_LockStruct = {0};
	GPIO_LockStruct.PinNumbers = GPIO_PIN_0;
	GPIO_LockStruct.Mode = GPIO_MODE_OUTPUT;
	GPIO_LockStruct.PuPd = GPIO_PUPD_NOPULL;

	GPIO_Init(GPIOC, &GPIO_LockStruct);


}
*/
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





























