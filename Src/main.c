#include "stm32f767xx.h"

static void GPIO_LedConfig();
static void GPIO_LockControl();

int main(void)
{
	GPIO_LedConfig();
	GPIO_LockPin(GPIOC, GPIO_PIN_13);
	GPIO_LockControl();

	for(;;)
	{

		if(GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_Pin_Set)
		{
			GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_14, GPIO_Pin_Set);
		}
		else
		{
			GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7|GPIO_PIN_14, GPIO_Pin_Reset);
		}
	}
}

static void GPIO_LedConfig()
{
	RCC_GPIOB_CLK_ENABLE(); // GPIOB clock is active
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

static void GPIO_LockControl()
{
	GPIO_InitTypeDef_t GPIO_LockStruct = {0};
	GPIO_LockStruct.PinNumbers = 13;
	GPIO_LockStruct.Mode = GPIO_MODE_OUTPUT;
	GPIO_LockStruct.PuPd = GPIO_PUPD_NOPULL;

	GPIO_Init(GPIOC, &GPIO_LockStruct);


}





























