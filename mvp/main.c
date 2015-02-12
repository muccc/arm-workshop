#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"

#include <stdbool.h>

#define LED4_PIN                         GPIO_PIN_12
#define LED4_GPIO_PORT                   GPIOD

int main(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;

    /* Enable GPIOD clock */
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure PD12 pin as output */
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pin = LED4_PIN;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

    HAL_GPIO_WritePin(LED4_GPIO_PORT, LED4_PIN, GPIO_PIN_SET);

    while(1);
}
