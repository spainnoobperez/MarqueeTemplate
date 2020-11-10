#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"

int main() {
    GPIO_InitTypeDef *initstu;
    int tot = 0;
    initstu = (GPIO_InitTypeDef *)malloc(sizeof(GPIO_InitTypeDef));
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    initstu->GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    initstu->GPIO_Mode = GPIO_Mode_OUT;
    initstu->GPIO_OType = GPIO_OType_PP;
    initstu->GPIO_Speed = GPIO_Speed_100MHz;
    initstu->GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, initstu);
    uart_init(115200);
    delay_init(84);
    while(1) {
        tot++;
        printf("Round %d, Enciende el puto LED!\n", tot);
        GPIO_SetBits(GPIOF, GPIO_Pin_9 | GPIO_Pin_10);
        delay_ms(500);
        printf("Apaga el puto LED!\n");
        GPIO_ResetBits(GPIOF, GPIO_Pin_9 | GPIO_Pin_10);
        delay_ms(500);
    }
    return 0;
}
