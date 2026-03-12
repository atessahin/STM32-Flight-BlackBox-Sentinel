#ifndef BOARD_INIT_H
#define BOARD_INIT_H
#include "stm32f4xx.h"
void gpioConfig(void);
void I2C_Config(I2C_TypeDef *I2Cx);
void boardInit(void);
void uart2Config();
void dmaInit();
void spiInit();
void watchDogInit();
void feedTheDog(void);
#endif // BOARD_INIT_H
