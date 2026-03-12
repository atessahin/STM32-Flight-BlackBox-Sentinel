#include "stm32f4xx.h"
#include "system_clock.h"

void gpioConfig();
void i2cConfig();

void boardInit()
{
	systemClockConfig();
	gpioConfig();
}

void gpioConfig()
{
	RCC->AHB1ENR |= (1<<1); // Enable GPIOB
	RCC->AHB1ENR |= (1<<0); // Enable GPIOA

	// Configure PB6 SCL and PB7 SDA for Alternate Function
	GPIOB->MODER &= ~((3<<12) | (3<<14)); // PB6, PB7 clear MODE bits
	GPIOB->MODER |=  ((2<<12) | (2<<14)); // Set to Alternate Function Mode

	// Set output type to Open-Drain
	GPIOB->OTYPER |= (1<<6) | (1<<7);

	// Configure Pull-up resistors
	GPIOB->PUPDR &= ~((3<<12) | (3<<14));
	GPIOB->PUPDR |=  (1<<12) | (1<<14); // Set to Pull-up

	// Select Alternate Function 4 AF4 for I2C1
	GPIOB->AFR[0] &= ~((0xF<<24) | (0xF<<28)); // Clear AF bits
	GPIOB->AFR[0] |=  ((4<<24) | (4<<28)); // Set AF4 for PB6/PB7

	//for i2c2
	//PB3-SDA PB10-SCL
	GPIOB->MODER &= ~((3 << 6) | (3 << 20)); // clear
	GPIOB->MODER |=  ((2 << 6) | (2 << 20)); // AF mode

	GPIOB->OTYPER |= ((1 << 3) | (1 << 10));//open drain

	//pull-up
	GPIOB->PUPDR &= ~((3 << 6) | (3 << 20));
	GPIOB->PUPDR |=  ((1 << 6) | (1 << 20));

	GPIOB->AFR[0] &= ~(0xF << 12); //clear
	GPIOB->AFR[0] |=  (9 << 12);   // pb3 AF9

	GPIOB->AFR[1] &= ~(0xF << 8);  //clear
	GPIOB->AFR[1] |=  (4 << 8);// pb10 AF4

	//USART2 TX (PA2)

	// Set PA2 mode to Alternate Function (10)
	GPIOA->MODER &= ~(3 << 4);
	GPIOA->MODER |=  (2 << 4);

	// Select AF7 (USART2) for PA2 (AFRL[11:8])
	GPIOA->AFR[0] &= ~(0xF << 8);
	GPIOA->AFR[0] |=  (0x7 << 8);

	// Set PA2 output speed to High
	GPIOA->OSPEEDR |= (3 << 4);


	//USART2 RX (PA3)

	// Set PA3 mode to Alternate Function (10)
	GPIOA->MODER &= ~(3 << 6);
	GPIOA->MODER |=  (2 << 6);

	// Select AF7 (USART2) for PA3 (AFRL[15:12])
	GPIOA->AFR[0] &= ~(0xF << 12);
	GPIOA->AFR[0] |=  (0x7 << 12);


	//LED (PA10) for debug

	// Set PA10 as General Purpose Output (01)
	GPIOA->MODER &= ~(3 << 20);
	GPIOA->MODER |=  (1 << 20);


	//for spi PB(12-13-14-15)
	//PB15-MOSI PB14-MISO PB13-SCK PB12-CS

	//PB12-CS
	GPIOB->MODER &= ~(3 << 24);
	GPIOB->MODER |=  (1 << 24); // 1 (Output)

	GPIOB->BSRR   |=  (1 << 12); // CS High start
	// PB13 (SCK)
	GPIOB->MODER &= ~(3 << 26);
	GPIOB->MODER |=  (2 << 26);

	// PB14 (MISO)
	GPIOB->MODER &= ~(3 << 28);
	GPIOB->MODER |=  (2 << 28);

	// PB15 (MOSI)
	GPIOB->MODER &= ~(3 << 30);
	GPIOB->MODER |=  (2 << 30);

	// PB13 -> AF5
	GPIOB->AFR[1] &= ~(0xF << 20);
	GPIOB->AFR[1] |=  (5   << 20);

	// PB14 -> AF5
	GPIOB->AFR[1] &= ~(0xF << 24);
	GPIOB->AFR[1] |=  (5   << 24);

	// PB15 -> AF5
	GPIOB->AFR[1] &= ~(0xF << 28);
	GPIOB->AFR[1] |=  (5   << 28);
}

void I2C_Config(I2C_TypeDef *I2Cx)
{
    if (I2Cx == I2C1) {
        RCC->APB1ENR |= (1<<21);
    } else if (I2Cx == I2C2) {
        RCC->APB1ENR |= (1<<22);
    }

    //Software Reset
    I2Cx->CR1 |= (1<<15);
    I2Cx->CR1 &= ~(1<<15);

    I2Cx->CR1 &= ~(1<<0);  //Disable Peripheral

    I2Cx->CR2 = 42;    // PCLK1 frequency
    I2Cx->CCR = 210;   // Speed
    I2Cx->TRISE = 43;  //max rise time

    I2Cx->CR1 |= (1<<10);  // Enable Acknowledge
    I2Cx->CR1 |= (1<<0);   // Enable Peripheral

    for(volatile int i=0; i<50000; i++);
}

//FOR USART2_TX STREAM_6
void uart2Config()
{
	RCC->APB1ENR|=RCC_APB1ENR_USART2EN ;

	USART2->CR1 &= ~USART_CR1_UE;
	/*USART_BRR =Fck / Baud
	 *
	 42,000,000 / 9,600 = 4375

	 Mantissa = 4375
	 Fraction = 0

	 BRR = (Mantissa << 4) | Fraction

	 Mantissa << 4 = 4375 × 16 = 70000
	 Fraction        = 0
	 --------------------------------
	 BRR             = 70000 (decimal)


	70000 (decimal) = 0x11170

	USART2->BRR = 0x1170;

	 */
    USART2->BRR  = 0x1117;

	USART2->CR1 &= ~USART_CR1_M;

	USART2->CR1 &= ~USART_CR1_PCE;

	USART2->CR1|=USART_CR1_RE;

	USART2->CR1|=USART_CR1_TE;

	USART2->CR3|=USART_CR3_DMAR;

	USART2->CR3|=USART_CR3_DMAT;

	USART2->CR1|=USART_CR1_UE;
}

/*

| Setting                          | What does it provide?                      | Why is it enabled/disabled?              |
| -------------------------------- | ------------------------------------------ | ---------------------------------------- |
| **Circular mode**                | Restarts from the beginning after transfer | For continuous data streaming            |
| **Peripheral increment disable** | Keeps the peripheral address fixed         | For fixed-address registers like DR      |
| **Memory increment enable**      | Increments memory pointer each transfer    | To write data sequentially into a buffer |

 */

void dmaInit()
{
	RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
	DMA1_Stream6->CR&= ~DMA_SxCR_EN;//unable
	while(DMA1_Stream6->CR & DMA_SxCR_EN);//unable wait
	DMA1_Stream6->CR|= DMA_SxCR_DIR_0;//perhipal->memory

	DMA1_Stream6->CR &= ~DMA_SxCR_CIRC;//circ mode
	DMA1_Stream6->CR &= ~DMA_SxCR_PINC;//pinc mode
	DMA1_Stream6->CR|=DMA_SxCR_MINC;//minc mode
	DMA1_Stream6->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);// perhipal data size and memory data size both of 8 bit
	DMA1_Stream6->CR|=(DMA_SxCR_CHSEL_2);//chanel selected 4
	DMA1_Stream6->CR|=(DMA_SxCR_PL_0);//priorty lvl 1
	DMA1_Stream6->PAR = (uint32_t)&(USART2->DR);//perhipal adres

	//FOR SPI
	DMA1_Stream4->CR&= ~DMA_SxCR_EN;//unable
	while(DMA1_Stream4->CR & DMA_SxCR_EN);//unable wait
	DMA1_Stream4->CR|= DMA_SxCR_DIR_0;//perhipal->memory

	DMA1_Stream4->CR &= ~DMA_SxCR_CIRC;//circ mode
	DMA1_Stream4->CR &= ~DMA_SxCR_PINC;//pinc mode
	DMA1_Stream4->CR|=DMA_SxCR_MINC;//minc mode
	DMA1_Stream4->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);// perhipal data size and memory data size both of 8 bit
	DMA1_Stream4->CR&= ~(7<<25);//chanel selected 0
	DMA1_Stream4->CR|=(2<<16);//priorty lvl 1
	DMA1_Stream4->CR|=(1<<4);//Transfer complete interrupt enable
	DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);//perhipal adres
	NVIC_SetPriority(DMA1_Stream4_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);

	//FOR I2C
	DMA1_Stream7->CR&= ~DMA_SxCR_EN;
	while(DMA1_Stream7->CR & DMA_SxCR_EN);//unable wait
	DMA1_Stream7->CR|= DMA_SxCR_DIR_0;//perhipal->memory
	DMA1_Stream7->CR &= ~DMA_SxCR_CIRC;//circ mode
	DMA1_Stream7->CR &= ~DMA_SxCR_PINC;//pinc mode
	DMA1_Stream7->CR|=DMA_SxCR_MINC;//minc mode
	DMA1_Stream7->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);// perhipal data size and memory data size both of 8 bit
	DMA1_Stream7->CR|=(7<<25);//chanel selected 7
	DMA1_Stream7->CR|=(2<<16);//priorty lvl 1
	DMA1_Stream7->CR|=(1<<4);//Transfer complete interrupt enable
	DMA1_Stream7->PAR = (uint32_t)&(I2C2->DR);//perhipal adres
	NVIC_SetPriority(DMA1_Stream7_IRQn, 5);
	NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

void spiInit()
{
	RCC->APB1ENR|=(1<<14);//SPI PERHIPAL ENABLE
	SPI2->CR1&= ~(1<<6);//SPI DISABLE
	SPI2->CR1|= (1<<3);//baudrate control fpclk/4
	SPI2->CR1|= (1<<2);//master selectin=master config
	SPI2->CR1&= ~(1<<7);//frame format MSB transmitted first
	SPI2->CR1&= ~(1<<1);//cpol=0
	SPI2->CR1&= ~(1<<0);//cpha=0
	SPI2->CR1 &= ~(1<<10);//rxonly=0 full-duplex
	SPI2->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management
	SPI2->CR2 |= (1<<1);//DMA TX BUFF ENABLE
	SPI2->CR1 |= (1 << 6);//START
}
void watchDogInit()
{
	//for debug
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
	IWDG->KR = 0xCCCC;//start wdt
	IWDG->KR = 0x5555;//enable acces
	IWDG->PR = 0x03;// Prescaler 							T_{timeout} = 200 * 32/32000khz		calculation
	IWDG->RLR=200;// Reload val
	while (IWDG->SR != 0);//wait the changes


}
void feedTheDog(void) {
	IWDG->KR = 0xAAAA;//feed this shit
}

