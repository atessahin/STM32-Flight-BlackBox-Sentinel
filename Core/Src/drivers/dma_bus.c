#include "dma_bus.h"
#include "stm32f4xx.h"

void dmaUsart2Transmit(uint8_t *dataBuffer, uint16_t size)
{
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;//unable
    while(DMA1_Stream6->CR & DMA_SxCR_EN);//wait unable
    DMA1->HIFCR |= (1 << 21) | (1 << 20);
    DMA1_Stream6->NDTR = size;//number of data transfer
    DMA1_Stream6->M0AR = (uint32_t)dataBuffer;//which data will be read/written
    DMA1_Stream6->CR |= DMA_SxCR_EN;//enable
}

void dmaSpiTransmit(uint8_t *dataBuffer, uint16_t size)
{
	 GPIOB->BSRR|=  (1 << (12+16));
	 DMA1_Stream4->CR &= ~DMA_SxCR_EN;
	 while(DMA1_Stream4->CR & DMA_SxCR_EN);//wait unable
	 DMA1->HIFCR |= (1 << 5) | (1 << 4);
	 DMA1_Stream4->NDTR = size;//number of data transfer
	 DMA1_Stream4->M0AR = (uint32_t)dataBuffer;//which data will be read/written
	 DMA1_Stream4->CR |= DMA_SxCR_EN;//enable
}

void dmaI2CTransmit(uint8_t *dataBuffer,uint16_t size)
{
	I2C2->CR2 |= (1<<11);//i2c dma enable
	DMA1_Stream7->CR &= ~DMA_SxCR_EN;
	while(DMA1_Stream7->CR & DMA_SxCR_EN);//wait unable
	DMA1->HIFCR |= (1 << 27) | (1 << 26) | (1 << 25);;
	DMA1_Stream7->NDTR = size;
	DMA1_Stream7->M0AR = (uint32_t)dataBuffer;//which data will be read/written
	DMA1_Stream7->CR |= DMA_SxCR_EN;//enable
}

uint8_t SPI2_Transfer(uint8_t data)
{

    while (!(SPI2->SR & (1 << 1)));

    SPI2->DR = data;

    while (!(SPI2->SR & (1 << 0)));

    uint8_t received = SPI2->DR;

    while (SPI2->SR & (1 << 7));

    return received;
}
void spiTxByte(uint8_t TxData)
{
	SPI2_Transfer(TxData);
}
uint8_t spiRxByte()
{
    return SPI2_Transfer(0xFF);
}
//144khz
void SPI_SetSpeed_Low(void) {
    SPI2->CR1 &= ~(1 << 6);
    SPI2->CR1 |=  (7 << 3);
    SPI2->CR1 |=  (1 << 6);
}

//10mhz
void SPI_SetSpeed_High(void) {
    SPI2->CR1 &= ~(1 << 6);
    SPI2->CR1 &= ~(7 << 3);
    SPI2->CR1 |=  (1 << 3);
    SPI2->CR1 |=  (1 << 6);
}
