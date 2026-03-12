#include <stdint.h>

void dmaUsart2Transmit(uint8_t *dataBuffer, uint16_t size);

void dmaSpiTransmit(uint8_t *dataBuffer, uint16_t size);
void spiTxByte(uint8_t TxData);
uint8_t spiRxByte();
void SPI_SetSpeed_High(void);
void SPI_SetSpeed_Low(void);
void dmaI2CTransmit(uint8_t *dataBuffer,uint16_t size);
