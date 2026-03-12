#include "sd_driver_write.h"
#include "crc-buffer.h"

// Static functions ----------------------------------------------------------

static sd_error sd_card_transmit_data_block(
  SPI_HandleTypeDef *const hspi,
  const uint8_t *const data,
  const uint16_t data_size,
  const uint8_t start_token
)
{
  crc_buffer_16 crc_buffer = { 0 };
  uint8_t data_response = 0x0;
  //crc summary
  crc_16_result crc_result = crc_buffer_calculate_crc_16(
    &crc_buffer, (uint8_t*)data, data_size
  );

  //clear previous spi data(R/W) for spi not to give OVR(overrun fault)
  while (SPI2->SR & (1 << 0)) { volatile uint8_t garbage = SPI2->DR; }
  volatile uint32_t clear_ovr = SPI2->SR;

  //start token send with polling 0xFE(start block byte)
  sd_error status = sd_card_transmit_byte(hspi, &start_token);

  extern volatile uint8_t dma_transfer_done;
  dma_transfer_done = 0;
  //DMA enable again cause TXDMAEN unenable DMA1_Stream4_IRQHandler(main.c) if we dont enable TXDMAEN again it dont work
  SPI2->CR2 |= (1 << 1);
  dmaSpiTransmit((uint8_t*)data, data_size);

  /*dma while doing write 512 byte cpu wait here if dma say ok im done and made dma_transfer_done=1 loop finish
   * but spi dma unable(TXDMAEN) and cables(jumpers) broken or different fault like that wait 100ms and return SD_TIMEOUT
  */
  uint32_t start_time = millis();
  while (!dma_transfer_done) {
      if ((millis() - start_time) > 100) return SD_TIMEOUT;
  }

  // the SD card sends dummy data back while we are transmitting
  // we must read and throw away this garbage data to empty the buffer
  while (SPI2->SR & (1 << 0)) {
      volatile uint8_t garbage = SPI2->DR;
  }
  //if the buffer was full the SPI hardware locks itself with an OVR error
  //by hardware rules, reading DR  and then SR  clears this lock
  volatile uint32_t sr_oku = SPI2->SR;
  (void)sr_oku;

  //dma finishes its job fast but the SPI hardware is still sending the last bit on the wire
  //we must wait here until the Busy (BSY) flag becomes 0
  while (SPI2->SR & (1 << 7));

  /*when 512 byte data send finish we send crc we calculate top of the function
   *sd_card comparing crc and data if all the thing OK it send to us 0x05 approval code
  */
  status |= sd_card_transmit_bytes(hspi, (uint8_t*)&crc_result, sizeof(crc_16_result));
  status |= sd_card_receive_byte(hspi, &data_response);

  /*when sd_card accepted 0x05 it has been moved 512 byte to the SD card RAM.
   * to make it permanent it physically writes (programs) the data into its internal flash memory (NAND)
   * this process takes a few milliseconds during this time the card continuously sends 0x00 on the line (meaning i am busy)
   * */
  if ((data_response & 0x0F) == SD_DATA_RESPONSE_ACCEPTED)
  {
      uint8_t busy = 0x00;
      uint32_t t_out = millis();
      do {
          sd_card_receive_byte(hspi, &busy);
          if ((millis() - t_out) > 250) return SD_TIMEOUT;
      } while (busy == 0x00);
  }

  switch (data_response & 0xf)
  {
    case SD_DATA_RESPONSE_CRC_ERROR:
      status = SD_CRC_ERROR;
      break;
    case SD_DATA_RESPONSE_WRITE_ERROR:
      status = SD_ERROR;
      break;
    case SD_DATA_RESPONSE_ACCEPTED:
      break;
    default:
      status |= SD_TRANSMISSION_ERROR;
  }

  return status;
}


sd_error sd_card_write_data(
  SPI_HandleTypeDef *const hspi,
  const uint32_t address,
  const uint8_t *const data,
  const uint32_t block_length
)
{
  sd_command cmd_write_block = sd_card_get_cmd(24, address);
  sd_r1_response r1 = { 0 };

  CS_LOW;
  sd_error status = sd_card_transmit_bytes(hspi, (uint8_t*)&cmd_write_block, sizeof(cmd_write_block));
  status |= sd_card_receive_cmd_response(hspi, &r1, 1);

  if (r1)
    status = SD_TRANSMISSION_ERROR;
  if (status)
    goto end_write;

  status |= sd_card_transmit_data_block(
    hspi, data, block_length, 0xfe
  );

end_write:
	CS_HIGH;

	uint8_t dummy = 0xFF;
	sd_card_receive_byte(hspi, &dummy);
  return status;
}

sd_error sd_card_write_multiple_data(
  SPI_HandleTypeDef *const hspi,
  const uint32_t address,
  const uint8_t *const data,
  const uint32_t block_length,
  const uint32_t number_of_blocks
)
{
  sd_command cmd_write_multiple_block = sd_card_get_cmd(25, address);
  sd_r1_response r1 = { 0 };
  uint8_t stop_token = 0xfd;
  uint8_t busy_signal = 0;

  CS_LOW;
  sd_error status = sd_card_transmit_bytes(hspi, (uint8_t*)&cmd_write_multiple_block, sizeof(cmd_write_multiple_block));
  status |= sd_card_receive_cmd_response(hspi, &r1, 1);

  if (r1)
    status = SD_TRANSMISSION_ERROR;
  if (status)
    goto end_write;

  for (uint32_t i = 0; i < number_of_blocks; i++)
  {
    status |= sd_card_transmit_data_block(
      hspi, data + (i * block_length), block_length, 0xfc
    );
  }

  if (status)
    goto end_write;

  status |= sd_card_transmit_byte(hspi, &stop_token);
  status |= sd_card_wait_response(hspi, &busy_signal, 0xff);
  status |= sd_card_wait_response(hspi, &busy_signal, 0x0);

end_write:
	CS_HIGH;


	uint8_t dummy = 0xFF;
	sd_card_receive_byte(hspi, &dummy);
  return status;
}
