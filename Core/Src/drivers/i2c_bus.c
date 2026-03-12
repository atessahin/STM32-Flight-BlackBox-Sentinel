#include "i2c_bus.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dma_bus.h"
#include "main.h"

#define I2C_TIMEOUT 50000

// RETURN: 0 = Success, 1 = Error
// RETURN: 0 = Success, 1 = Error (Timeout or NACK)
uint8_t i2cWrite(uint8_t deviceAddr, uint8_t registerAddr, uint8_t *data, uint16_t length, I2C_TypeDef *I2Cx)
{
    volatile uint32_t timeout;

    // Check Bus Busy status
    timeout = I2C_TIMEOUT;
    while(I2Cx->SR2 & (1<<1))
    {
        if(--timeout == 0) return 1; //Bus stayed busy
    }

    // Send Start Bit
    I2Cx->CR1 |= (1<<8);

    timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & (1<<0))) // Wait for SB
    {
        if(--timeout == 0) return 1; //SB timeout
    }

    // Send Slave Address
    I2Cx->DR = deviceAddr << 1;

    timeout = I2C_TIMEOUT;
    // Wait for ADDR or AF
    while( !((I2Cx->SR1 & (1<<1)) || (I2Cx->SR1 & (1<<10))) )
    {
        if(--timeout == 0) return 1; // ADDR/AF timeout
    }

    // NACK Check
    if(I2Cx->SR1 & (1<<10))
    {
         I2Cx->SR1 &= ~(1<<10); // Clear AF flag
         I2Cx->CR1 |= (1<<9);   // Send STOP
         return 1;              // ERROR: NACK detected
    }

    volatile uint32_t temp = I2Cx->SR1;
    temp = I2Cx->SR2; // Clear ADDR flag by reading SR1 then SR2

    //Send Register Address
    I2Cx->DR = registerAddr;

    timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & (1<<7))) // Wait for TXE
    {
        if(--timeout == 0) return 1; // TXE timeout
    }

    if(I2Cx==I2C2 && first_call==0)
    {
    	dmaI2CTransmit(data,length);

    	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    	volatile uint32_t wait_timeout = I2C_TIMEOUT;
    	while(!(I2Cx->SR1 & (1<<2))) // BTF (Byte Transfer Finished)
    	 {
    	   if(--wait_timeout == 0) return 1;
    	 }
    	I2Cx->CR1 |= (1<<9);//i2c2 close


    	I2Cx->CR2 &= ~(1<<1);//i2c dma close
    	return 0;
    }
    else
    {
		// Send Data Payload
		for (uint16_t i= 0; i < length; i++)
		{
			I2Cx->DR = data[i];

			timeout = I2C_TIMEOUT;
			while(!(I2Cx->SR1 & (1<<7))) // Wait for TXE
			{
				if(--timeout == 0) return 1;
			}
		}

		// Wait for BTF (Byte Transfer Finished)
		timeout = I2C_TIMEOUT;
		while(!(I2Cx->SR1 & (1<<2)))
		{
			if(--timeout == 0) return 1;
		}

	   // Send Stop Bit
		I2Cx->CR1 |= (1<<9);

		return 0;
    }
}

// RETURN: 0 = Success, 1 = Error (Timeout or NACK)
uint8_t i2cRead(uint8_t deviceAddr, uint8_t registerAddr, uint8_t length, uint8_t *buffer, I2C_TypeDef *I2Cx)
{
    volatile uint32_t timeout;

    //Check Bus Busy status
    timeout = I2C_TIMEOUT;
    while(I2Cx->SR2 & (1<<1))
    {
        if(--timeout == 0) return 1;
    }

    // Send Start Bit
    I2Cx->CR1 |= (1<<8);

    timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & (1<<0))) // Wait for SB
    {
        if(--timeout == 0) return 1;
    }

    //Send Slave Address
    I2Cx->DR = deviceAddr << 1;

    timeout = I2C_TIMEOUT;
    while( !((I2Cx->SR1 & (1<<1)) || (I2Cx->SR1 & (1<<10))) ) // Wait for ADDR or AF
    {
        if(--timeout == 0) return 1;
    }

    if(I2Cx->SR1 & (1<<10)) // NACK Check
    {
        I2Cx->SR1 &= ~(1<<10);
        I2Cx->CR1 |= (1<<9);
        return 1;
    }

    volatile uint32_t temp = I2Cx->SR1;
    temp = I2Cx->SR2; // Clear ADDR flag

    //Send Register Address
    I2Cx->DR = registerAddr;

    timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & (1<<7))) // TXE
    {
        if(--timeout == 0) return 1;
    }

    // 5. Send Restart
    I2Cx->CR1 |= (1<<8);

    timeout = I2C_TIMEOUT;
    while(!(I2Cx->SR1 & (1<<0))) // Wait for SB
    {
        if(--timeout == 0) return 1;
    }

    //Send Slave Address
    I2Cx->DR = (deviceAddr << 1) | 1;

    timeout = I2C_TIMEOUT;
    while( !((I2Cx->SR1 & (1<<1)) || (I2Cx->SR1 & (1<<10))) ) // Wait for ADDR or AF
    {
        if(--timeout == 0) return 1;
    }

    if(I2Cx->SR1 & (1<<10)) // NACK Check
    {
        I2Cx->SR1 &= ~(1<<10);
        I2Cx->CR1 |= (1<<9);
        return 1;
    }

    temp = I2Cx->SR1;
    temp = I2Cx->SR2; // Clear ADDR flag

    //Data Read Loop
    for (int i= 0; i < length; i++)
    {
        // Handle last byte read differently (NACK + STOP)
        if(i == length - 1)
        {
            I2Cx->CR1 &= ~(1<<10); // Clear ACK bit (send NACK on last byte)
            I2Cx->CR1 |= (1<<9);   // Send STOP after receiving last byte
        }
        else
        {
            I2Cx->CR1 |= (1<<10);  // Send ACK
        }

        timeout = I2C_TIMEOUT;
        while(!(I2Cx->SR1 & (1<<6))) // Wait for RxNE (Receive Buffer Not Empty)
        {
            if(--timeout == 0) return 1; //RxNE timeout
        }
        buffer[i] = I2Cx->DR; // Read the data byte
    }

    return 0;
}

void i2c_software_reset(I2C_TypeDef *I2Cx)
{
    // Reset  1
	I2Cx->CR1 |= (1<<15);

    // Reset  0
	I2Cx->CR1 &= ~(1<<15);


	I2Cx->CR1 &= ~(1<<0);  // PE=0 (Disable)
	I2Cx->CR1 |= (1<<0);   // PE=1 (Enable)
}

