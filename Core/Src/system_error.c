
#include "system_error.h"
#include "stm32f4xx.h"
#include "delay.h"
void System_Error_Handler(int error_code)
{
	while(1)
	{
		for (int i = 0; i < error_code; i++)
		{
			GPIOA->BSRR = (1U << 10);//LED ON
			delay_ms(1000);
			GPIOA->BSRR = (1U << (10+16));//LED OFF
			delay_ms(500);
		}
		delay_ms(2000);
	}

}
/*
2 Times, IMU (MPU6050), Sensor not found or faulty., During the 0x68 check in mpu6050_init().

3 Times, Barometer (BMP280), Pressure sensor is not responding., During the 0x60 check in bmp280_init().
 */
