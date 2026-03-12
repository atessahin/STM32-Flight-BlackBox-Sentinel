#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "event_groups.h"

#include "board_init.h"
#include "delay.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "flight_core.h"
#include "ssd1306.h"
#include "telemetry.h"
#include "dma_bus.h"
#include "ff.h"
#include "blackBoxLog.h"

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>


#define SENSOR_OK_BIT    (1 << 0)
#define FUSION_OK_BIT    (1 << 1)
#define BLACKBOX_OK_BIT  (1 << 2)
#define TELEMETRY_OK_BIT (1 << 3)
#define DISPLAY_OK_BIT   (1 << 4)

#define ALL_TASKS_OK     (SENSOR_OK_BIT | FUSION_OK_BIT | BLACKBOX_OK_BIT | TELEMETRY_OK_BIT | DISPLAY_OK_BIT)

EventGroupHandle_t flagGroup;

//global variables
FATFS fs;
FIL fil;
FRESULT fres;
UINT bw;
UINT br;
uint8_t first_call = 1;
volatile uint8_t dma_transfer_done = 0;
volatile uint8_t dma_transfer_done_2 = 0;


//Rtos Stuff
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t mpuInterruptSem;
QueueHandle_t Raw_Data_Queue;
QueueHandle_t Display_Queue;
QueueHandle_t BlackBox_Queue;


TaskHandle_t sensor;
TaskHandle_t fusion;
TaskHandle_t display;
TaskHandle_t telemetry;
TaskHandle_t blackBox;
TaskHandle_t wdt;


//structs
extern Attitude_t current_attitude;
FlyData_t flyData;


//buffers
uint8_t buffer_1[512];
uint8_t buffer_2[512];

uint8_t flyBuffer[17];



void SensorTask(void *argument);
void FusionTask(void *argument);
void DisplayTask(void *argument);
void TelemetryTask(void *argument);
void BlackBoxTask(void *argument);
void WdtTask(void *argument);


int main(void)
{

    boardInit();
    uart2Config();
    dmaInit();
    spiInit();
    delayInit();
    SEGGER_SYSVIEW_Conf();
    SEGGER_SYSVIEW_Start();
    I2C_Config(I2C1);
    I2C_Config(I2C2);
    delay_ms(50);


    ssd1306_Init();
    ssd1306_Fill(Black);

    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("INITIALIZING...", Font_7x10, White);
    ssd1306_SetCursor(10, 30);
    ssd1306_WriteString("DO NOT MOVE!", Font_7x10, White);
    ssd1306_UpdateScreen();

    delay_ms(1000);



    mpu6050_init();
    delay_ms(100);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 20);
    ssd1306_WriteString("CALIBRATING...", Font_7x10, White);
    ssd1306_UpdateScreen();

    mpu6050_calibrate_gyro();



    bmp280_init();
    bmp280_calibrate_basic_altitude();



    ssd1306_Fill(Black);
    ssd1306_SetCursor(5, 10);
    ssd1306_WriteString("SYSTEM READY!", Font_7x10, White);
    ssd1306_SetCursor(0, 30);
    ssd1306_WriteString("READY FOR TAKEOFF!", Font_7x10, White);
    ssd1306_UpdateScreen();
    delay_ms(1000);

    flagGroup = xEventGroupCreate();
    i2cMutex = xSemaphoreCreateMutex();
    Raw_Data_Queue = xQueueCreate(5, sizeof(RawData_t));
    Display_Queue  = xQueueCreate(1, sizeof(Attitude_t));
    BlackBox_Queue  = xQueueCreate(16, sizeof(Blackbox_Data_t));

    //start sd_card
    fres = f_mount(&fs, "", 1);

    //if we need format
    /*if (fres == FR_NO_FILESYSTEM)
    	{


              ssd1306_Fill(Black);
              ssd1306_SetCursor(0, 10);
              ssd1306_WriteString("FORMATTING SD...", Font_7x10, White);
              ssd1306_UpdateScreen();


              MKFS_PARM opt = {FM_ANY, 0, 0, 0, 0};


              BYTE work_buffer[512];


              fres = f_mkfs("", &opt, work_buffer, sizeof(work_buffer));

              if (fres == FR_OK) {

                  fres = f_mount(&fs, "", 1);
              } else {

              }
          }*/
    xTaskCreate(WdtTask, "WatchDog",  512, NULL, 6, &wdt);
    xTaskCreate(SensorTask, "ReadSensors",  512, NULL, 5, &sensor);
    xTaskCreate(FusionTask, "SensorFusion", 512, NULL, 4, &fusion);
    xTaskCreate(BlackBoxTask, "BlackBoxLog",  512, NULL, 3, &blackBox);
    xTaskCreate(TelemetryTask, "TelemetryData",  512, NULL, 2, &telemetry);
    xTaskCreate(DisplayTask,"OLED_Screen",  1024, NULL, 1, &display);

    first_call=0;
    watchDogInit();
    vTaskStartScheduler();

    while(1) { }
}

void SensorTask(void *argument)
{
    RawData_t localData;
    uint32_t now;
    uint8_t int_status = 0;
    int baro_divider = 0;
    float raw_altitude = 0.0f;
    static float filtered_altitude = 0.0f;
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // Task runs every 4ms (250Hz)
    TickType_t xLastWakeTime;

    xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Wait for the next cycle

        if(xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) // Lock the I2C bus
        {
            // Read MPU6050 Data Ready Status
            if(i2cRead(MPU6050_ADDR, INT_STATUS, 1, &int_status,I2C1) == 0) // Check I2C success (0)
            {
                if(int_status & 0x01)
                {
                    now = millis();
                    localData.timestamp = now;


                    mpu6050_read_accel(&localData.ax, &localData.ay, &localData.az); // Read Accel
                    mpu6050_read_gyro(&localData.gx, &localData.gy, &localData.gz); // Read Gyro


                    //localData.altitude = read_altitude(); // Read raw altitude

                    baro_divider++;

                    if(baro_divider >= 10)
                    {
                        baro_divider = 0;


                        raw_altitude = read_altitude();

                        if(filtered_altitude == 0.0f)
                        {
                            filtered_altitude = raw_altitude; // Initialize filter
                        }
                        else
                        {
                            filtered_altitude = (filtered_altitude * 0.90f) + (raw_altitude * (1.0f - 0.90f));
                        }
                    }

                    localData.altitude = filtered_altitude; // Use filtered altitude
                    xQueueSend(Raw_Data_Queue, &localData, 0); // Send data to FusionTask
                    xEventGroupSetBits(flagGroup,SENSOR_OK_BIT);
                }
            }
            else
            {
            	i2c_software_reset(I2C1);
            }
            xSemaphoreGive(i2cMutex);
        }
    }
}
void FusionTask(void *argument)
{
	Blackbox_Data_t blackbox_Data_tx;
    RawData_t localData;
    float dt;

    while(1)
    {
        // Wait for sensor data indefinitely
        if(xQueueReceive(Raw_Data_Queue, &localData, portMAX_DELAY) == pdTRUE)
        {
            dt = calculate_dt(localData.timestamp);

            update_attitude(localData.ax, localData.ay, localData.az,
                            localData.gx, localData.gy,
                            localData.altitude, dt);

            //put the data to blackboxtx_data_struct
            blackbox_Data_tx.syncWord=0xFFFFFFFF;
            blackbox_Data_tx.timestamp=xTaskGetTickCount();
            blackbox_Data_tx.pitch=current_attitude.pitch;
            blackbox_Data_tx.roll=current_attitude.roll;
            blackbox_Data_tx.altitude=current_attitude.altitude;
            blackbox_Data_tx.accX=localData.ax;
            blackbox_Data_tx.accY=localData.ay;
            blackbox_Data_tx.accZ=localData.az;
            blackbox_Data_tx.gccX=localData.gx;
            blackbox_Data_tx.gccY=localData.gy;
            blackbox_Data_tx.gccZ=localData.gz;
            blackbox_Data_tx.systemState=1;
            blackbox_Data_tx.terminator=0xAA;

            //Send blackboxtx_data_struct to BlackBoxTask
    		xQueueSend(BlackBox_Queue,&blackbox_Data_tx,portMAX_DELAY);
    		 // Send result to DisplayTask
            xQueueSend(Display_Queue, &current_attitude, portMAX_DELAY);

            xEventGroupSetBits(flagGroup,FUSION_OK_BIT);
        }
    }
}
void DisplayTask(void *argument)
{
    Attitude_t localData;

    char buf_pitch[16];
    char buf_roll[16];
    char buf_alt[16];

    int refresh_counter = 0;

    while(1)
    {
        // Wait for filtered data
        if(xQueueReceive(Display_Queue, &localData, portMAX_DELAY) == pdTRUE)
        {
            refresh_counter++;

            if(refresh_counter >= 10) // Update display slower
            {
                refresh_counter = 0;


                sprintf(buf_pitch, "P: %5.1f", localData.pitch);
                sprintf(buf_roll,  "R: %5.1f", localData.roll);
                sprintf(buf_alt,   "A: %5.1f", localData.altitude);

                // Write to OLED buffer
                ssd1306_Fill(Black);

                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString(buf_pitch, Font_7x10, White);

                ssd1306_SetCursor(0, 15);
                ssd1306_WriteString(buf_roll, Font_7x10, White);

                ssd1306_SetCursor(0, 45);
                ssd1306_WriteString(buf_alt, Font_7x10, White);

                ssd1306_UpdateScreen();


            }
            xEventGroupSetBits(flagGroup,DISPLAY_OK_BIT);
        }
    }
}
void TelemetryTask(void *argument)
{
    const uint16_t packetSize = sizeof(FlyData_t);


    while(1)
    {
        // Fill telemetry structure
        flyData.header    = 0xABCD;
        flyData.timeStamp = xTaskGetTickCount();

        flyData.altitude  = current_attitude.altitude;
        flyData.pitch     = current_attitude.pitch;
        flyData.roll      = current_attitude.roll;
        flyData.status    = 0x01;

        // Byte pointer to struct for checksum calculation
        const uint8_t *ptr = (const uint8_t *)&flyData;

        // XOR checksum
        uint8_t xor = 0;
        for (int i = 0; i < packetSize - 1; i++)
        {
            xor ^= ptr[i];
        }

        flyData.checksum = xor;

        // Transmit packet via DMA
        dmaUsart2Transmit((uint8_t*)&flyData, packetSize);
        xEventGroupSetBits(flagGroup,TELEMETRY_OK_BIT);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
void BlackBoxTask(void *argument)
{
	Blackbox_Data_t blackbox_Data_rx;

	const uint16_t packetSize = sizeof(Blackbox_Data_t);

	uint8_t *currentBuff = buffer_1;

	uint8_t *writeBuff =NULL;

	int bufferSize=512;

	int bufferIndex=0;

	fres = f_open(&fil, "UCUS.BIN", FA_WRITE | FA_CREATE_ALWAYS);


	while(1)
	{
		 if(xQueueReceive(BlackBox_Queue, &blackbox_Data_rx, portMAX_DELAY) == pdTRUE)
		 {
			 if(packetSize+bufferIndex>bufferSize)
			 {
				 writeBuff=currentBuff;
				 if(currentBuff==buffer_1)
				 {
					 currentBuff=buffer_2;
				 }
				 else
				 {
					 currentBuff=buffer_1;
				 }

				 /*write section*/
				  if (fres == FR_OK) {
					  f_write(&fil, writeBuff, bufferSize, &bw);
					  f_sync(&fil);

				  }

				  bufferIndex = 0;
			 }

			 	 //copy incoming data to the currentBuff(buffer1 or buffer2 at this time whichever in currentbuffer)
				const uint8_t *ptr = (const uint8_t *)&blackbox_Data_rx;
				for(int i=0;i<packetSize;i++)
				{
				 currentBuff[bufferIndex]=ptr[i];
				 bufferIndex++;
				}
				xEventGroupSetBits(flagGroup,BLACKBOX_OK_BIT);

		 }

	}

}
void WdtTask(void *argument)
{
	EventBits_t AllBits;
	while(1)
	{
		AllBits=xEventGroupWaitBits(flagGroup,ALL_TASKS_OK,pdTRUE,pdTRUE,pdMS_TO_TICKS(150));

		if(AllBits==0x1F)
		{
			/*succes*/
			feedTheDog();
		}
		else if(AllBits==0xF)
		{

			/*display task not much necessary*/
			feedTheDog();

		}
		else
		{
			/*WARNING RESET BLINK LED OR WHAT YOU WANT*/
		}
	}

}
void DMA1_Stream4_IRQHandler(void)
{
	if (DMA1->HISR & (1 << 5))
	    {
	        //clear flag
	        DMA1->HIFCR |= (1 << 5);

	        SPI2->CR2 &= ~(1 << 1);      // TXDMAEN = 0
	        DMA1_Stream4->CR &= ~DMA_SxCR_EN;   // DMA(EN = 0)

	        dma_transfer_done=1;
	    }
}
void DMA1_Stream7_IRQHandler(void)
{
	if (DMA1->HISR & (1 << 27))
	    {
		 	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	        DMA1->HIFCR |= (1 << 27);//transfer complete clear flag

	        vTaskNotifyGiveFromISR(display,&xHigherPriorityTaskWoken);
	        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	        DMA1_Stream7->CR &= ~DMA_SxCR_EN;// DMA(EN = 0)

	        dma_transfer_done_2=1;
	    }
}


void Error_Handler(void)
{
    __disable_irq(); // Disable all interrupts
    while(1); // Infinite loop on error
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    while(1); // Infinite loop on assertion failure
}
#endif

