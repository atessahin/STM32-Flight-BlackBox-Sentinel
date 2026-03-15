# STM32-Flight-BlackBox-Sentinel 

The most advanced and robust iteration of the **STM32-Flight** series. Building upon the foundations of my previous `STM32-Flight-Telemetry` and `STM32-Flight-Controller-FreeRTOS` projects, this repository introduces a fault-tolerant RTOS architecture designed for flight controllers.

##  Key Features & Updates

### 1. Fault-Tolerant Watchdog Architecture (IWDG + RTOS Event Groups)
* **Aviation-Grade Safety:** The system is strictly protected by a hardware Independent Watchdog (IWDG) integrated with FreeRTOS Event Groups.
* **Highest System Priority:** The dedicated `WdtTask` is assigned the absolute highest RTOS priority (Priority 6). This guarantees the guardian task is never starved by lower-level operations and always evaluates the system state precisely. 
* **Smart Task Monitoring:** A dedicated `WdtTask` constantly monitors the health of all critical system components (Sensors, Fusion, SD Card, Telemetry, Display) with a strict 200ms timeout window.
* **Avionics Fault Tolerance:** The system utilizes a specific bitmask logic (`0x0F`). If a non-critical component like the OLED Display freezes or disconnects, the watchdog intelligently ignores the missing display flag and continues to feed the dog, allowing the drone to fly. However, if a critical task (like the I2C IMU sensor or SD Card) hangs, the WDT ruthlessly resets the system to prevent a catastrophic failure.

### 2. High-Performance Display Task
* **DMA & Dedicated I2C:** The OLED display rendering has been heavily optimized. It now runs on a completely separate I2C bus and utilizes Direct Memory Access (DMA) to transmit the display buffer. This ensures the CPU is never blocked by slow I2C screen updates, leaving maximum processing power for the flight stabilization core.

### 3. BlackBox Flight Data Recorder (SPI + SD Card)
* **Continuous Logging:** Added a `BlackBoxTask` that writes high-frequency flight telemetry and attitude data directly to a MicroSD card over the SPI bus.
* **Double Buffering Mechanism:** Implemented a continuous double-buffer architecture (two 512-byte arrays). While the `f_write` and `f_sync` functions are blocking the CPU to physically write one buffer to the SD card, the incoming high-frequency flight data is seamlessly routed to the second buffer. This ensures zero data loss and prevents the RTOS queues from overflowing during slow SPI write cycles.
## Hardware Connections (Pinout)

The following table details the register-level pin configurations used in this project.

| Peripheral | Function | Pin | Mode / Additional Details |
| :--- | :--- | :--- | :--- |
| **I2C1 (Sensor)** | SCL | **PB6** | Alternate Function 4 (AF4) - Pull-Up |
| **I2C1 (Sensor)** | SDA | **PB7** | Alternate Function 4 (AF4) - Pull-Up |
| **I2C2 (OLED)** | SCL | **PB10**| Alternate Function 4 (AF4) - Pull-Up *(DMA Stream 7)* |
| **I2C2 (OLED)** | SDA | **PB3** | Alternate Function 9 (AF9) - Pull-Up |
| **USART2 (Telemetry)**| TX | **PA2** | Alternate Function 7 (AF7) *(DMA Stream 6)* |
| **USART2 (Telemetry)**| RX | **PA3** | Alternate Function 7 (AF7) |
| **SPI2 (SD Card)** | CS | **PB12**| General Purpose Output *(Starts High)* |
| **SPI2 (SD Card)** | SCK | **PB13**| Alternate Function 5 (AF5) |
| **SPI2 (SD Card)** | MISO | **PB14**| Alternate Function 5 (AF5) |
| **SPI2 (SD Card)** | MOSI | **PB15**| Alternate Function 5 (AF5) *(DMA Stream 4)* |
| **Debug / Status** | LED | **PA10**| General Purpose Output |

## Credits & Acknowledgments
A massive shoutout to [@MatveyMelnikov](https://github.com/MatveyMelnikov) for the amazing foundation of the `SDCardDriver` and FatFs integration. I heavily adapted and ported their original work, stripping away the HAL layer and converting it into a pure **Register-Level (Bare-Metal)** implementation specifically optimized for this STM32F4 RTOS environment. I successfully achieved the desired performance, and I highly respect the original architecture they provided.

## Feedback & Contributions
I am continuously learning and striving to improve my embedded systems and RTOS architecture skills. If you notice any bugs, have suggestions for better RTOS queue management, or just want to share some advice on flight controller development, **please feel free to open an Issue or** Every piece of feedback is highly appreciated.
