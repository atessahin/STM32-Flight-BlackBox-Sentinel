#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint32_t syncWord;  // 0xFFFFFFFF (if data packet breaks find the star)
    uint32_t timestamp; // xTaskGetTickCount()
    float pitch;
    float roll;
    float altitude;
    int16_t accX, accY, accZ; // for vibration analysis
    int16_t gccX, gccY, gccZ; // for vibration analysis
    uint8_t systemState; // 0:Init, 1:Fly, 2:Error
    uint8_t terminator;  // 0xAA (packet final)
} Blackbox_Data_t;
