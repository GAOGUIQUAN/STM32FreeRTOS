#ifndef __BH1750_H
#define __BH1750_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/* BH1750 I2C地址（ADDR悬空） */
#define BH1750_ADDR         (0x23 << 1)  // 0x46 (7位地址左移1位)

/* BH1750操作模式 */
typedef enum {
    BH1750_CONTINUOUS_HIGH_RES_MODE  = 0x10,  // 连续高分辨率模式
    BH1750_CONTINUOUS_HIGH_RES_MODE2 = 0x11,  // 连续高分辨率模式2
    BH1750_CONTINUOUS_LOW_RES_MODE   = 0x13,  // 连续低分辨率模式
    BH1750_ONE_TIME_HIGH_RES_MODE   = 0x20,   // 单次高分辨率模式
    BH1750_ONE_TIME_HIGH_RES_MODE2  = 0x21,   // 单次高分辨率模式2
    BH1750_ONE_TIME_LOW_RES_MODE    = 0x23    // 单次低分辨率模式
} BH1750_Mode;

/* 函数声明 */
void BH1750_Init(void);
void BH1750_StartMeasurement(BH1750_Mode mode);
float BH1750_ReadLightIntensity(void);

#endif

