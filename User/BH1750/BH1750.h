#ifndef __BH1750_H
#define __BH1750_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/* BH1750 I2C��ַ��ADDR���գ� */
#define BH1750_ADDR         (0x23 << 1)  // 0x46 (7λ��ַ����1λ)

/* BH1750����ģʽ */
typedef enum {
    BH1750_CONTINUOUS_HIGH_RES_MODE  = 0x10,  // �����߷ֱ���ģʽ
    BH1750_CONTINUOUS_HIGH_RES_MODE2 = 0x11,  // �����߷ֱ���ģʽ2
    BH1750_CONTINUOUS_LOW_RES_MODE   = 0x13,  // �����ͷֱ���ģʽ
    BH1750_ONE_TIME_HIGH_RES_MODE   = 0x20,   // ���θ߷ֱ���ģʽ
    BH1750_ONE_TIME_HIGH_RES_MODE2  = 0x21,   // ���θ߷ֱ���ģʽ2
    BH1750_ONE_TIME_LOW_RES_MODE    = 0x23    // ���εͷֱ���ģʽ
} BH1750_Mode;

/* �������� */
void BH1750_Init(void);
void BH1750_StartMeasurement(BH1750_Mode mode);
float BH1750_ReadLightIntensity(void);

#endif

