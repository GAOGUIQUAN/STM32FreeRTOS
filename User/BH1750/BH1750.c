#include "bh1750.h"

/* 引脚定义 */
#define BH1750_SCL_PIN     GPIO_Pin_6   // PB6
#define BH1750_SDA_PIN     GPIO_Pin_7   // PB7
#define BH1750_PORT        GPIOB


#define I2C_Delay()        taskYIELD()


static void SoftwareI2C_Init(void);
static void SoftwareI2C_Start(void);
static void SoftwareI2C_Stop(void);
static void SoftwareI2C_Ack(void);
static void SoftwareI2C_NAck(void);
static uint8_t SoftwareI2C_WaitAck(void);
static void SoftwareI2C_SendByte(uint8_t byte);
static uint8_t SoftwareI2C_ReadByte(void);

/*----------------------------------------------------------
 * 软件I2C初始化
 *----------------------------------------------------------*/
static void SoftwareI2C_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = BH1750_SCL_PIN | BH1750_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BH1750_PORT, &GPIO_InitStruct);

    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN | BH1750_SDA_PIN);
}

/*----------------------------------------------------------
 * I2C起始信号
 *----------------------------------------------------------*/
static void SoftwareI2C_Start(void) {
    GPIO_SetBits(BH1750_PORT, BH1750_SDA_PIN);
    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(BH1750_PORT, BH1750_SDA_PIN);
    I2C_Delay();
    GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
}

/*----------------------------------------------------------
 * I2C停止信号
 *----------------------------------------------------------*/
static void SoftwareI2C_Stop(void) {
    GPIO_ResetBits(BH1750_PORT, BH1750_SDA_PIN);
    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
    I2C_Delay();
    GPIO_SetBits(BH1750_PORT, BH1750_SDA_PIN);
    I2C_Delay();
}

/*----------------------------------------------------------
 * 发送ACK信号
 *----------------------------------------------------------*/
static void SoftwareI2C_Ack(void) {
    GPIO_ResetBits(BH1750_PORT, BH1750_SDA_PIN);
    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
}

/*----------------------------------------------------------
 * 发送NACK信号
 *----------------------------------------------------------*/
static void SoftwareI2C_NAck(void) {
    GPIO_SetBits(BH1750_PORT, BH1750_SDA_PIN);
    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
}

/*----------------------------------------------------------
 * 等待ACK响应
 *----------------------------------------------------------*/
static uint8_t SoftwareI2C_WaitAck(void) {
    GPIO_SetBits(BH1750_PORT, BH1750_SDA_PIN);
    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
    I2C_Delay();
    if (GPIO_ReadInputDataBit(BH1750_PORT, BH1750_SDA_PIN)) {
        GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
        return 1;  // NACK
    }
    GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
    return 0;      // ACK
}

/*----------------------------------------------------------
 * 发送一个字节
 *----------------------------------------------------------*/
static void SoftwareI2C_SendByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        GPIO_WriteBit(BH1750_PORT, BH1750_SDA_PIN, (byte & 0x80) ? Bit_SET : Bit_RESET);
        byte <<= 1;
        GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
        I2C_Delay();
        GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
    }
    SoftwareI2C_WaitAck();
}

/*----------------------------------------------------------
 * 读取一个字节
 *----------------------------------------------------------*/
static uint8_t SoftwareI2C_ReadByte(void) {
    uint8_t byte = 0;
    GPIO_SetBits(BH1750_PORT, BH1750_SDA_PIN);  // 释放SDA线

    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
        I2C_Delay();
        if (GPIO_ReadInputDataBit(BH1750_PORT, BH1750_SDA_PIN)) {
            byte |= 0x01;
        }
        GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
    }
    return byte;
}

/*----------------------------------------------------------
 * BH1750初始化
 *----------------------------------------------------------*/
void BH1750_Init(void) {
    SoftwareI2C_Init();
    BH1750_StartMeasurement(BH1750_CONTINUOUS_HIGH_RES_MODE);
}

/*----------------------------------------------------------
 * 启动测量
 *----------------------------------------------------------*/
void BH1750_StartMeasurement(BH1750_Mode mode) {
    SoftwareI2C_Start();
    SoftwareI2C_SendByte(BH1750_ADDR);
    SoftwareI2C_SendByte(mode);
    SoftwareI2C_Stop();
    vTaskDelay(pdMS_TO_TICKS(180));  // 等待测量完成（高分辨率模式需要120ms+）
}

/*----------------------------------------------------------
 * 读取光照强度
 *----------------------------------------------------------*/
float BH1750_ReadLightIntensity(void) {
    uint8_t data[2] = {0};
    float lux = 0.0;

    SoftwareI2C_Start();
    SoftwareI2C_SendByte(BH1750_ADDR | 0x01);  // 读模式
    data[0] = SoftwareI2C_ReadByte();
    SoftwareI2C_Ack();
    data[1] = SoftwareI2C_ReadByte();
    SoftwareI2C_NAck();
    SoftwareI2C_Stop();

    lux = (data[0] << 8) | data[1];
    lux /= 1.2;  // 根据分辨率调整
    return lux;
}

