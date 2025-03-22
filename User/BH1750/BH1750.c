#include "bh1750.h"

/* ���Ŷ��� */
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
 * ���I2C��ʼ��
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
 * I2C��ʼ�ź�
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
 * I2Cֹͣ�ź�
 *----------------------------------------------------------*/
static void SoftwareI2C_Stop(void) {
    GPIO_ResetBits(BH1750_PORT, BH1750_SDA_PIN);
    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
    I2C_Delay();
    GPIO_SetBits(BH1750_PORT, BH1750_SDA_PIN);
    I2C_Delay();
}

/*----------------------------------------------------------
 * ����ACK�ź�
 *----------------------------------------------------------*/
static void SoftwareI2C_Ack(void) {
    GPIO_ResetBits(BH1750_PORT, BH1750_SDA_PIN);
    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
}

/*----------------------------------------------------------
 * ����NACK�ź�
 *----------------------------------------------------------*/
static void SoftwareI2C_NAck(void) {
    GPIO_SetBits(BH1750_PORT, BH1750_SDA_PIN);
    GPIO_SetBits(BH1750_PORT, BH1750_SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(BH1750_PORT, BH1750_SCL_PIN);
}

/*----------------------------------------------------------
 * �ȴ�ACK��Ӧ
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
 * ����һ���ֽ�
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
 * ��ȡһ���ֽ�
 *----------------------------------------------------------*/
static uint8_t SoftwareI2C_ReadByte(void) {
    uint8_t byte = 0;
    GPIO_SetBits(BH1750_PORT, BH1750_SDA_PIN);  // �ͷ�SDA��

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
 * BH1750��ʼ��
 *----------------------------------------------------------*/
void BH1750_Init(void) {
    SoftwareI2C_Init();
    BH1750_StartMeasurement(BH1750_CONTINUOUS_HIGH_RES_MODE);
}

/*----------------------------------------------------------
 * ��������
 *----------------------------------------------------------*/
void BH1750_StartMeasurement(BH1750_Mode mode) {
    SoftwareI2C_Start();
    SoftwareI2C_SendByte(BH1750_ADDR);
    SoftwareI2C_SendByte(mode);
    SoftwareI2C_Stop();
    vTaskDelay(pdMS_TO_TICKS(180));  // �ȴ�������ɣ��߷ֱ���ģʽ��Ҫ120ms+��
}

/*----------------------------------------------------------
 * ��ȡ����ǿ��
 *----------------------------------------------------------*/
float BH1750_ReadLightIntensity(void) {
    uint8_t data[2] = {0};
    float lux = 0.0;

    SoftwareI2C_Start();
    SoftwareI2C_SendByte(BH1750_ADDR | 0x01);  // ��ģʽ
    data[0] = SoftwareI2C_ReadByte();
    SoftwareI2C_Ack();
    data[1] = SoftwareI2C_ReadByte();
    SoftwareI2C_NAck();
    SoftwareI2C_Stop();

    lux = (data[0] << 8) | data[1];
    lux /= 1.2;  // ���ݷֱ��ʵ���
    return lux;
}

