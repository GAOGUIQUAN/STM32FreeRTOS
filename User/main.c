#include "stm32f10x.h"
#include "OLED.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "BSP/UART1/bsp_uart1.h"
#include "dht11.h"
#include "bh1750.h"

// ����ȫ�����ݽṹ
DHT11_HandleTypeDef hdht11;
QueueHandle_t xSensorQueue;

// ���崫�������ݽṹ
typedef struct {
    uint8_t temp;
    uint8_t humi;
    float lux;
} SensorData_t;

// ��ʾ���񣨼��ɴ��ڷ��ͣ�
void OLED_Task(void *arg) {
    SensorData_t sensor_data;
    char str_temp[16], str_humi[16], str_lux[16];
    static uint8_t last_temp = 0, last_humi = 0;  // ������һ�ε���ʪ������

    // ��ʼ����ʾ
    OLED_ShowString(1, 1, "Temp:   C");
    OLED_ShowString(2, 1, "Humi:   %");
    OLED_ShowString(3, 1, "Lux:    lx");
    AT_Send_Command("System Start");

    while (1) {
        // �Ӷ��л�ȡ����
        if (xQueueReceive(xSensorQueue, &sensor_data, pdMS_TO_TICKS(100)) == pdPASS) {
            /* OLED��ʾ���� */
            if (sensor_data.temp != 0) {  // �����ʪ�������Ƿ�Ϊ0
                last_temp = sensor_data.temp;
                last_humi = sensor_data.humi;
            }
            sprintf(str_temp, "%2d", last_temp);
            OLED_ShowString(1, 6, str_temp);
            sprintf(str_humi, "%2d", last_humi);
            OLED_ShowString(2, 6, str_humi);
            sprintf(str_lux, "%.2f", sensor_data.lux);
            OLED_ShowString(3, 6, str_lux);

            /* ���ڷ��ͣ�����ԭ�к������÷�ʽ�� */
            char buffer[64];
            sprintf(buffer, "Temp:%dC Humi:%d%% Lux:%.2flx", last_temp, last_humi, sensor_data.lux);
            AT_Send_Command(buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(500));  // 500msˢ��һ��
    }
}

// DHT11�ɼ�����
void DHT11_Task(void *arg) {
    SensorData_t sensor_data;

    while (1) {
        if (DHT11_Read(&hdht11) == pdTRUE) {
            // �����ʪ������
            sensor_data.temp = hdht11.temperature;
            sensor_data.humi = hdht11.humidity;

            // ����֮ǰ�Ĺ���ǿ������
            SensorData_t last_data;
            if (xQueueReceive(xSensorQueue, &last_data, 0) == pdPASS) {
                sensor_data.lux = last_data.lux;
            }

            // ͨ�����з�����������
            xQueueSend(xSensorQueue, &sensor_data, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // 2��ɼ����
    }
}

// BH1750�ɼ�����
void BH1750_Task(void *arg) {
    SensorData_t sensor_data;

    BH1750_Init();  // ��ʼ��BH1750

    while (1) {
        // ��ȡ����ǿ��
        sensor_data.lux = BH1750_ReadLightIntensity();

        // ����֮ǰ����ʪ������
        SensorData_t last_data;
        if (xQueueReceive(xSensorQueue, &last_data, 0) == pdPASS) {
            sensor_data.temp = last_data.temp;
            sensor_data.humi = last_data.humi;
        }

        // ͨ�����з�����������
        xQueueSend(xSensorQueue, &sensor_data, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1��ɼ����
    }
}

// ��ʼ������
void Init_BSP(void *param) {
    USART_Config();  // ��ʼ������
    OLED_Init();     // ��ʼ��OLED
    DHT11_Init(&hdht11);  // ��ʼ��DHT11

    // �������ݶ��У��ɴ洢4�����ݣ�
    xSensorQueue = xQueueCreate(4, sizeof(SensorData_t));

    // ����Ӧ������
    xTaskCreate(DHT11_Task, "DHT11", 128, NULL, 2, NULL);
    xTaskCreate(BH1750_Task, "BH1750", 128, NULL, 2, NULL);
    xTaskCreate(OLED_Task, "OLED", 128, NULL, 1, NULL);

    vTaskDelete(NULL);  // ɾ����ʼ������
}

int main(void) {
    // ������ʼ������������ȼ���
    xTaskCreate(Init_BSP, "Init", 128, NULL, 3, NULL);

    // ����FreeRTOS������
    vTaskStartScheduler();

    while (1);
}

