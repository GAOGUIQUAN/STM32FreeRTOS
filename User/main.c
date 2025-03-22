#include "stm32f10x.h"
#include "OLED.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "BSP/UART1/bsp_uart1.h"
#include "dht11.h"
#include "bh1750.h"

// 定义全局数据结构
DHT11_HandleTypeDef hdht11;
QueueHandle_t xSensorQueue;

// 定义传感器数据结构
typedef struct {
    uint8_t temp;
    uint8_t humi;
    float lux;
} SensorData_t;

// 显示任务（集成串口发送）
void OLED_Task(void *arg) {
    SensorData_t sensor_data;
    char str_temp[16], str_humi[16], str_lux[16];
    static uint8_t last_temp = 0, last_humi = 0;  // 保留上一次的温湿度数据

    // 初始化显示
    OLED_ShowString(1, 1, "Temp:   C");
    OLED_ShowString(2, 1, "Humi:   %");
    OLED_ShowString(3, 1, "Lux:    lx");
    AT_Send_Command("System Start");

    while (1) {
        // 从队列获取数据
        if (xQueueReceive(xSensorQueue, &sensor_data, pdMS_TO_TICKS(100)) == pdPASS) {
            /* OLED显示更新 */
            if (sensor_data.temp != 0) {  // 检查温湿度数据是否为0
                last_temp = sensor_data.temp;
                last_humi = sensor_data.humi;
            }
            sprintf(str_temp, "%2d", last_temp);
            OLED_ShowString(1, 6, str_temp);
            sprintf(str_humi, "%2d", last_humi);
            OLED_ShowString(2, 6, str_humi);
            sprintf(str_lux, "%.2f", sensor_data.lux);
            OLED_ShowString(3, 6, str_lux);

            /* 串口发送（保持原有函数调用方式） */
            char buffer[64];
            sprintf(buffer, "Temp:%dC Humi:%d%% Lux:%.2flx", last_temp, last_humi, sensor_data.lux);
            AT_Send_Command(buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(500));  // 500ms刷新一次
    }
}

// DHT11采集任务
void DHT11_Task(void *arg) {
    SensorData_t sensor_data;

    while (1) {
        if (DHT11_Read(&hdht11) == pdTRUE) {
            // 填充温湿度数据
            sensor_data.temp = hdht11.temperature;
            sensor_data.humi = hdht11.humidity;

            // 保留之前的光照强度数据
            SensorData_t last_data;
            if (xQueueReceive(xSensorQueue, &last_data, 0) == pdPASS) {
                sensor_data.lux = last_data.lux;
            }

            // 通过队列发送完整数据
            xQueueSend(xSensorQueue, &sensor_data, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // 2秒采集间隔
    }
}

// BH1750采集任务
void BH1750_Task(void *arg) {
    SensorData_t sensor_data;

    BH1750_Init();  // 初始化BH1750

    while (1) {
        // 读取光照强度
        sensor_data.lux = BH1750_ReadLightIntensity();

        // 保留之前的温湿度数据
        SensorData_t last_data;
        if (xQueueReceive(xSensorQueue, &last_data, 0) == pdPASS) {
            sensor_data.temp = last_data.temp;
            sensor_data.humi = last_data.humi;
        }

        // 通过队列发送完整数据
        xQueueSend(xSensorQueue, &sensor_data, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒采集间隔
    }
}

// 初始化任务
void Init_BSP(void *param) {
    USART_Config();  // 初始化串口
    OLED_Init();     // 初始化OLED
    DHT11_Init(&hdht11);  // 初始化DHT11

    // 创建数据队列（可存储4组数据）
    xSensorQueue = xQueueCreate(4, sizeof(SensorData_t));

    // 创建应用任务
    xTaskCreate(DHT11_Task, "DHT11", 128, NULL, 2, NULL);
    xTaskCreate(BH1750_Task, "BH1750", 128, NULL, 2, NULL);
    xTaskCreate(OLED_Task, "OLED", 128, NULL, 1, NULL);

    vTaskDelete(NULL);  // 删除初始化任务
}

int main(void) {
    // 创建初始化任务（最高优先级）
    xTaskCreate(Init_BSP, "Init", 128, NULL, 3, NULL);

    // 启动FreeRTOS调度器
    vTaskStartScheduler();

    while (1);
}

