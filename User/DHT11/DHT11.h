/* dht11.h */
#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef struct {
    uint8_t humidity;
    uint8_t temperature;
    TickType_t last_read_tick;
    SemaphoreHandle_t mutex;
} DHT11_HandleTypeDef;

#define DHT11_GPIO_PORT        GPIOA
#define DHT11_GPIO_PIN         GPIO_Pin_6
#define DHT11_RCC_APB          RCC_APB2Periph_GPIOA
#define DHT11_READ_INTERVAL    pdMS_TO_TICKS(1500)  

void DHT11_Init(DHT11_HandleTypeDef *hsensor);
BaseType_t DHT11_Read(DHT11_HandleTypeDef *hsensor);

#endif /* __DHT11_H */
