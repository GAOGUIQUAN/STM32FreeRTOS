#include "dht11.h"

static void DHT11_GPIO_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = DHT11_GPIO_PIN,
        .GPIO_Mode = GPIO_Mode_IPU,
        .GPIO_Speed = GPIO_Speed_50MHz
    };
    GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

static void DHT11_GPIO_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = DHT11_GPIO_PIN,
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_50MHz
    };
    GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

static void DHT11_Delay(uint16_t us)
{
    volatile uint32_t cycles = us * 9;  // 72MHz下校准值
    while(cycles--);
}

void DHT11_Init(DHT11_HandleTypeDef *hsensor) 
{
    RCC_APB2PeriphClockCmd(DHT11_RCC_APB, ENABLE);
    hsensor->mutex = xSemaphoreCreateMutex();
    configASSERT(hsensor->mutex != NULL);
    hsensor->last_read_tick = 0;
}

BaseType_t DHT11_Read(DHT11_HandleTypeDef *hsensor) 
{
    uint8_t buffer[5] = {0};
    uint32_t timeout;
    
    if(xSemaphoreTake(hsensor->mutex, pdMS_TO_TICKS(100)) != pdPASS) {
        return pdFALSE;
    }
    
    if((xTaskGetTickCount() - hsensor->last_read_tick) < DHT11_READ_INTERVAL) {
        vTaskDelay(DHT11_READ_INTERVAL - (xTaskGetTickCount() - hsensor->last_read_tick));
    }

    taskENTER_CRITICAL();
    
    /* 发送起始信号 */
    DHT11_GPIO_Output();
    GPIO_ResetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
    DHT11_Delay(18000);  // 18ms低电平
    
    GPIO_SetBits(DHT11_GPIO_PORT, DHT11_GPIO_PIN);
    DHT11_Delay(30);     // 主机拉高30us
    
    /* 切换输入模式 */
    DHT11_GPIO_Input();
    DHT11_Delay(10);
    
    /* 检测DHT11响应 */
    timeout = 0;
    while(GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == Bit_SET) {
        if(timeout++ > 100) {
            taskEXIT_CRITICAL();
            xSemaphoreGive(hsensor->mutex);
            return pdFALSE;
        }
        DHT11_Delay(1);
    }
    
    timeout = 0;
    while(GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == Bit_RESET) {
        if(timeout++ > 200) {
            taskEXIT_CRITICAL();
            xSemaphoreGive(hsensor->mutex);
            return pdFALSE;
        }
        DHT11_Delay(1);
    }
    
    timeout = 0;
    while(GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == Bit_SET) {
        if(timeout++ > 200) {
            taskEXIT_CRITICAL();
            xSemaphoreGive(hsensor->mutex);
            return pdFALSE;
        }
        DHT11_Delay(1);
    }
    
    /* 读取40位数据 */
    for(uint8_t i=0; i<40; i++) {
        while(GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == Bit_RESET);
        
        DHT11_Delay(40);  // 等待高电平稳定
        
        if(GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN)) {
            buffer[i/8] |= (1 << (7 - (i%8)));
            timeout = 0;
            while(GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN) == Bit_SET) {
                if(timeout++ > 100) break;
                DHT11_Delay(1);
            }
        }
    }
    
    taskEXIT_CRITICAL();
    xSemaphoreGive(hsensor->mutex);
    
    /* 数据校验 */
    if(buffer[0] + buffer[1] + buffer[2] + buffer[3] == buffer[4]) {
        hsensor->humidity = buffer[0];
        hsensor->temperature = buffer[2];
        hsensor->last_read_tick = xTaskGetTickCount();
        return pdTRUE;
    }
    
    return pdFALSE;
}
