#ifndef __ICS43434_I2S_H
#define __ICS43434_I2S_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


#define ICS43434_BUFFER_SIZE 1024

// Тип выходных данных: 16-битные signed сэмплы (после сдвига 24→16)
typedef int16_t ICS43434_Sample_t;

// Глобальный буфер (уже обработанный: 24→16 бит)
extern ICS43434_Sample_t g_ics43434_buffer[ICS43434_BUFFER_SIZE];

// Флаг готовности DMA
extern volatile bool g_ics43434_dma_ready;

// Инициализация I2S3 для ICS-43434
HAL_StatusTypeDef ICS43434_Init(void);

// Получить указатель на буфер и его размер
ICS43434_Sample_t* ICS43434_GetBuffer(void);
uint32_t ICS43434_GetBufferSize(void);

// Состояние
bool ICS43434_IsReady(void);

// Коллбэки (можно переопределить в user code)
void ICS43434_OnHalfTransfer(void);
void ICS43434_OnFullTransfer(void);

#endif /* __ICS43434_I2S_H */
