#include "ics43434_i2s.h"
//#include "stm32f4xx_hal_i2s.h"


// Статический буфер для необработанных 32-битных данных от I2S
static volatile int32_t i2s_raw_buffer[ICS43434_BUFFER_SIZE];

// Выходной буфер: 16-битные сэмплы (24→16)
ICS43434_Sample_t g_ics43434_buffer[ICS43434_BUFFER_SIZE];
volatile bool g_ics43434_dma_ready = false;

// Хендлеры
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_rx;

// ======================================================================================
// Инициализация I2S3 как Slave Receiver для ICS-43434
// ======================================================================================
HAL_StatusTypeDef ICS43434_Init(void)
{
    // Включение тактирования
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
        gpio.Pin = GPIO_PIN_15;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF6_SPI3;  // AF6 для I2S3
        HAL_GPIO_Init(GPIOA, &gpio);

        // PB3 - Serial Clock (CK/BCLK), PB5 - Serial Data (SD/DIN)
        gpio.Pin = GPIO_PIN_3 | GPIO_PIN_5;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF6_SPI3;  // AF6 для I2S3
        HAL_GPIO_Init(GPIOB, &gpio);

    // Настройка I2S3
        hi2s3.Instance = SPI3;
        hi2s3.Init.Mode = I2S_MODE_SLAVE_RX;
        hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
        hi2s3.Init.DataFormat = I2S_DATAFORMAT_32B; // 32-битные фреймы
        hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
        hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K; // значение формальное (slave)
        hi2s3.Init.CPOL = I2S_CPOL_LOW;           // данные по переднему фронту
        hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
        hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;

    if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
        return HAL_ERROR;
    }

    // Настройка DMA
    hdma_spi3_rx.Instance = DMA1_Stream0;
    hdma_spi3_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_spi3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_spi3_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_spi3_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_spi3_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

    __HAL_LINKDMA(&hi2s3, hdmarx, hdma_spi3_rx);

    if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) {
        return HAL_ERROR;
    }

    // Включаем прерывания DMA
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);



    // Запуск приёма в циклическом режиме
    if (HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*)i2s_raw_buffer, ICS43434_BUFFER_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_DMA_Start_IT(hi2s3.hdmarx, (uint32_t)&hi2s3.Instance->DR,
                     (uint32_t)i2s_raw_buffer, ICS43434_BUFFER_SIZE);

    g_ics43434_dma_ready = false;
    return HAL_OK;
}

// ======================================================================================
// Обработчики DMA: преобразование 24→16 бит при передаче
// ======================================================================================
__weak void ICS43434_OnHalfTransfer(void) {
    // По умолчанию: преобразуем первую половину буфера
    for (uint32_t i = 0; i < ICS43434_BUFFER_SIZE / 2; i++) {
        // ICS-43434: 24-bit signed в старших 24 битах 32-битного слова (MSB-aligned)
        // Сдвигаем вправо на 8, чтобы получить 16-битное signed значение
        g_ics43434_buffer[i] = (int16_t)(i2s_raw_buffer[i] >> 8);
    }
}

__weak void ICS43434_OnFullTransfer(void) {
    // Преобразуем вторую половину буфера
    for (uint32_t i = ICS43434_BUFFER_SIZE / 2; i < ICS43434_BUFFER_SIZE; i++) {
        g_ics43434_buffer[i] = (int16_t)(i2s_raw_buffer[i] >> 8);
    }
    g_ics43434_dma_ready = true;

}


// ======================================================================================
// Вспомогательные функции
// ======================================================================================
ICS43434_Sample_t* ICS43434_GetBuffer(void)
{
    return g_ics43434_buffer;
}

uint32_t ICS43434_GetBufferSize(void)
{
    return ICS43434_BUFFER_SIZE;
}

bool ICS43434_IsReady(void)
{
    return g_ics43434_dma_ready;
}
