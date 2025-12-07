#include "adxl345_spi.h"
#include <string.h> // не обязателен, но может пригодиться


float32_t  fft_2buf_result[ADXL345DATA_DATALENGTH];
float      fft_2buf_freq[ADXL345DATA_DATALENGTH];
float      main_2buf_freq;


// Инициализация SPI и CS
void ADXL345_Init_Calib(adxl345_ic_t *adxl345_ic)
{
	uint8_t raw[6];

    if (ADXL345_ReadID( adxl345_ic)) {
        ADXL345_SPI_WriteByte(adxl345_ic,ADXL345_DATA_FORMAT, ADXL345_FULL_RES | ADXL345_RANGE_16G);
        ADXL345_SPI_WriteByte(adxl345_ic,ADXL345_BW_RATE, ADXL345_RATE_3200HZ);
        ADXL345_SPI_WriteByte(adxl345_ic,ADXL345_FIFO_CTL, ADXL345_FIFO_MODE_BYPASS);
        ADXL345_SPI_WriteByte(adxl345_ic,ADXL345_POWER_CTL, ADXL345_MEASURE_MODE);
    } else {
        while (1); // зависнуть или обработать ошибку
    }

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик


    adxl345_ic->ms1000 = DWT->CYCCNT;
    HAL_Delay(1000);
    adxl345_ic->ms1000 = DWT->CYCCNT - adxl345_ic->ms1000;  // если разделить число на ms1000 то получим время в секундах
    adxl345_ic->ticks_per_us = (float)adxl345_ic->ms1000/1000000.0;

    adxl345_ic->dataindex =0;

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (uint16_t i = 0; i < 100; i++) {
      ADXL345_SPI_ReadMulti(adxl345_ic, ADXL345_DATAX0, raw, 6);
      sum_x += (int16_t)(raw[0] | (raw[1] << 8));
      sum_y += (int16_t)(raw[2] | (raw[3] << 8));
      sum_z += (int16_t)(raw[4] | (raw[5] << 8));
    };

    adxl345_ic->offsetx = (float)sum_x / 100;
    adxl345_ic->offsety = (float)sum_y / 100;
    adxl345_ic->offsetz = (float)sum_z / 100;

    adxl345_ic->scale =9.81 * (1/sqrt(pow(adxl345_ic->offsetx,2)+pow(adxl345_ic->offsetz,2)+pow(adxl345_ic->offsetz,2)));

}


void delay_us(adxl345_ic_t *adxl345_ic,uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = roundl(us * adxl345_ic->ticks_per_us);
    while((DWT->CYCCNT - start) < cycles);
}


// Чтение одного байта
uint8_t ADXL345_SPI_ReadByte(adxl345_ic_t *adxl345_ic, uint8_t reg_addr)
{
    uint8_t tx_buf[2] = { (reg_addr | 0x80), 0x00 };
    uint8_t rx_buf[2] = { 0 };

    HAL_GPIO_WritePin( adxl345_ic->cs_port,  adxl345_ic->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&adxl345_ic->hspi, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin( adxl345_ic->cs_port,  adxl345_ic->cs_pin, GPIO_PIN_SET);

    return rx_buf[1];
}

// Запись одного байта
void ADXL345_SPI_WriteByte(adxl345_ic_t *adxl345_ic, uint8_t reg_addr, uint8_t data)
{
    uint8_t tx_buf[2] = { (reg_addr & 0x7F), data };
    uint8_t rx_dummy[2];

    HAL_GPIO_WritePin( adxl345_ic->cs_port,  adxl345_ic->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&adxl345_ic->hspi, tx_buf, rx_dummy, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin( adxl345_ic->cs_port,  adxl345_ic->cs_pin, GPIO_PIN_SET);
}

// Чтение нескольких байт — ТОЛЬКО статический буфер
void ADXL345_SPI_ReadMulti(adxl345_ic_t *adxl345_ic, uint8_t reg_addr, uint8_t *data, uint8_t count)
{
    if (count == 0 || count > 32) return; // защита от переполнения

    static uint8_t tx_buf[33];
    static uint8_t rx_buf[33];

    tx_buf[0] = reg_addr | ADXL345_READ | ADXL345_MULTIBYTE; // мультичтение


    HAL_GPIO_WritePin( adxl345_ic->cs_port,  adxl345_ic->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&adxl345_ic->hspi, tx_buf, rx_buf, count + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin( adxl345_ic->cs_port,  adxl345_ic->cs_pin, GPIO_PIN_SET);

    // Копируем принятые данные (начиная с rx_buf[1])
    for (uint8_t i = 0; i < count; i++) {
        data[i] = rx_buf[i + 1];
    }
}

// Чтение данных X, Y, Z
void ADXL345_ReadXYZ(adxl345_ic_t *adxl345_ic )
{
    uint8_t raw[6];
    ADXL345_SPI_ReadMulti(adxl345_ic, ADXL345_DATAZ0, raw, 2);

    adxl345_ic->zdata[adxl345_ic->dataindex]= (int16_t)(raw[0] | (raw[1] << 8));

    adxl345_ic->dataindex++;
    adxl345_ic->dataindex = adxl345_ic->dataindex % ADXL345DATA_DATALENGTH;
}

uint8_t ADXL345_ReadID(adxl345_ic_t *adxl345_ic)
{
    return (ADXL345_SPI_ReadByte(adxl345_ic, 0x00)==ADXL345_DEVID_VAL);
}





void ADXL345_FFT(adxl345_ic_t *adxl345_ic)  {
//void OPM_compute_fft_magnitude(uint16_t* input_buffer, uint32_t length, float32_t* magnitude_buffer) {
    // Проверка: длина должна быть степенью двойки

	uint32_t length = ADXL345DATA_DATALENGTH;

    if ((length & (length - 1)) != 0) {
        // Ошибка: длина не степень двойки
        return;
    }

    // Максимальный размер FFT в CMSIS-DSP зависит от библиотеки
    // Поддерживаемые размеры: 16, 32, 64, ..., 2048, 4096
    uint32_t fft_size = length;

    // Указатели на буферы
    static float32_t fft_input_buffer[ADXL345DATA_DATALENGTH];
    float32_t fft_output_buffer[ADXL345DATA_DATALENGTH];// Реальный и мнимый — чередуются

    // Инициализация FFT
    arm_rfft_fast_instance_f32 fft_instance;
    arm_status status;

    // Преобразуем uint16_t -> float32_t, вычитаем offsetz
    for (int i = 0; i < length; i++) {
        fft_input_buffer[i] = ((float32_t)( adxl345_ic->zdata[i] - adxl345_ic->offsetz ))*adxl345_ic->scale;
    }

    // Инициализация RFFT (для вещественного входа)
    status = arm_rfft_fast_init_f32(&fft_instance, fft_size);
    if (status != ARM_MATH_SUCCESS) {
        return; // Ошибка инициализации
    }

    // Выполнение прямого FFT
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0); // 0 = прямое преобразование

    // Вычисление магнитуд
    // Для RFFT результат — комплексный вектор длины fft_size, но симметричный
    // Нам нужны только первые (length/2 + 1) точек (от 0 до Nyquist)
    arm_cmplx_mag_f32(fft_output_buffer,adxl345_ic->fft_result, fft_size / 2 + 1);

    // Опционально: нормализация (деление на длину)
    arm_scale_f32(adxl345_ic->fft_result, 1.0f / fft_size, adxl345_ic->fft_result, fft_size / 2 + 1);

 //   for (int i = 0; i < (ADXL345DATA_DATALENGTH / 2 + 1); i++) {
 //     adxl345_ic->fft_freqs[i] = ((float)(adxl345_ic->fft_samples/2.0) * (float)i) / (float) (ADXL345DATA_DATALENGTH / 2 + 1);
 //   };


}

void ADXL345_2buf_FFT(adxl345_ic_t *buf1,adxl345_ic_t *buf2)  {
//void OPM_compute_fft_magnitude(uint16_t* input_buffer, uint32_t length, float32_t* magnitude_buffer) {
    // Проверка: длина должна быть степенью двойки

	uint32_t length = 2*ADXL345DATA_DATALENGTH;

    if ((length & (length - 1)) != 0) {
        // Ошибка: длина не степень двойки
        return;
    }

    // Максимальный размер FFT в CMSIS-DSP зависит от библиотеки
    // Поддерживаемые размеры: 16, 32, 64, ..., 2048, 4096
    uint32_t fft_size = length;

    // Указатели на буферы
    static float32_t fft_input_buffer[2*ADXL345DATA_DATALENGTH];
    float32_t fft_output_buffer[2*ADXL345DATA_DATALENGTH];// Реальный и мнимый — чередуются

    // Инициализация FFT
    arm_rfft_fast_instance_f32 fft_instance;
    arm_status status;

    // Преобразуем uint16_t -> float32_t, вычитаем offsetz
    for (int i = 0; i < ADXL345DATA_DATALENGTH; i++) {
        fft_input_buffer[2*i] =   ((float32_t)( buf1->zdata[i] - buf1->offsetz ))*buf1->scale;
        fft_input_buffer[2*i+1] = ((float32_t)( buf2->zdata[i] - buf2->offsetz ))*buf2->scale;
    }

 //   for (int i = 0; i < ADXL345DATA_DATALENGTH; i++) {
 //       fft_input_buffer[2*i] =   0;
 //       fft_input_buffer[2*i+1] = 0;
 //   }


    // Инициализация RFFT (для вещественного входа)
    status = arm_rfft_fast_init_f32(&fft_instance, fft_size);
    if (status != ARM_MATH_SUCCESS) {
        return; // Ошибка инициализации
    }

    // Выполнение прямого FFT
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0); // 0 = прямое преобразование

    // Вычисление магнитуд
    // Для RFFT результат — комплексный вектор длины fft_size, но симметричный
    // Нам нужны только первые (length/2 + 1) точек (от 0 до Nyquist)
    arm_cmplx_mag_f32(fft_output_buffer,fft_2buf_result, fft_size / 2 + 1);


    for (int i = 0; i <  ADXL345DATA_DATALENGTH ; i++) {
      fft_2buf_freq[i] = ((float)(buf1->fft_samples) * (float)i) / (float) (ADXL345DATA_DATALENGTH );
    };

    float max_val = fft_2buf_result[1];
    for (int i = 1; i <  ADXL345DATA_DATALENGTH ; i++) {
      if (fft_2buf_result[i]  > max_val) {
    	  max_val = fft_2buf_result[i];
    	  main_2buf_freq = fft_2buf_freq[i];
      }

    }



}
