#ifndef __ADXL345_SPI_H
#define __ADXL345_SPI_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#include "main.h"
#include "arm_math.h"

#define ADXL345DATA_DATALENGTH   512


// Адреса регистров ADXL345
#define ADXL345_DEVID              0x00  // Device ID
#define ADXL345_THRESH_TAP         0x1D  // Tap threshold
#define ADXL345_OFSX               0x1E  // X-axis offset
#define ADXL345_OFSY               0x1F  // Y-axis offset
#define ADXL345_OFSZ               0x20  // Z-axis offset
#define ADXL345_DUR                0x21  // Tap duration
#define ADXL345_LATENT             0x22  // Tap latency
#define ADXL345_WINDOW             0x23  // Tap window
#define ADXL345_THRESH_ACT         0x24  // Activity threshold
#define ADXL345_THRESH_INACT       0x25  // Inactivity threshold
#define ADXL345_TIME_INACT         0x26  // Inactivity time
#define ADXL345_ACT_INACT_CTL      0x27  // Axis enable control for activity/inactivity detection
#define ADXL345_THRESH_FF          0x28  // Free-fall threshold
#define ADXL345_TIME_FF            0x29  // Free-fall time
#define ADXL345_TAP_AXES           0x2A  // Axis control for single/double tap
#define ADXL345_ACT_TAP_STATUS     0x2B  // Source of single/double tap
#define ADXL345_BW_RATE            0x2C  // Data rate and power mode control
#define ADXL345_POWER_CTL          0x2D  // Power-saving features control
#define ADXL345_INT_ENABLE         0x2E  // Interrupt enable control
#define ADXL345_INT_MAP            0x2F  // Interrupt mapping control
#define ADXL345_INT_SOURCE         0x30  // Source of interrupts
#define ADXL345_DATA_FORMAT        0x31  // Data format control
#define ADXL345_DATAX0             0x32  // X-axis data 0
#define ADXL345_DATAX1             0x33  // X-axis data 1
#define ADXL345_DATAY0             0x34  // Y-axis data 1
#define ADXL345_DATAY1             0x35  // Y-axis data 0
#define ADXL345_DATAZ0             0x36  // Z-axis data 1
#define ADXL345_DATAZ1             0x37  // Z-axis data 0
#define ADXL345_FIFO_CTL           0x38  // FIFO control
#define ADXL345_FIFO_STATUS        0x39  // FIFO status

// Флаги для чтения/записи
#define ADXL345_READ               (1 << 7)
#define ADXL345_MULTIBYTE          (1 << 6)

// Режимы измерения (POWER_CTL)
#define ADXL345_MEASURE_MODE       0x08
#define ADXL345_STANDBY_MODE       0x00
#define ADXL345_SLEEP_MODE         0x04
#define ADXL345_AUTO_SLEEP_MODE    0x10
#define ADXL345_LINK_MODE          0x20
#define ADXL345_WAKEUP_8HZ         0x00
#define ADXL345_WAKEUP_4HZ         0x01
#define ADXL345_WAKEUP_2HZ         0x02
#define ADXL345_WAKEUP_1HZ         0x03

// Настройки DATA_FORMAT
#define ADXL345_RANGE_2G           (0x00 << 0)
#define ADXL345_RANGE_4G           (0x01 << 0)
#define ADXL345_RANGE_8G           (0x02 << 0)
#define ADXL345_RANGE_16G          (0x03 << 0)
#define ADXL345_FULL_RES           (1 << 3)
#define ADXL345_JUSTIFY_MSB        (1 << 2)
#define ADXL345_JUSTIFY_RIGHT      0x00
#define ADXL345_SELF_TEST          (1 << 7)
#define ADXL345_SPI_3WIRE          (1 << 6)
#define ADXL345_INT_INVERT         (1 << 5)

// Частоты семплирования (BW_RATE)
#define ADXL345_RATE_3200HZ        0x0F  // 3200 Hz
#define ADXL345_RATE_1600HZ        0x0E  // 1600 Hz
#define ADXL345_RATE_800HZ         0x0D  // 800 Hz
#define ADXL345_RATE_400HZ         0x0C  // 400 Hz
#define ADXL345_RATE_200HZ         0x0B  // 200 Hz
#define ADXL345_RATE_100HZ         0x0A  // 100 Hz
#define ADXL345_RATE_50HZ          0x09  // 50 Hz
#define ADXL345_RATE_25HZ          0x08  // 25 Hz
#define ADXL345_RATE_12_5HZ        0x07  // 12.5 Hz
#define ADXL345_RATE_6_25HZ        0x06  // 6.25 Hz
#define ADXL345_RATE_3_13HZ        0x05  // 3.13 Hz
#define ADXL345_RATE_1_56HZ        0x04  // 1.56 Hz
#define ADXL345_RATE_0_78HZ        0x03  // 0.78 Hz
#define ADXL345_RATE_0_39HZ        0x02  // 0.39 Hz
#define ADXL345_RATE_0_20HZ        0x01  // 0.20 Hz
#define ADXL345_RATE_0_10HZ        0x00  // 0.10 Hz
#define ADXL345_LOW_POWER          (1 << 4)

// FIFO режимы (FIFO_CTL)
#define ADXL345_FIFO_MODE_BYPASS   0x00
#define ADXL345_FIFO_MODE_FIFO     0x40
#define ADXL345_FIFO_MODE_STREAM   0x80
#define ADXL345_FIFO_MODE_TRIGGER  0xC0
#define ADXL345_FIFO_TRIG_INT2     (1 << 5)

// Битрейт SPI
#define ADXL345_SPI_BITRATE        5000000  // 5 MHz максимально для ADXL345

// Прерывания (INT_ENABLE, INT_MAP, INT_SOURCE)
#define ADXL345_INT_DATA_READY     (1 << 7)
#define ADXL345_INT_SINGLE_TAP     (1 << 6)
#define ADXL345_INT_DOUBLE_TAP     (1 << 5)
#define ADXL345_INT_ACTIVITY       (1 << 4)
#define ADXL345_INT_INACTIVITY     (1 << 3)
#define ADXL345_INT_FREE_FALL      (1 << 2)
#define ADXL345_INT_WATERMARK      (1 << 1)
#define ADXL345_INT_OVERRUN        (1 << 0)

// Tap axes control
#define ADXL345_TAP_X_EN           (1 << 2)
#define ADXL345_TAP_Y_EN           (1 << 1)
#define ADXL345_TAP_Z_EN           (1 << 0)
#define ADXL345_TAP_SUPPRESS       (1 << 3)

// Activity/Inactivity control
#define ADXL345_ACT_AC_DC          (1 << 7)
#define ADXL345_ACT_X_EN           (1 << 6)
#define ADXL345_ACT_Y_EN           (1 << 5)
#define ADXL345_ACT_Z_EN           (1 << 4)
#define ADXL345_INACT_AC_DC        (1 << 3)
#define ADXL345_INACT_X_EN         (1 << 2)
#define ADXL345_INACT_Y_EN         (1 << 1)
#define ADXL345_INACT_Z_EN         (1 << 0)

#define ADXL345_DEVID_VAL              0xE5




typedef struct {

	int16_t zdata[ADXL345DATA_DATALENGTH];

	float32_t  fft_result[ADXL345DATA_DATALENGTH / 2 + 1];



	float fft_samples;
	float main_freq;

	uint16_t dataindex;

	SPI_HandleTypeDef hspi;

	float offsetx,offsety,offsetz;
	float scale;

	GPIO_TypeDef* cs_port;
	uint16_t cs_pin;

	uint32_t ms1000;
	float ticks_per_us;

} adxl345_ic_t;



void ADXL345_Init_Calib(adxl345_ic_t *adxl345_ic);
void ADXL345_ReadXYZ(adxl345_ic_t *adxl345_ic );


uint8_t ADXL345_SPI_ReadByte(adxl345_ic_t *adxl345_ic, uint8_t reg_addr);
void ADXL345_SPI_ReadMulti(adxl345_ic_t *adxl345_ic,uint8_t reg_addr, uint8_t *data, uint8_t count);
void ADXL345_SPI_WriteByte(adxl345_ic_t *adxl345_ic,uint8_t reg_addr, uint8_t data);

uint8_t ADXL345_ReadID(adxl345_ic_t *adxl345_ic);
void ADXL345_FFT(adxl345_ic_t *adxl345_ic);
void ADXL345_2buf_FFT(adxl345_ic_t *buf1,adxl345_ic_t *buf2);
void ADXL345_FFT2buf(adxl345_ic_t *b1,adxl345_ic_t *b2);
float32_t ADXL345_FFT_qwen(float* input_buffer, uint32_t length, float32_t sample_rate_hz);

#endif /* __ADXL345_SPI_H */
