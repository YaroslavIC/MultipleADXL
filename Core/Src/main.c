/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adxl345_spi.h"
#include "stm32f401xc.h"
#include "arm_math.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define UART_BUFFER_SIZE 10

struct BTstruct{
	char receivecomplete;
	uint8_t receiveBufferchar;
	uint8_t receivebufferindex;
	uint8_t receiveBuffer[UART_BUFFER_SIZE];
	uint8_t cmd[UART_BUFFER_SIZE];
	uint32_t timeout_us;
	uint32_t priortime_us;
	uint32_t timeout_count;

} ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  ICS_BUFFER_RAW   ( 4*1024)
#define  ICS_BUFFER 	  (ICS_BUFFER_RAW / 2)
#define  ICS_BUFFER_FFT   (ICS_BUFFER / 2 + 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_rx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

uint8_t cmdbuf[4];
uint8_t stop_flag;
uint16_t i2s_buffercount = 0;

arm_status status;

static float32_t fft_input_buffer[ICS_BUFFER];
float32_t fft_output_buffer[ICS_BUFFER];// Реальный и мнимый — чередуются
arm_rfft_fast_instance_f32 fft_instance;

//static float32_t fft_input_buffer_out[ICS_BUFFER];


struct BTstruct BTUART;



int16_t i2s_buf[ICS_BUFFER_RAW];
//int16_t buf2[ICS_BUFFER];
static float32_t fft_result[ICS_BUFFER_FFT];

float ics_freq;

//adxl345_ic_t adxl345spi1;
//adxl345_ic_t adxl345spi2;
uint32_t adxl345_selector;


float adxl_freq ;
float b2zdata[2*ADXL345DATA_DATALENGTH];
HAL_StatusTypeDef  rdma;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
    //   if (adxl345_selector%2==0) {ADXL345_ReadXYZ(&adxl345spi1);};
    //   if (adxl345_selector%2==1) {ADXL345_ReadXYZ(&adxl345spi2);};

       adxl345_selector++;
    }
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){

//	for (int i = 0; i < ICS_BUFFER_RAW; i += 2) {
//	  buf2[i / 2]=i2s_buf[i];
//	};
	i2s_buffercount++;

}

float ICS43434_FFT2(int16_t *in_buf,float fs)  {

    // Указатели на буферы
    static float32_t fft_input_buffer[ICS_BUFFER];
    float32_t fft_output_buffer[ICS_BUFFER];// Реальный и мнимый — чередуются

    // Инициализация FFT
    arm_rfft_fast_instance_f32 fft_instance;
    arm_status status;



    // Преобразуем uint16_t -> float32_t, вычитаем offsetz
    for (int i = 0; i < ICS_BUFFER; i++) {
        fft_input_buffer[i] = (float32_t)(in_buf[i]);
   //     fft_input_buffer_out[i] = (float32_t)(in_buf[i]);
      // 	fft_input_buffer[i] = (float32_t)(sin(20.0*(float)i/(float)(ICS_BUFFER)*2.0*3.14159));
    }

    // Инициализация RFFT (для вещественного входа)
    status = arm_rfft_fast_init_f32(&fft_instance, ICS_BUFFER);
    if (status != ARM_MATH_SUCCESS) {
        return 0; // Ошибка инициализации
    }

    // Выполнение прямого FFT
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0); // 0 = прямое преобразование

    // Вычисление магнитуд
    // Для RFFT результат — комплексный вектор длины fft_size, но симметричный
    // Нам нужны только первые (length/2 + 1) точек (от 0 до Nyquist)
    arm_cmplx_mag_f32(fft_output_buffer,fft_result, ICS_BUFFER_FFT);

    // Опционально: нормализация (деление на длину)
    arm_scale_f32(fft_result, 1.0f / ICS_BUFFER, fft_result, ICS_BUFFER_FFT);

    float loc_main_freq = 0;

    float max_val = fft_result[1];
    for (int i = 2; i <  ICS_BUFFER_FFT  ; i++) {
      if (fft_result[i]  > max_val) {
    	max_val = fft_result[i];
    	loc_main_freq = i;
      }
    }

    loc_main_freq = (fs/2.0) * ((float)loc_main_freq / (float)ICS_BUFFER_FFT);

    return  loc_main_freq;
}

float ICS43434_FFT()  {

    // Указатели на буферы

    float fs = (float)hi2s3.Init.AudioFreq;

    // Инициализация FFT


    int j = 0;
    float32_t raw = 0;
    for (int i = 0; i < ICS_BUFFER; i++) {
        raw = (float32_t )i2s_buf[j];
    	fft_input_buffer[i]= raw;
   // 	fft_input_buffer_out[i]= raw;
    	j++;
    	j++;
    };


    // Выполнение прямого FFT
    arm_rfft_fast_f32(&fft_instance, fft_input_buffer, fft_output_buffer, 0); // 0 = прямое преобразование

    // Вычисление магнитуд
    // Для RFFT результат — комплексный вектор длины fft_size, но симметричный
    // Нам нужны только первые (length/2 + 1) точек (от 0 до Nyquist)
    arm_cmplx_mag_f32(fft_output_buffer,fft_result, ICS_BUFFER_FFT);

    // Опционально: нормализация (деление на длину)
    arm_scale_f32(fft_result, 1.0f / ICS_BUFFER, fft_result, ICS_BUFFER_FFT);

    float loc_main_freq = 0;

    float max_val = fft_result[1];
    for (int i = 2; i <  ICS_BUFFER_FFT  ; i++) {
      if (fft_result[i]  > max_val) {
    	max_val = fft_result[i];
    	loc_main_freq = i;
      }
    }

    loc_main_freq = (fs/2.0) * ((float)loc_main_freq / (float)ICS_BUFFER_FFT);

    return  loc_main_freq;
}

void CDC_ReceiveCallBack(uint8_t *Buf, uint32_t *Len)
{
	if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
		if ( (*Len>=4) && (Buf[3]==0x5A) ) {
			memcpy(cmdbuf, Buf, 4);
			Buf[3]=0;
		}
	}
}

void Buffer_Transmit(const uint8_t *pData, uint16_t Size_in_Bytes, uint16_t subPacketSize)
{

	uint16_t datalength =  Size_in_Bytes;
	uint8_t *tx_data = (uint8_t*)&datalength;

	CDC_Transmit_FS((uint8_t*)tx_data,2);

	//HAL_UART_Transmit(bthuart,  tx_data,	2, Timeout);
    HAL_Delay(1);

	uint16_t i=0;
	while (i<Size_in_Bytes) {
	   if ((i+subPacketSize)<Size_in_Bytes) {
		   datalength = subPacketSize;
	   } else {
		   datalength =  Size_in_Bytes - i;
	   }

	   CDC_Transmit_FS((uint8_t*) (pData+i),	 datalength );
	   i = i + datalength;
	   HAL_Delay(1);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */



  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_I2S3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */



//  adxl345spi1.dataindex = 0;
//  adxl345spi1.hspi = hspi1;
//  adxl345spi1.cs_port = GPIOA;
//  adxl345spi1.cs_pin = GPIO_PIN_10;
 // adxl345spi1.fft_samples = 3200;

 // adxl345spi2.dataindex = 0;
 // adxl345spi2.hspi = hspi2;
 // adxl345spi2.cs_port = GPIOB;
//  adxl345spi2.cs_pin = GPIO_PIN_7;
//  adxl345spi2.fft_samples = 3200;


//  ADXL345_Init_Calib(&adxl345spi1);
//  ADXL345_Init_Calib(&adxl345spi2);

  // Инициализация RFFT (для вещественного входа)
  status = arm_rfft_fast_init_f32(&fft_instance, ICS_BUFFER);
  if (status != ARM_MATH_SUCCESS) {
     while(1){;};; // Ошибка инициализации
  }


  stop_flag=0;

  HAL_TIM_Base_Start_IT(&htim3);
  rdma = HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *) i2s_buf, ICS_BUFFER_RAW);
  //rdma = HAL_I2S_DMAPause(&hi2s3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 // if (stop_flag==1) {
		//  HAL_TIM_Base_Stop_IT(&htim3);
		//  rdma = HAL_I2S_DMAPause(&hi2s3);

	 // }

	//  if (stop_flag==0) {
		//  HAL_TIM_Base_Start_IT(&htim3);
		//  HAL_I2S_DMAResume(&hi2s3);
		//  stop_flag=2;
	//  };

    //	for (int i = 0; i < ICS_BUFFER_RAW; i += 2) {  buf2[i / 2]=i2s_buf[i]+1; };
	  ics_freq = ICS43434_FFT();

	//  ADXL345_FFT(&adxl345spi1);
	//  ADXL345_FFT(&adxl345spi2);

	//    for (int i = 0; i < ADXL345DATA_DATALENGTH; i++) {
	 //   	b2zdata[2*i]     =  ((float)( adxl345spi1.zdata[i] - adxl345spi1.offsetz ))*adxl345spi1.scale;
	 //   	b2zdata[2*i+1] =  ((float)( adxl345spi2.zdata[i] - adxl345spi2.offsetz ))*adxl345spi2.scale;
	 //   }

	 //  adxl_freq = ADXL345_FFT_qwen(b2zdata,2*ADXL345DATA_DATALENGTH, 6400);


		  if((cmdbuf[0] == 'A') && (cmdbuf[3] == 'Z'))
		  {
			  cmdbuf[0] = 0;

		//	  Buffer_Transmit((uint8_t *)fft_input_buffer_out,4*ICS_BUFFER,4*ICS_BUFFER);
		  };

		  if((cmdbuf[0] == 'B') && (cmdbuf[3] == 'Z'))
		  {
			  cmdbuf[0] = 0;
			//  Buffer_Transmit((uint8_t *)buf2,2*ICS_BUFFER,500);
			  //Buffer_Transmit((uint8_t *)i2s_buf,2*ICS_BUFFER_RAW,500);
		//	  Buffer_Transmit((uint8_t *)fft_input_buffer_out,4*ICS_BUFFER,4*ICS_BUFFER);
		  };


		  if((cmdbuf[0] == 'C') && (cmdbuf[3] == 'Z'))
		  {
			  cmdbuf[0] = 0;
			  Buffer_Transmit((uint8_t *)fft_result,4*ICS_BUFFER_FFT,500);
		  };

		  if((cmdbuf[0] == 'D') && (cmdbuf[3] == 'Z'))
		  {
			  cmdbuf[0] = 0;
			  Buffer_Transmit((uint8_t *)b2zdata,2*2*ADXL345DATA_DATALENGTH,500);
		  };

		  if((cmdbuf[0] == 'E') && (cmdbuf[3] == 'Z'))
		  {
			  cmdbuf[0] = 0;
			stop_flag = 1;
		  };

		  if((cmdbuf[0] == 'F') && (cmdbuf[3] == 'Z'))
		  {
			  cmdbuf[0] = 0;
			stop_flag = 0;
		  };

		  if((cmdbuf[0] == 'G') && (cmdbuf[3] == 'Z'))
		  {
			  cmdbuf[0] = 0;
			   //Buffer_Transmit((uint8_t *)adxl345spi1.zdata,2*ADXL345DATA_DATALENGTH,500);
			   Buffer_Transmit((uint8_t *)i2s_buf,2*ICS_BUFFER_RAW,500);
		  };


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 155;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
