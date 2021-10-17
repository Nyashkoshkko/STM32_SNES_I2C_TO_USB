/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include "usbd_hid.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

// https://github.com/leech001/micros/blob/master/Inc/micros.h
__STATIC_INLINE void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}
__STATIC_INLINE void delay_us(uint32_t us)
{
	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000U);
	DWT->CYCCNT = 0U;
	while(DWT->CYCCNT < us_count_tic);
}

// func from nintendo driver sources
#define DF3_BTN_R      1
#define DF3_BTN_START  2
#define DF3_BTN_SELECT 4
#define DF3_BTN_L      5
#define DF3_BTN_DOWN   6
#define DF3_BTN_RIGHT  7
#define DF3_BTN_UP     0
#define DF3_BTN_LEFT   1
#define DF3_BTN_X      3
#define DF3_BTN_A      4
#define DF3_BTN_Y      5
#define DF3_BTN_B      6
static unsigned char get_bit(unsigned char data, int bitnum)
{
	return (data & (1 << bitnum)) >> bitnum;
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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	DWT_Init();

    int connected = 0;
    unsigned char data_i2c[21];
    unsigned char data_usb[10];

	#define ADDR (0x52 << 1)
	#define DELAY 5

    #define DELAY_STEP1 200
	#define DELAY_STEP2 200
	#define DELAY_STEP3 400

    int r = 0, start = 0, select_ = 0, l = 0, down = 0, right = 0, up = 0, left = 0, x = 0, a = 0, y = 0, b = 0;

    HAL_StatusTypeDef ret;
    while(1)
    {
    	if(connected == 0)
    	{
    		delay_us(DELAY_STEP2);
			
			// -> 2b: f0 55
			data_i2c[0] = 0xf0;
			data_i2c[1] = 0x55;
			ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR, data_i2c, 2, DELAY); // timeout in ticks, tick is 1 ms
			delay_us(DELAY_STEP1);
			if(ret != 0) continue;

    		// -> 2b: fb 00
			data_i2c[0] = 0xfb;
			data_i2c[1] = 0x00;
			ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR, data_i2c, 2, DELAY);
			if(ret != 0) continue;
			delay_us(DELAY_STEP1);

			// -> 2b: fe 03 // 03 - data format, 3 in driver
			data_i2c[0] = 0xfe;
			data_i2c[1] = 0x03;
			ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR, data_i2c, 2, DELAY);
			if(ret != 0) continue;
			delay_us(DELAY_STEP1);

			// -> 1b: fa
			data_i2c[0] = 0xfa;
			ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR, data_i2c, 1, DELAY);
			if(ret != 0) continue;
			delay_us(DELAY_STEP2); // wait 150-200 us // in raspberry pi driver in this init sequence it waits only 10 us

			// <- 6b: ?? ?? ?? ?? df id // df - data format, 3 in driver; id - controller id, 1 in driver
			if(HAL_I2C_Master_Receive(&hi2c1, ADDR, data_i2c, 6, DELAY) == HAL_OK)
			{
				if((data_i2c[4] == 0x03) && (data_i2c[5] == 0x01))
				{
					connected = 1;
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (GPIO_PinState) 1); // 9 - green
				}
				delay_us(DELAY_STEP2);
			}
			else
			{
				connected = 0;
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (GPIO_PinState) 0); // 9 - green
			}
    	}

    	if(connected == 1)
    	{
    		// -> 1b: 00
    		data_i2c[0] = 0x06; // 0x00; // read address 0 -> 6
			ret = HAL_I2C_Master_Transmit(&hi2c1, ADDR, data_i2c, 1, DELAY);
			if(ret != 0)
			{
				connected = 0;
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (GPIO_PinState) 0); // 9 - green
				continue;
			}
			delay_us(DELAY_STEP2); // wait 150-200 us // in raspberry pi driver it waits only 120 us

			// <- 21b: ?? ?? ?? ??  ?? ?? b0 b1  00 00 00 00  00 00 00 00  00 00 00 00  00 // b0, b1 - buttons of snes mini controller
			if(HAL_I2C_Master_Receive(&hi2c1, ADDR, &data_i2c[6], 2, DELAY) == HAL_OK) // read 2 bytes instead 21 for mininize delay
			{
				// interpret data
				r      = !get_bit(data_i2c[6], DF3_BTN_R);
				start  = !get_bit(data_i2c[6], DF3_BTN_START);
				select_= !get_bit(data_i2c[6], DF3_BTN_SELECT);
				l      = !get_bit(data_i2c[6], DF3_BTN_L);
				down   = !get_bit(data_i2c[6], DF3_BTN_DOWN);
				right  = !get_bit(data_i2c[6], DF3_BTN_RIGHT);
				up     = !get_bit(data_i2c[7], DF3_BTN_UP);
				left   = !get_bit(data_i2c[7], DF3_BTN_LEFT);
				x      = !get_bit(data_i2c[7], DF3_BTN_X);
				a      = !get_bit(data_i2c[7], DF3_BTN_A);
				y      = !get_bit(data_i2c[7], DF3_BTN_Y);
				b      = !get_bit(data_i2c[7], DF3_BTN_B);

				// RAM Factory Store usb gamepad hub protocol
				// 1 byte - ID (1)
				data_usb[0] = 1;
				// 6 bytes - X, Y, Z, Rx, Ry, Rz (0x81 - min, 0x7F - max)
				data_usb[1] = left ? 0x81 : right ? 0x7F : 0; // left - 0x81, right - 0x7F, center = 0
				data_usb[2] = up ? 0x81: down ? 0x7F : 0; // up - 0x81, down - 0x7F, center = 0
				data_usb[3] = data_usb[4] = data_usb[5] = data_usb[6] = 0;
				// 2 bytes - 16 bits of buttons values
				data_usb[7] = (b << 0) | (a << 1) | (y << 2) | (x << 3) | (select_ << 4) | (start << 5) | (l << 6) | (r << 7);
				data_usb[8] = 0;
				USBD_HID_SendReport(&hUsbDeviceFS, data_usb, 9);

				delay_us(DELAY_STEP3 - DELAY_STEP2); // sub delay after sending read command
			}
			else
			{
				connected = 0;
				//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (GPIO_PinState) 0); // 9 - green
			}
    	}
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
