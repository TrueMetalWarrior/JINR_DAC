/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDR (0x27 << 1)//Определение адреса LCD-дисплея

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags)
{
    HAL_StatusTypeDef res;
    for(;;)
    {
        res = HAL_I2C_IsDeviceReady(&hi2c2, lcd_addr, 1, HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
    }
    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;

    res = HAL_I2C_Master_Transmit(&hi2c2, lcd_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
    HAL_Delay(LCD_DELAY_MS);
    return res;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TS_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
int f=1,j=1;//Глобальные переменные, f-флаг для вывода значения шага напряжения на дисплей, j - значение шага напряжения
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd)
{
    LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data)
{
    LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_Init(uint8_t lcd_addr)
{
    // 4-bit mode, 2 lines, 5x7 format
    LCD_SendCommand(lcd_addr, 0b00110000);
    // display & cursor home (keep this!)
    LCD_SendCommand(lcd_addr, 0b00000010);
    // display on, right shift, underline off, blink off
    LCD_SendCommand(lcd_addr, 0b00001100);
    // clear display (optional here)
    LCD_SendCommand(lcd_addr, 0b00000001);
}

void LCD_SendString(uint8_t lcd_addr, char *str)
{
    while(*str)
    {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}

void Send()//Функция вывода данных на дисплей
{
	float Vdac;//Переменная, отвечающая за получение данных с ЦАП, в которой хранится численное значение с ЦАП
	char Vlcd[]=" Voltage:0000mV";//Массив для вывода значения напряжения на дисплей
	Vdac=HAL_DAC_GetValue(&hdac,DAC_CHANNEL_2);//Функция получения значения с ЦАП
	Vdac=4095-Vdac;//Из-за инвертированной работы ЦАП из 4095 вычитаем полученное значение с ЦАП для рассчета нужного напряжения
	Vdac=Vdac*1000/896+5;//Рассчет текущего напряжения на выходе с ЦАП, где 1000 - перевод в мВ, 896 - коэффициент преобразования из пропорции, 5 - смещение выводимого на дисплей значения т.к. минимальное значение на выводе ЦАП 5 мВ
	Vlcd[9]=((int)Vdac/1000);//Преобразование численных значений в символы ascii
	Vlcd[10]=(int)Vdac/100-Vlcd[9]*10;//Преобразование численных значений в символы ascii
	Vlcd[11]=(int)Vdac/10-Vlcd[9]*100-Vlcd[10]*10;//Преобразование численных значений в символы ascii
	Vlcd[12]=(int)Vdac-Vlcd[9]*1000-Vlcd[10]*100-Vlcd[11]*10;//Преобразование численных значений в символы ascii
	Vlcd[9]=Vlcd[9]+48;//Преобразование численных значений в символы ascii
	Vlcd[10]=Vlcd[10]+48;//Преобразование численных значений в символы ascii
	Vlcd[11]=Vlcd[11]+48;//Преобразование численных значений в символы ascii
	Vlcd[12]=Vlcd[12]+48;//Преобразование численных значений в символы ascii
    LCD_Init(LCD_ADDR);//Инициализация lcd
    // set address to 0x00
    LCD_SendCommand(LCD_ADDR, 0b10000000);//Отправка данных в 1ую строку
    LCD_SendString(LCD_ADDR, Vlcd);//В 1ой строке - текущее значение напряжения на выводе ЦАП
    if(f==1)//Условие для проверки значения шага для вывода на дисплей
    {
    	LCD_SendCommand(LCD_ADDR, 0b11000000);//Функция отправки данных во 2ую строку
    	LCD_SendString(LCD_ADDR, "    Step:1mV");
    }
    if(f==2)//Аналогично предыдущему условию
    {
    // set address to 0x40
    LCD_SendCommand(LCD_ADDR, 0b11000000);
    LCD_SendString(LCD_ADDR, "   Step:10mV");
    }
    if(f==3)//Аналогично предыдущему условию
    {
        // set address to 0x40
        LCD_SendCommand(LCD_ADDR, 0b11000000);
        LCD_SendString(LCD_ADDR, "   Step:100mV");
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
	float V,i;
	int Vd=0;//Значение напряжения на выходе ЦАП
	i=0.896;//коэффициент преобразования напряжения из пропорции i=4095/5, где 4095 - разрядность ЦАП, 5 - максимально выдаваемое напряжение, В
	V=4095;//Переменная отвечающая за текущее значение напряжения на выводе ЦАП
	//LCD_Init(LCD_ADDR);
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
  MX_TS_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac,DAC_CHANNEL_2);//Инициализация 2ого канала ЦАП

  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,4095);//Установка значения ЦАП в 5мВ
// !!!При переходе на новую плату возможно инвертирование значений, задаваемых в ЦАП, т.е. 0 будет соответствовать 0!!!
  HAL_GPIO_WritePin(GPIOB, LD4_Pin,1);//Зажигание светодиода для определения прошился ли микроконтроллер
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOB, LD4_Pin,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if((GPIOC->IDR & (1<<0))==0)//Условие для замыкания пина PC0, который обнуляет напряжение на выводе ЦАП
	  {
		  HAL_Delay(500);//Защита от дребезга клавиш
		  if((GPIOC->IDR & (1<<0))==0)
		  {
			  Vd=0;
			  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,4095);
			  HAL_GPIO_WritePin(GPIOB, LD4_Pin,1);
			  HAL_Delay(250);
			  HAL_GPIO_WritePin(GPIOB, LD4_Pin,0);
		  }
	  }
	  if((GPIOC->IDR & (1<<1))==0)//Условие для замыкания пина PC1, который изменяет шаг изменения напряжения на выводе ЦАП
	  	  {
	  		  HAL_Delay(500);
	  		  f++;//Инкриментация флага
	  		  if(f>3)//Условие для цикличности изменения шага
	  		  {
	  			  f=1;
	  		  }
	  		  switch(f)//Конструкция switch для выбора текущего значения f и, соответственно, выбора шага
	  		  {
	  		  	  case 1://Шаг 1мВ
	  		  	  {
	  		  		  j=1;
	  		  		  HAL_GPIO_WritePin(GPIOB, LD4_Pin,1);
	  		  		  HAL_Delay(200);
	  		  		  HAL_GPIO_WritePin(GPIOB, LD4_Pin,0);
	  		  		  break;
	  		  	  }
	  		  	  case 2://Шаг 10мВ
	  		  	  {
	  		  		  j=10;
	  		  		  HAL_GPIO_WritePin(GPIOB, LD4_Pin,1);
	  		  		  HAL_Delay(200);
	  		  		  HAL_GPIO_WritePin(GPIOB, LD4_Pin,0);
	  		  		  break;
	  		  	  }
	  		  	  case 3://Шаг 100мВ
	  		  	  {
	  		  		  j=100;
	  		  		  HAL_GPIO_WritePin(GPIOB, LD4_Pin,1);
	  		  		  HAL_Delay(200);
	  		  		  HAL_GPIO_WritePin(GPIOB, LD4_Pin,0);
	  		  		  break;
	  		  	  }
	  		  }
	  	  }
	  if((GPIOC->IDR & (1<<2))==0)//Увеличение напряжения на выводе ЦАП
	  {
		  Vd=Vd+j;//Vd - буффер для запоминания значения напряжения, прибавляя к нему величину j, которая зависит от выбранного шага, управляем выдаваемым напряжениием
		  if(Vd>250)//Ограничение по максимуму - 250мВ
		  {
			  Vd=0;
		  }
		  V=4095-i*Vd;//Преобразование значения напряжения в значение для ЦАП(без инвертирования просто i*Vd)
		  HAL_Delay(200);
		  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,V);//Установка полученного напряжения
		  HAL_GPIO_WritePin(GPIOB, LD3_Pin,1);
		  HAL_Delay(200);
		  HAL_GPIO_WritePin(GPIOB, LD3_Pin,0);
	  }
	  if((GPIOC->IDR & (1<<3))==0)//Уменьшение напряжения на выводе ЦАП
	  {
		  Vd=Vd-j;//Аналогично предыдущему условию, но с поправкой на уменьшение значения
		  if(Vd<0)//Ограничение по минимуму
		  {
			  Vd=0;
		  }
		  V=4095-i*Vd;//Преобразование значения напряжения в значение для ЦАП(без инвертирования просто i*Vd)
		  HAL_Delay(200);
		  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,V);
		  HAL_GPIO_WritePin(GPIOB, LD3_Pin,1);
		  HAL_Delay(200);
		  HAL_GPIO_WritePin(GPIOB, LD3_Pin,0);
	  }
	  HAL_Delay(200);
	  Send();
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IDD_CNT_EN_GPIO_Port, IDD_CNT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IDD_CNT_EN_Pin */
  GPIO_InitStruct.Pin = IDD_CNT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IDD_CNT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG0_Pin SEG1_Pin SEG2_Pin COM0_Pin 
                           COM1_Pin COM2_Pin SEG12_Pin */
  GPIO_InitStruct.Pin = SEG0_Pin|SEG1_Pin|SEG2_Pin|COM0_Pin 
                          |COM1_Pin|COM2_Pin|SEG12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG8_Pin SEG9_Pin SEG10_Pin SEG11_Pin 
                           SEG3_Pin SEG4_Pin SEG5_Pin SEG13_Pin 
                           COM3_Pin */
  GPIO_InitStruct.Pin = SEG8_Pin|SEG9_Pin|SEG10_Pin|SEG11_Pin 
                          |SEG3_Pin|SEG4_Pin|SEG5_Pin|SEG13_Pin 
                          |COM3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG18_Pin SEG19_Pin SEG20_Pin SEG21_Pin 
                           SEG22_Pin SEG23_Pin */
  GPIO_InitStruct.Pin = SEG18_Pin|SEG19_Pin|SEG20_Pin|SEG21_Pin 
                          |SEG22_Pin|SEG23_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
