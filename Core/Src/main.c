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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- HW-611 (BMP280) ADDRESSES --- */
#define HW611_ADDR            (0x76 << 1)
#define HW611_REG_TEMP_MSB    0xFA
#define HW611_REG_CTRL_MEAS   0xF4
#define HW611_REG_ID          0xD0

/* --- LCD ADDRESSES --- */
#define LCD_ADDR        (0x27 << 1)
#define LCD_BACKLIGHT   0x08
#define LCD_RS          0x01
#define LCD_EN          0x04
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t dig_T1;
int16_t  dig_T2, dig_T3;
float Temperature;
char uartBuf[64];
char lcdBuf[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* UART Functions */
void UART_SendString_PC(char *str);
void UART_SendString_BT(char *str);

/* HW-611 (BMP280) Functions */
uint8_t HW611_Read8(uint8_t reg);
uint16_t HW611_Read16LE(uint8_t reg);
void HW611_ReadCalibration(void);
int HW611_Init(void);
float HW611_ReadTemperature(void);

/* LCD Functions */
void LCD_Send(uint8_t data, uint8_t rs);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_Print(char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  UART_SendString_PC("SYSTEM START\r\n");
  UART_SendString_BT("SYSTEM START\r\n");

  /* HW-611 Initialization and Check */
  if (HW611_Init() != 0)
  {
      UART_SendString_PC("HW-611 ERROR\r\n");
      UART_SendString_BT("HW-611 ERROR\r\n");
      while (1); // Halt here if error occurs
  }

  LCD_Init();
  LCD_Print("Temperature:"); // Changed "Sicaklik" to "Temperature"
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 1. Read Temperature */
    Temperature = HW611_ReadTemperature();

    /* 2. Create String */
    sprintf(uartBuf, "Temp: %.2f C\r\n", Temperature);

    /* 3. Send via UART (PC and Bluetooth) */
    UART_SendString_PC(uartBuf);
    UART_SendString_BT(uartBuf);

    /* 4. Update LCD */
    LCD_Clear();
    sprintf(lcdBuf, "%.2f C", Temperature);
    LCD_Print("Temperature:");
    LCD_Send(0xC0, 0); // Move to next line
    LCD_Print(lcdBuf);

    /* 5. LED Control (Based on Temperature) */
    /* First, turn off all LEDs */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_RESET);

    if (Temperature < 20.0f)
    {
        // Cold: Turn on PC8
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    }
    else if (Temperature < 35.0f)
    {
        // Normal: Turn on PC5
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else
    {
        // Hot: Turn on PC6
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    }

    HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* Reset LED Pins (Turn Off) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /* Configure LED PINS (Set as Output) */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure USART2 PINS (PC) */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure USART1 PINS (Bluetooth) */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* --- UART Implementations --- */
void UART_SendString_PC(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void UART_SendString_BT(char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/* --- HW-611 (BMP280) Implementations --- */
uint8_t HW611_Read8(uint8_t reg)
{
    uint8_t val;
    HAL_I2C_Mem_Read(&hi2c1, HW611_ADDR, reg, 1, &val, 1, 100);
    return val;
}

uint16_t HW611_Read16LE(uint8_t reg)
{
    uint8_t data[2];
    HAL_I2C_Mem_Read(&hi2c1, HW611_ADDR, reg, 1, data, 2, 100);
    return (data[1] << 8) | data[0];
}

void HW611_ReadCalibration(void)
{
    dig_T1 = HW611_Read16LE(0x88);
    dig_T2 = (int16_t)HW611_Read16LE(0x8A);
    dig_T3 = (int16_t)HW611_Read16LE(0x8C);
}

int HW611_Init(void)
{
    /* Check Chip ID (Register 0xD0 must return 0x58) */
    if (HW611_Read8(HW611_REG_ID) != 0x58)
        return -1;

    /* Read calibration data */
    HW611_ReadCalibration();

    /* Set module to continuous measurement mode (Normal Mode) */
    uint8_t ctrl_meas = 0x27;
    HAL_I2C_Mem_Write(&hi2c1, HW611_ADDR, HW611_REG_CTRL_MEAS, 1, &ctrl_meas, 1, 100);
    return 0;
}

float HW611_ReadTemperature(void)
{
    int32_t var1, var2, adc_T, t_fine;
    uint8_t raw[3];

    /* Read temperature data (MSB, LSB, XLSB) */
    HAL_I2C_Mem_Read(&hi2c1, HW611_ADDR, HW611_REG_TEMP_MSB, 1, raw, 3, 100);

    adc_T = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);

    /* Bosch Datasheet Compensation Formula */
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
             ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
             ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    return ((t_fine * 5 + 128) >> 8) / 100.0f;
}

/* --- LCD Implementations --- */
void LCD_Send(uint8_t data, uint8_t rs)
{
    uint8_t high = (data & 0xF0) | rs | LCD_BACKLIGHT;
    uint8_t low  = ((data << 4) & 0xF0) | rs | LCD_BACKLIGHT;
    uint8_t buf[4] = { high | LCD_EN, high, low | LCD_EN, low };
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, buf, 4, 100);
}

void LCD_Init(void)
{
    HAL_Delay(50);
    LCD_Send(0x33, 0); LCD_Send(0x32, 0); LCD_Send(0x28, 0);
    LCD_Send(0x0C, 0); LCD_Send(0x06, 0); LCD_Send(0x01, 0);
    HAL_Delay(2);
}

void LCD_Clear(void)
{
    LCD_Send(0x01, 0);
    HAL_Delay(2);
}

void LCD_Print(char *str)
{
    while (*str) LCD_Send(*str++, LCD_RS);
}
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
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
