/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    C3, Cs3, D3, Ds3, E3, F3, Fs3, G3, Gs3, A3, As3, B3,
    C4, Cs4, D4, Ds4, E4, F4, Fs4, G4, Gs4, A4, As4, B4,
    C5, Cs5, D5, Ds5, E5, F5, Fs5, G5, Gs5, A5, As5, B5,
    C6, Cs6, D6, Ds6, E6, F6, Fs6, G6, Gs6, A6, As6, B6,
    C7, Cs7, D7, Ds7, E7, F7, Fs7, G7, Gs7, A7, As7, B7
} Notes;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CS43L22_I2C_ADDRESS 	0x94
#define I2C_TIMEOUT 			10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
int16_t dataI2S[100] = {0};

unsigned int isMyMelody = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CS43L22_Init(void) {

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

	  uint8_t TxBuffer[2];

	  TxBuffer[0] = 0x0D;
	  TxBuffer[1] = 0x01;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x00;
	  TxBuffer[1] = 0x99;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x47;
	  TxBuffer[1] = 0x80;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x32;
	  TxBuffer[1] = 0xFF;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x32;
	  TxBuffer[1] = 0x7F;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x00;
	  TxBuffer[1] = 0x00;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x04;
	  TxBuffer[1] = 0xAF;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  //
	  TxBuffer[0] = 0x00;
	  TxBuffer[1] = 0x70;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x05;
	  TxBuffer[1] = 0x81;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x06;
	  TxBuffer[1] = 0x07;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x0A;
	  TxBuffer[1] = 0x00;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x27;
	  TxBuffer[1] = 0x00;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x1A;
	  TxBuffer[1] = 0x0A;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x1B;
	  TxBuffer[1] = 0x0A;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x1F;
	  TxBuffer[1] = 0x0F;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	  TxBuffer[0] = 0x02;
	  TxBuffer[1] = 0x9E;
	  HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

}

void CS43L22_Beep(Notes note, uint32_t duration_ms) {
	uint8_t TxBuffer[2];

	TxBuffer[0] = 0x1D;
	TxBuffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);


	TxBuffer[0] = 0x1C;
	switch(note) {
		case C4:
			TxBuffer[1] = 0x00;
			break;
		case C5:
			TxBuffer[1] = 0x10;
			break;
		case D5:
			TxBuffer[1] = 0x20;
			break;
		case E5:
			TxBuffer[1] = 0x30;
			break;
		case F5:
			TxBuffer[1] = 0x40;
			break;
		case G5:
			TxBuffer[1] = 0x50;
			break;
		case A5:
			TxBuffer[1] = 0x60;
			break;
		case B5:
			TxBuffer[1] = 0x70;
			break;
		case C6:
			TxBuffer[1] = 0x80;
			break;
		case D6:
			TxBuffer[1] = 0x90;
			break;
		case E6:
			TxBuffer[1] = 0xA0;
			break;
		case F6:
			TxBuffer[1] = 0xB0;
			break;
		case G6:
			TxBuffer[1] = 0xC0;
			break;
		case A6:
			TxBuffer[1] = 0xD0;
			break;
		case B6:
			TxBuffer[1] = 0xE0;
			break;
		case C7:
			TxBuffer[1] = 0xF0;
			break;
		default:
			TxBuffer[1] = 0x00;
			break;
	}
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	TxBuffer[0] = 0x1E;
	TxBuffer[1] = 0xC0;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);

	HAL_Delay(duration_ms);

	TxBuffer[0] = 0x1E;
	TxBuffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t*) &TxBuffer, 2, I2C_TIMEOUT);
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  CS43L22_Init();
  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dataI2S, 100);

    void allNotes(void) {
  	  Notes note;

  	  for (note = C4; note <= C7; note++) {
  		  CS43L22_Beep(note, 200);
  		  HAL_Delay(200);
  	  }
    }
    void newMelody() {
        // You can define a new melody here
        // For example, a simple ascending scale
        Notes scale[] = {C6, D6, E6, F6, G6, A6, B6, C5};

        for (int i = 0; i < sizeof(scale) / sizeof(scale[0]); i++) {
            CS43L22_Beep(scale[i], 200);
            HAL_Delay(200);
        }
    }

    void spearOfJusticeMelody() {
        // Main theme of "Spear of Justice" from Undertale
        Notes melody[] = {D5, E5, F5, G5, A5, G5, F5, E5,
                          D5, D5, E5, E5, F5, G5, A5, G5,
                          F5, E5, D5, E5, F5, E5, D5, C5};

        int durations[] = {400, 200, 200, 200, 200, 200, 200, 200,
                           400, 200, 200, 200, 200, 200, 200, 200,
                           400, 200, 200, 200, 200, 200, 200, 400};

        for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
            CS43L22_Beep(melody[i], durations[i]);
            HAL_Delay(20);  // Add a short pause between notes
        }
    }

    void metalCrusherMelody() {
        // "Metal Crusher" from Undertale
        Notes melody[] = {D6, F6, G6, A6, F6, E6, C6, E6, G6, A6, G6, F6,
                          D6, F6, G6, A6, F6, E6, C6, E6, G6, A6, G6, F6,
                          A5, D6, F6, G6, A6, F6, E6, C6, E6, G6, A6, G6, F6,
                          D6, F6, G6, A6, F6, E6, C6, E6, G6, A6, G6, F6};

        int durations[] = {300, 150, 150, 300, 150, 150, 150, 150, 150, 300, 150, 150,
                           300, 150, 150, 300, 150, 150, 150, 150, 150, 300, 150, 150,
                           300, 150, 150, 150, 150, 300, 150, 150, 150, 150, 150, 300, 150, 150,
                           300, 150, 150, 150, 150, 300, 150, 150, 150, 150, 150, 300};

        for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
            CS43L22_Beep(melody[i], durations[i]);
            HAL_Delay(20);  // Add a short pause between notes
        }
    }

    void superMarioBrosMainTheme() {
        Notes melody[] = {E5, E5, E5, C5, E5, G5, G4, C5, G4, E4, A4, A4, A4, A4,
                          E5, E5, E5, C5, E5, G5, G4, C5, G4, E4, A4, A4, A4, A4,
                          G5, G5, G5, G4, F5, F5, F5, F4, E5, E5, E5, D5, D5, D5, D4,
                          C5, C5, C5, C4, G4, G4, G4, G4, A4, A4, A4, A4, G4, F4, F4, F4, F4,
                          G4, G4, G4, G4, A4, A4, A4, A4, G4, F4, F4, F4, F4, E4, E4, E4, E4,
                          A4, A4, A4, A4, B4, B4, B4, B4, C5, C5, C5, C5, D5, D5, D5, D5,
                          E5, E5, E5, E5, C5, C5, C5, C5, D5, D5, D5, D5, E5, E5, E5, E5,
                          G5, G5, G5, G5, E5, E5, E5, E5, G5, G5, G5, G5, A5, A5, A5, A5,
                          G5, F5, F5, F5, F5, G5, G5, G5, G5, A5, A5, A5, A5, G5, F5, F5, F5, F5, E5, E5, E5, E5,
                          A5, A5, A5, A5, B5, B5, B5, B5, C6, C6, C6, C6, D6, D6, D6, D6,
                          E6, E6, E6, E6, G5, G5, G5, G5, A5, A5, A5, A5, G5, F5, F5, F5, F5,
                          G5, G5, G5, G5, A5, A5, A5, A5, G5, F5, F5, F5, F5, E5, E5, E5, E5,
                          A5, A5, A5, A5, B5, B5, B5, B5, C6, C6, C6, C6, D6, D6, D6, D6,
                          E6, E6, E6, E6, G5, G5, G5, G5, A5, A5, A5, A5, G5, F5, F5, F5, F5,
                          G5, G5, G5, G5, A5, A5, A5, A5, G5, F5, F5, F5, F5, E5, E5, E5, E5};

        int durations[] = {400, 400, 400, 200, 400, 200, 200, 400, 200, 200, 400, 200, 200, 200,
                           400, 400, 400, 200, 400, 200, 200, 400, 200, 200, 400, 200, 200, 200,
                           400, 400, 400, 200, 400, 200, 200, 400, 200, 200, 200, 200, 200, 200, 200,
                           400, 400, 400, 200, 400, 200, 200, 400, 200, 200, 400, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200,
                           200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200};

        for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
            CS43L22_Beep(melody[i], durations[i]);
            HAL_Delay(10);  // Add a short pause between notes
        }
    }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();
    if (isMyMelody == 1) {
    	metalCrusherMelody();
    	  } else {
    		  allNotes();
    	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           PD4 */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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
