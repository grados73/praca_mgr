
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "sd.h"
#include "ff.h"
#include "gps.h"
#include "funkcje_wlasne.h"
#include "SSD1331.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint16_t Timer1=0;
uint8_t sect[512];
extern char str1[60];
uint32_t byteswritten,bytesread;
uint8_t result;
char USER_Path[4]; /* logical drive path */
FATFS SDFatFs;
FATFS *fs;
FIL MyFile;

uint16_t cnt_pulse = 0; // inkrementowany z kazdym przerwaniem od odbiornika pulsu
uint16_t licznik1 = 0; // do okreslenia przerwan do pulsu
uint16_t licznik2 = 0;
uint16_t aktualny_puls = 0;
uint16_t puls_poprzedni = 0;
uint8_t flaga_puls = 0;
uint8_t flaga_koniec_ekr_start = 0;
uint16_t puls_koncowy = 0;
uint16_t puls_stary = 0;
uint16_t puls_sredni_trip = 0;
uint16_t puls_max = 0;

uint16_t przebyty_dystans = 0; // w metrach
uint16_t predkosc_max = 0;

uint8_t przycisk1 = 0;
uint8_t przycisk2 = 0;
uint8_t przycisk3 = 0;
uint8_t flaga_konca = 0;
uint8_t flaga_zapisu = 0;
uint8_t flaga_temp = 0;

uint16_t czas_ms = 0;
uint16_t czas_s = 0;
uint16_t czas_min = 0;
uint16_t flaga_stoper = 0;

uint16_t ilosc_satelit = 0;

uint8_t liczba_jed = 0;
uint8_t liczba_dzies = 0;
uint8_t liczba_setek = 0;

uint16_t PomiarADC;
float Temperature;
float Vsense;
 
const float V25 = 0.76; // [Volts]
const float Avg_slope = 0.0025; //[Volts/degree]
const float SupplyVoltage = 3.0; // [Volts]
const float ADCResolution = 4096.0;

		FRESULT res; //????????? ??????????
		
//================== .TXT DO SD======================
char wtext[100]="sek:    \r\nmin:    \r\nodl:    \r\nmxv:    \r\npls:    \r\nplm:    \r\n";
//================== .TXT DO SD======================

//char buffer1[512] ="To jest tekst testowy sluzacy sprawdzeniu karty,.... sporzadzony na sluzbie";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);  // PRZERWANIA ZEWNETRZNE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); // PRZERWANIA OD TIMerow 

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//===================== G P S ===================================#
			volatile struct gps_state gps_handle;  //PRZECHOWUJE BUFORY, LICZNIKI ZNAKOW + AKTUALNE DANE Z GPS
			volatile uint8_t recv_char;
			 
			void HAL_UART_RxCpltCallback(UART_HandleTypeDef * uart) { // FUNKCJA OBSLUGI PRZERWANIA OD UART - GPS
				if (uart == &huart1) {
					gps_recv_char(&gps_handle, recv_char);           // ZAPIS KOLEJNYCH ZNAKOW DO BUFORA BIBLIOTEKI GPS 
																													 // OD "$" DO "\n" LUB "\r", INKREMENTUJE writer_position
					HAL_UART_Receive_IT(&huart1, &recv_char, 1);
					}
				}
//===================== G P S ===================================#
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim6);
HAL_TIM_Base_Start_IT(&htim16);
HAL_TIM_Base_Start_IT(&htim7);


ssd1331_init();

				disk_initialize(SDFatFs.drv);
				
									if(f_mount(&SDFatFs,(TCHAR const*)USER_Path,0)!=FR_OK)
									{
										Error_Handler();
										flaga_temp = 1;
									}
									else
									{
										flaga_temp = 3;
									
//===================== G P S ===================================#
			gps_handle = gps_init(&huart1);
			HAL_UART_Receive_IT(&huart1, &recv_char, 1); //  WYWOLANIE PRZERWANIA OD UART - GPS, 
																									//po 1 znaku przekazujemy do recv_char
			char output_buffer[100];
			for (uint8_t i = 0; i < 100; i++)
				output_buffer[i] = '\0';
//===================== G P S ===================================#
									
oled_start(&flaga_koniec_ekr_start);
oled_ekr_glowny(czas_s, czas_min, puls_koncowy, ilosc_satelit);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 959;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 79;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 799;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin|CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZ_Pin|CS_SD_Pin|RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PULSE_IN_Pin BUTTON1_Pin BUTTON2_Pin */
  GPIO_InitStruct.Pin = PULSE_IN_Pin|BUTTON1_Pin|BUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON3_Pin */
  GPIO_InitStruct.Pin = BUTTON3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = DC_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin RES_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SD_Pin */
  GPIO_InitStruct.Pin = CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_SD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)           // Funkcja obslugujaca 
	{                                                    // przerwania zewnetrzne
	
		if(GPIO_Pin == PULSE_IN_Pin)                     //Jesli przerwanie pochodzi 
			{                                                    //od odbiornika pulsu
				
					aktualny_puls = licz_puls(licznik1);        //obliczenie akt. pulsu za 
																							       //pomoca funkcji licz_puls()
					licznik1 =0;	      //zerowanie licznika odpowiedzialnego za zliczanie
												                           //czasu pomiedzy przerwaniami
					if ( flaga_puls == 0)                          // pierwszy obieg petli
						{
							puls_poprzedni = aktualny_puls;                // el. jeden wstecz
							puls_stary = aktualny_puls;                      // el. dwa wstecz
							puls_koncowy = aktualny_puls + 1;
							flaga_puls = 1;                    // flaga pierwszego obiegu petli
						}
					else                          // jesli nie jest to pierwszy obieg petli
						{
							puls_koncowy = ((3*aktualny_puls + 2*puls_poprzedni+ puls_stary)/6);
												     			// srednia wazona do obliczania pulsu koncowego
							puls_stary = puls_poprzedni;          //uzupelnienie listy wiazanej
							puls_poprzedni = aktualny_puls;       //uzupelnienie listy wiazanej
						
						}
						// obsluga wyswietlacza - wyswietlanie aktualnego pulsu
						if(flaga_koniec_ekr_start == 1) oled_akt_puls(puls_koncowy); 
			}
			
//===================== PRZYCISKI ==============================#
			else if(GPIO_Pin == BUTTON1_Pin) // " X "
				{
					przycisk1 =! przycisk1;
					flaga_konca++;
					flaga_stoper = 100;
					
					if(flaga_konca <2) oled_header(3);
					
					
					//%%%%% trzeba dodac reset drogi przebytej - najlepiej funkcja inicjalizujaca zerami czy cos
					
					//oled_czas(0,0);
				}
			else if(GPIO_Pin == BUTTON2_Pin) // " > "
				{
					if(flaga_temp >=3)
					{
					przycisk2 =! przycisk2;
					flaga_stoper = 1;
					flaga_konca = 0;
								if(flaga_zapisu >= 1)
								{
										// TUTAJ WKLEIC FUNKCJE ZAPISU!!!
										flaga_zapisu = 100;
//									extern	FRESULT res; //????????? ??????????
//								extern		uint8_t wtext[50];
//------------------------------- ZAPIS NA SD --------------------------------------------------------------------
//															disk_initialize(SDFatFs.drv);
//				
//									if(f_mount(&SDFatFs,(TCHAR const*)USER_Path,0)!=FR_OK)
//									{
//										Error_Handler();
//									}
//									else
//									{
									// PRZYGOTOWYWANIE DANYCH DO ZAPISU NA KARTE
											konwert_liczb(czas_s, &liczba_jed, &liczba_dzies, &liczba_setek); //SEKUNDY
							 				wtext[5] = liczba_dzies; 
											wtext[6] = liczba_jed;
											konwert_liczb(czas_min, &liczba_jed, &liczba_dzies, &liczba_setek); //MINUTY
											wtext[15] = liczba_dzies; 
											wtext[16] = liczba_jed;
											konwert_liczb(przebyty_dystans, &liczba_jed, &liczba_dzies, &liczba_setek);	//ODLEGLOSC W m
											wtext[25] = liczba_setek;
											wtext[26] = liczba_dzies; 
											wtext[27] = liczba_jed;
											konwert_liczb(predkosc_max, &liczba_jed, &liczba_dzies, &liczba_setek);	 // SREDNIA PREDKOSC
											wtext[35] = liczba_setek;
											wtext[36] = liczba_dzies; 
											wtext[37] = liczba_jed;
											konwert_liczb(predkosc_max, &liczba_jed, &liczba_dzies, &liczba_setek);	// PREDKOSC MAX
											wtext[45] = liczba_setek;
											wtext[46] = liczba_dzies; 
											wtext[47] = liczba_jed;
											konwert_liczb(puls_sredni_trip, &liczba_jed, &liczba_dzies, &liczba_setek);	//PULS SREDNI ZA TRENING
											wtext[55] = liczba_setek;
											wtext[56] = liczba_dzies; 
											wtext[57] = liczba_jed;
											konwert_liczb(puls_max, &liczba_jed, &liczba_dzies, &liczba_setek);	//PULS MAX ZA TRENING
											wtext[65] = liczba_setek;
											wtext[66] = liczba_dzies; 
											wtext[67] = liczba_jed;
											
//										//Przeniesc to na dole w funkcje, ktora bedzie w for np i w zaleznosci od arg nazywala plik.txt, np 10 treningow mozna zapisac.
										if(f_open(&MyFile,"tr33.txt",FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK)
										{
											flaga_zapisu = 101;
											Error_Handler();
											
										}
										else
										{
											res=f_write(&MyFile,wtext,sizeof(wtext),(void*)&byteswritten);
											if((byteswritten==0)||(res!=FR_OK))
											{
												Error_Handler();
												flaga_zapisu = 102;
											}
											f_close(&MyFile);
											flaga_zapisu = 103;
										}
//									}
//------------------------------- ZAPIS NA SD --------------------------------------------------------------------
									oled_end(flaga_stoper);
										while(1);
								}
					oled_header(2);
					
									}
									else flaga_temp++;
									}
			else if(GPIO_Pin == BUTTON3_Pin) // " || "
				{
					przycisk3 =! przycisk3;
					flaga_stoper = 0;
					flaga_konca = 0;
								if(flaga_zapisu >= 1)
								{
									oled_end(flaga_stoper);
										flaga_zapisu = 200;
										while(1);
								}
					oled_header(1);
				}
//===================== PRZYCISKI ==============================#
}
//---------------------------------------------------------------------------------------------------------
//===================== TIMERY =================================#
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		
//-------------------- TIM6 -------------------------------------
	if(htim == &htim6)
	{
		Timer1++;
	}
//-------------------- TIM16 -------------------------------------
	if(htim->Instance == TIM16){ // Jezeli przerwanie pochodzi od timera 16 - 100Hz
		
				licznik1++; // inkrementowany z f=100Hz, tzn liczik1 = 1 = 10[ms]
				if(licznik1 >= 500) licznik1 =0; // Na wypadek niepodlaczenia pulsometru
		
				licznik2++;
				if((licznik2 >= 100) && (flaga_koniec_ekr_start == 1)) //PRZERWANIE CO 1S
				{
					ilosc_satelit = gps_handle.satelites_number;
					oled_akt_il_sat(ilosc_satelit);
					licznik2 = 0;
				}
		 }
//-------------------- TIM6 -------------------------------------
	if(htim->Instance == TIM7) // Jezeli przerwanie pochodzi od timera 7 -1000Hz = 1kHz
		 {
			 if(flaga_stoper == 1)
			 {
			 czas_ms++;
			 if(czas_ms >= 1000 && (flaga_koniec_ekr_start == 1))  // CO 1 S
			 {
				 czas_s++;
				 czas_ms = 0;
				 //  CO 5s OBLICZA SREDNI PULS ZA ODCINEK, JESLI JEST "START"
				 if( (czas_s != 0) && ((czas_s%5) == 0)) puls_sredni_trip = licz_sr_puls(puls_koncowy, &puls_max);
				 // CO 5s OBLICZA PRZEBYTA DROGE 
				 if( (czas_s != 0) && ((czas_s%5) == 0)) przebyty_dystans += licz_odleglosc(gps_handle.speed_kilometers, &predkosc_max);
					 
				 if((czas_s > 0) || (czas_min > 0)) oled_czas(czas_s, czas_min); // AKTUALIZACJA STOPERA
				 
				 if(czas_s == 60) // CO 1 MIN
				 {
					 czas_min++;
					 czas_s = 0;
					 oled_czas(czas_s, czas_min);
				 }
			 }
			 
		 }
			if((flaga_stoper == 100) && (flaga_konca >= 2)&&(flaga_konca <= 5))  // "X" - reset
			{
				
				flaga_koniec_ekr_start = 0;
				oled_koniec(przebyty_dystans, czas_min, czas_s, puls_sredni_trip); // EKRAN KONCOWY - PODSUMOWUJACY
//				oled_end(flaga_stoper);
//				czas_min = 0;
//				czas_s = 0;
//				czas_ms = 0;
				
				
				//oled_czas(czas_s, czas_min);  //oled_czas(0,0)
//				licz_sr_puls(777, &puls_max); // RESET SUMY SREDNIEGO PULSU I LICZNIKA SR PULSU
				flaga_stoper++;
				//flaga_konca = 0;
				flaga_zapisu = 1;
				
			}
//			if((flaga_konca == 2) && (flaga_stoper < 5))
//			{
//				//oled_start(&flaga_koniec_ekr_start);
//				flaga_konca = 0;
//				ssd1331_clear_screen(BLACK);
//			}
	 }

}

//=============================== ADC =====================================

//=============================== ADC =====================================
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
