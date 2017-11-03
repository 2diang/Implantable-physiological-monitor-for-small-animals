/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "lptim.h"
#include "opamp.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ADXL362.h"
#include "diskio.h"
#include "ff.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO uint16_t ECG_Data = 0; // store ECG data
uint16_t ACCE_Data[] = {0,0,0}; // store accelarometer data
float T_Data = 0;
uint8_t RTC_Data[6] ={0,0,0,0,0,0} ;
FATFS SDFatFs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void Sys(void);
static void ECG_Get_Data(void);
static void T_Get_Data(void);
static uint8_t ADXL362_Init(void);
static void ACCE_Get_Data(void);
static void RTC_GetTime();
static void SD_Run(void);
static void SD_Stop(void);
static void MODE_Stop(void);
static void MODE_Active(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

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
  MX_LPTIM1_Init();
  MX_LPTIM2_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_OPAMP1_Init();

  /* USER CODE BEGIN 2 */
  TIM_1ms_Start();
	TIM_1s_Start();
	LTIM_1min_Start();
	LTIM_1Hour_Start();
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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPTIM1
                              |RCC_PERIPHCLK_LPTIM2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
  PeriphClkInit.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/* * * * * * * * * * * * * * * * * * * * * Sys * * * * * * * * * * * * * * * * * * * * */
static void Sys(void)
{
	switch(SYS_S)
	{
		case IDLE:
			// do nothing
			break;
		case 	ECG:
			ECG_Get_Data();
			break;
		case T_ACCE:
			T_Get_Data();
		  ACCE_Get_Data();
			break;
		case SD:
			TIM_1ms_Stop();
		  TIM_1s_Stop();
		  LTIM_1min_Stop();
			SD_Run();
		  SYS_S = SLEEP;
			break;
		case SLEEP:
			MODE_Stop();
		  SYS_S = IDLE;
			break;
		case ACTIVATE:
			MODE_Active();
		  SYS_S = ECG;
			TIM_1ms_Start();
		  TIM_1s_Start();
		  LTIM_1min_Start();
			break;
		default:
			break;
	}
		
}

/* * * * *  * * * * * * * * * * * *ADC collection runing polling * * * * * * * * * * * * * * * * */
static void ECG_Get_Data(void)
{
	if (HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	{
		ECG_Data = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
}
/* * * * *  * * * * * * * * * * * *spi1 collection runing(T + ACCE) polling * * * * * * * * * * * * * * * * */

static void ADXL362_Read(uint8_t * data, uint8_t bytesnumber)
{
	ACCR_Start();
	HAL_SPI_Transmit(&hspi1,data,2,20);
	HAL_SPI_Receive(&hspi1,data,bytesnumber,20);
	ACCR_End();
}

static void ADXL362_Write(uint8_t * data)
{
	ACCR_Start();
	HAL_SPI_Transmit(&hspi1,data,2,20);
	HAL_SPI_Transmit(&hspi1,data+2,1,20);
	ACCR_End();
}
    
static void ADXL362_GetRegisterValue(uint8_t * pReadData, uint8_t  registerAddress, uint8_t  bytesNumber)
{
    uint8_t buffer[bytesNumber + 2];
    uint8_t index = 0;
    
    buffer[0] = ADXL362_READ_REG;
    buffer[1] = registerAddress;

    ADXL362_Read(buffer, bytesNumber);
    for(index = 0; index < bytesNumber; index++)
    {
        pReadData[index] = buffer[index];
    }
}

static void ADXL362_WriteRegisterValue(uint8_t pSetData, uint8_t registerAddress)
{
    uint8_t buffer[3];
	  uint8_t index = 0;
	
    buffer[0] = ADXL362_WRITE_REG;
    buffer[1] = registerAddress;
	  buffer[2] = pSetData;

    ADXL362_Write(buffer);
}

static uint8_t ADXL362_Init()
{
	  uint8_t regValue = 0;
    uint8_t status = 0;
	  uint8_t mode = 0x00;
	  uint8_t range =0xd3; //1101 0011b +- 8g
	
	  // test if spi is well configured
    ADXL362_GetRegisterValue(&regValue, ADXL362_REG_DEVID_AD, 1);
    if(regValue == ADXL362_DEVICE_AD)
    {
			//config ADXL362 at stop mode and change measure range
			//mode = 0x00; // stop mode
			//ADXL362_WriteRegisterValue(mode, ADXL362_REG_POWER_CTL);
			//ADXL362_WriteRegisterValue(range, ADXL362_REG_FILTER_CTL);
			
			// config ADXL362 at measure mode
			mode = 0x02;//measure mode
			ADXL362_WriteRegisterValue(mode, ADXL362_REG_POWER_CTL);
			
			status   = 1;
		}
		
    return status;
}

 static void T_Get_Data(void)
{
	uint8_t regValue[2] = {0,0};
	uint16_t T_raw = 0;
	ADXL362_GetRegisterValue(regValue, ADXL362_REG_TEMP_L, 2);
  
	T_raw = ((uint16_t)regValue[1]<<8) + regValue[0]; 
	T_Data = (float)T_raw * 0.065;
	
	}

static void ACCE_Get_Data(void)
{
	uint8_t index = 0;
	uint8_t regValue[6] = {0,0,0,0,0,0};
	uint16_t ACCE_raw[3] = {0,0,0};
	
	ADXL362_GetRegisterValue(regValue, ADXL362_REG_XDATA_L, 6);
	
	ACCE_raw[0] = ((uint16_t)regValue[1] << 8) + regValue[0]; //x
	ACCE_raw[1] = ((uint16_t)regValue[3] << 8) + regValue[2]; //y
	ACCE_raw[2] = ((uint16_t)regValue[5] << 8) + regValue[4]; //z
	
	for (index = 0; index < 3; index++)
	{
		ACCE_Data[index] = (ACCE_raw[index] & 0x07FF ); //0000 0111 1111 1111;
	}
}

static void RTC_GetTime()
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  //sprintf((char*)showtime,"%02d:%02d:%02d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	RTC_Data[0] = sdatestructureget.Year;
	RTC_Data[1] = sdatestructureget.Month;
	RTC_Data[2] = sdatestructureget.Date;
	RTC_Data[3] = stimestructureget.Hours;
	RTC_Data[4] = stimestructureget.Minutes;
	RTC_Data[5] = stimestructureget.Seconds;
} 
static void SD_Run(void)
{
	FRESULT res;
	res = disk_initialize(0);
	//res = f_mount(&SDFatFs,"0:",1);
	if(res != FR_OK)
	{
		Error_Handler();
	}
	 
}
static void SD_Stop(void)
{
	
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  while (1)
	{
		  HAL_GPIO_TogglePin (LD4_GPIO_Port, LD4_Pin);
      HAL_Delay(1000);
	}
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
