/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*enum opcode {
	first_temperature = 1,
	second_pressure = 2,
	third_altitude = 3
};*/

short AC1, AC2, AC3, B1, B2, MB, MC, MD;
unsigned short AC4, AC5, AC6;
long X1, X2, B5, T;
long X1, X2, X3, B3, B6, P;
unsigned long B4, B7;
short oss = 1;
long UT, UP;
int temperature, pressure, reference_pressure, altitude;

char gps_data[500];
char Latitude[9], Longitude[10];

char all_data[12] = {'0'};
char sicaklik[6] = {'0'};
char basinc[8] = {'0'};
char yukseklik[8] = {'0'};

uint8_t data_length = 0;
int sicaklik_counter = 0, basinc_counter = 0, yukseklik_counter = 0;

uint8_t UART_Status;

void Callibration();
void writeReg(uint8_t reg, uint8_t val);
uint8_t readReg(uint8_t reg);
long Uncompansate_Temp();
long Get_Temp();
long Uncompansate_Pressure();
long Get_Pressure();
void Reference_Altitude();
long Get_Altitude();
void Get_GPS();
void Transmit_Data();

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Callibration();
  HAL_Delay(5);
  Reference_Altitude();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  temperature = Get_Temp();
	  pressure = Get_Pressure();
	  altitude = Get_Altitude();
	  Get_GPS();
	  HAL_Delay(100);
	  Transmit_Data();
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void Callibration()
{
	AC1 = (readReg(0xAA) << 8 | readReg(0xAB));
	AC2 = (readReg(0xAC) << 8 | readReg(0xAD));
	AC3 = (readReg(0xAE) << 8 | readReg(0xAF));
	AC4 = (readReg(0xB0) << 8 | readReg(0xB1));
	AC5 = (readReg(0xB2) << 8 | readReg(0xB3));
	AC6 = (readReg(0xB4) << 8 | readReg(0xB5));
	B1 = (readReg(0xB6) << 8 | readReg(0xB7));
	B2 = (readReg(0xB8) << 8 | readReg(0xB9));
	MB = (readReg(0xBA) << 8 | readReg(0xBB));
	MC = (readReg(0xBC) << 8 | readReg(0xBD));
	MD = (readReg(0xBE) << 8 | readReg(0xBF));
}

uint8_t readReg(uint8_t reg)
{
	uint8_t retVal = 0;
	HAL_I2C_Master_Transmit(&hi2c1, 0xEF, &reg , 1, 10000);
	HAL_I2C_Master_Receive(&hi2c1, 0xEF, &retVal, 1, 10000);
	return retVal;
}

long Uncompansate_Temp()
{
	uint8_t val1, val2;
	uint8_t reg_val = 0x2E;

	HAL_I2C_Mem_Write(&hi2c1, 0xEF, 0xF4, 1, &reg_val, 1, 1000);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(&hi2c1, 0xEF, 0xF6, 1, &val1, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, 0xEF, 0xF7, 1, &val2, 1, 1000);

	return ((val1<<8) | val2);
}

long Get_Temp()
{
	UT = Uncompansate_Temp();

	X1 = ((UT - AC6) * AC5) / pow(2,15);
	X2 = (MC * pow(2,11)) / (X1 + MD);
	B5 = X1 + X2;
	T = (B5 + 8) / ((float)160);

	return T;
}

long Uncompansate_Pressure()
{
	uint8_t val1, val2, val3;
	uint8_t reg_val = 0x34;

	HAL_I2C_Mem_Write(&hi2c1, 0xEF, (0xF4+(oss<<6)), 1, &reg_val, 1, 1000);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(&hi2c1, 0xEF, 0xF6, 1, &val1, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, 0xEF, 0xF7, 1, &val2, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, 0xEF, 0xF8, 1, &val3, 1, 1000);

	return ((val1<<16 | val2<<8 | val3) >> (8-oss));
}

// Pressure value is calculated in term of hPa
long Get_Pressure()
{
	UP = Uncompansate_Pressure();

	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 / pow(2,12))) / pow(2,11);
	X2 = AC2 * B6 / pow(2,11);
	X3 = X1 + X2;
	B3 = ((((AC1 * 4) + X3) << oss) + 2) / 4;
	X1 = (AC3 * B6) / pow(2,13);
	X2 = (B1 * ((B6 * B6) / pow(2,12))) / pow(2,16);
	X3 = ((X1 + X2) + 2) / pow(2,2);
	B4 = (AC4 * (unsigned long)(X3 + 32768)) / pow(2,15);
	B7 = ((unsigned long)UP - B3) * (50000 >> oss);
	if(B7 < 0x80000000)
		P = (B7 * 2) / B4;
	else
		P = (B7 / B4) * 2;
	X1 = (P / pow(2,8) * (P / pow(2,8)));
	X1 = (X1 * 3038) / pow(2,16);
	X2 = (-7357 * P) / pow(2,16);
	P = P + (X1 + X2 + 3791) / 16;

	return P / 100;
}

void Reference_Altitude()
{
	Get_Temp();
	reference_pressure = Get_Pressure();
}

long Get_Altitude()
{
	long A;

	A = 44330 * (1 - pow((pressure / reference_pressure), (1 / 5.255)));

	return A;
}

void Get_GPS()
{
	char *strFindptr;
	char temp;
	//const char key_word[7] = "$GNGLL,";		// strstr() fonksiyonu bununla calismadi ???

	UART_Status = HAL_UART_Receive(&huart1,(uint8_t *)gps_data,500,1000);

	strFindptr = strstr(gps_data, "$GNGLL,");

	if(strFindptr != 0)	{
		//(3)Extract Latitude
		strFindptr = strstr(&strFindptr[1],",");
		if(strFindptr[1]!=',')
		{
			memcpy(Latitude,&strFindptr[1],9);
			temp = Latitude[4];
			Latitude[4] = Latitude[3];
			Latitude[3] = Latitude[2];
			Latitude[2] = temp;
		}
		else
		{
			sprintf(Latitude,"--.-----");
		}
		//(4)Extract Longitude
		strFindptr = strstr(&strFindptr[1],",");
		strFindptr = strstr(&strFindptr[1],",");
		if(strFindptr[1]!=',')
		{
			memcpy(Longitude,&strFindptr[1],10);
			temp = Latitude[4];
			Longitude[4] = Longitude[3];
			Longitude[3] = Longitude[2];
			Longitude[2] = temp;
		}
		else
		{
			sprintf(Longitude,"--.-----");
		}
	}
}

void Transmit_Data()
{
	data_length = 0;
	sicaklik_counter = 0;
	basinc_counter = 0;
	yukseklik_counter = 0;

	// Opcode
	sicaklik[0] = 1;
	basinc[0] = 2;
	yukseklik[0] = 3;

	// Value assignment
	sprintf(sicaklik+2,"%d",temperature);
	sprintf(basinc+2,"%d",pressure);
	sprintf(yukseklik+2,"%d",altitude);

	// Determining valid data length
	while(sicaklik[2+sicaklik_counter] != '\0'){
		data_length++;
		sicaklik_counter++;
	}

	while(basinc[2+basinc_counter] != '\0'){
		data_length++;
		basinc_counter++;
	}

	while(yukseklik[2+yukseklik_counter] != '\0'){
		data_length++;
		yukseklik_counter++;
	}

	// Assign valid length to array
	memcpy(sicaklik+1,&sicaklik_counter,1);
	memcpy(basinc+1,&basinc_counter,1);
	memcpy(yukseklik+1,&yukseklik_counter,1);

	/*memcpy(all_data,sicaklik,sicaklik_counter);
	memcpy((all_data+sicaklik_counter),basinc,basinc_counter);
	memcpy((all_data+sicaklik_counter+basinc_counter),yukseklik,yukseklik_counter);*/

	HAL_UART_Transmit(&huart2,(uint8_t *)sicaklik,sicaklik_counter+2,100);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart2,(uint8_t *)basinc,basinc_counter+2,100);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart2,(uint8_t *)yukseklik,yukseklik_counter+2,100);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart2,(uint8_t *)basinc,basinc_counter+2,100);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart2,(uint8_t *)yukseklik,yukseklik_counter+2,100);
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
