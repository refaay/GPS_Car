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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "dwt_stm32_delay.h"
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

//Config
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

//Basic Movement
void TEST_DAGU_MOVEMENT(void);

//Ultrasonic
void UltraSonic(void);
void moveRight(void);
void accelerate(void);

//GPS
void TEST_GPS(void);
void parse_gps(void);
double haversine(double,double,double,double);
double to_radians(double);

//IMU
void TEST_IMU(void);
void MoveForward(void);
void HandleIMU(void);
void TiltRight(void);
void TiltLeft(void);

//Color
void TEST_COLOR(void);
void COLOR_PARSE(void);
//Variables
//US
double distance;
uint8_t time1, time2, time_final,US_state;

//GPS
char gps_output[4];
double longitudeA = -1,latitudeA = -1,temp;
int i,dotlocation,cnt;
char A[3],B[10];
char word[] = "GPRMC";
char gps[900];
char gps_state = 'V';
char *pch;
char *gpgllstr;
char testMessage[9] = "NotFound";
char gps_distance[10];
double gps_distance_f;

//For Dagu Movement
char right = 0xC2, left= 0xCA;
char speed = 40;
char msg[20];

//IMU
bool FOUND_REF_POINT = false;
char IMU_MSG[1];
char temp2 = 'F';
int IMU_MSG_INT, IMU_MSG_CURRENT, DIFF_ALLOWED = 10, SPEED_LEFT,SPEED_RIGHT,tempNum;
int ADD_DIFF = 50;

//Color sensor
char color[25];
char col_out[11];
char word2[] = "$";

int main(void)
{

  HAL_Init();
  DWT_Delay_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	
  while (1)
  {
		
		//UltraSonic();
		
		
		
  }
}

void UltraSonic(){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
		DWT_Delay_us(10);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		
		do {
			US_state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
		}while(US_state == 0);
		
		time1 = HAL_GetTick();
		
		do {
			US_state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
		}while(US_state == 1);
		
		time2 = HAL_GetTick();
		
		time_final = time2 - time1;
		if (time_final!=0)
			distance=time_final*17.0;
	
		if (distance<=34)	moveRight();
		else accelerate();
}
void moveRight(){
			right=0xC8; left=0xC0;
			speed = 80;
			sprintf(msg,"%c%c%c%c",right,speed,left,speed);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,4,1);
			HAL_Delay(1500);
			right = 0xC1, left = 0xCA;
			speed = 40;
			sprintf(msg,"%c%c%c%c",right,speed,left,speed);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,4,1);
			HAL_Delay(1000);
}
void accelerate(){
		right = 0xC1, left= 0xC9;
		speed = 50;
		sprintf(msg,"%c%c%c%c",right,speed,left,speed);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,4,1);
}
void TEST_GPS(){
	HAL_UART_Receive(&huart1,(uint8_t *)gps,900,1000);
	parse_gps();
	sprintf(gps_output," (%lf, %lf) ",latitudeA,longitudeA);
	HAL_UART_Transmit(&huart2,(uint8_t *)gps_output,24,1000);
	HAL_Delay(2000);
	
	gps_distance_f = haversine(latitudeA,longitudeA,50.0,50.0);
	sprintf(gps_output,"D: %lf ", gps_distance_f);
	HAL_UART_Transmit(&huart2,(uint8_t *)gps_output,12,1000);
	HAL_Delay(2000);
}
double haversine(double latitude1, double latitude2, double longitude1, double longitude2){
	double temp1 = pow(sin(to_radians(latitude2 - latitude1)) / 2.0, 2.0);
	double temp2 = cos(to_radians(latitude1)) * cos(to_radians(latitude2)) * pow((to_radians(longitude2 - longitude1)) / 2.0, 2.0);
	double a = temp1 + temp2;

	double temp3 = atan2(sqrt(a), sqrt(1 - a));
	double c = 2.0 * temp3;

	double temp4 = 6371e3;
	double d = temp4 * c;

	return d;
}

double to_radians(double degrees){
	return degrees * (3.1415926535/180.0);
}


void parse_gps(){
		memset( A,'\0', sizeof(char)*3 );
		memset( B,'\0', sizeof(char)*10 );
		cnt = 0;
		gpgllstr = strstr(gps, word);
		if(gpgllstr){
			gpgllstr += 6;
			pch = strtok(gpgllstr,",");
			while (pch != NULL && ++cnt <= 6){
				int totalLength = strlen(pch);
				
				if(cnt == 3 || cnt == 5){
					if(cnt == 3) dotlocation = 4;
					else dotlocation = 5;
					
					strncpy(A,pch,dotlocation-2);
					strncpy(B,pch+(dotlocation-2)	,totalLength-(dotlocation-2));
					
					 temp = atof(A)+atof(B)/60.0;
					if(cnt ==3) latitudeA = temp;
					else longitudeA = temp;
				}
				pch = strtok (NULL, ",");
			}
		}
}
void TEST_IMU(){
	if(!FOUND_REF_POINT){
			HAL_UART_Receive(&huart2,(uint8_t *)IMU_MSG,3,2000);
			tempNum = atoi(IMU_MSG);
			if(tempNum >= 900) IMU_MSG_INT =tempNum - 900;
			else IMU_MSG_INT = tempNum;
			FOUND_REF_POINT = true;
		}else{
				HAL_UART_Receive(&huart2,(uint8_t *)IMU_MSG,3,2000);
				tempNum = atoi(IMU_MSG);
				if(tempNum >= 900) IMU_MSG_CURRENT =tempNum - 900;
				else IMU_MSG_CURRENT = tempNum;
				//sprintf(IMU_MSG,"%d",IMU_MSG_CURRENT);
				//HAL_UART_Transmit(&huart2,(uint8_t *)IMU_MSG,3,2000);
				//HAL_UART_Transmit(&huart2,(uint8_t *)" ",1,2000);
				HandleIMU();
		}
		//HAL_UART_Receive(&huart2,(uint8_t *)IMU_MSG,3,2000);
		//HAL_UART_Transmit(&huart2,(uint8_t *)IMU_MSG,3,2000);
}
void HandleIMU(void){
	if(abs(IMU_MSG_CURRENT - IMU_MSG_INT) <= DIFF_ALLOWED) MoveForward();
	else if(IMU_MSG_CURRENT < IMU_MSG_INT) TiltRight();
	else TiltLeft();
}
void MoveForward(void){
		char msg[20];
		char right = 0xC1, left= 0xC9;
		SPEED_RIGHT = 30;
		SPEED_LEFT = 30;
		sprintf(msg,"%c%c%c%c",right,SPEED_LEFT,left,SPEED_RIGHT);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,4,1);
}
void TiltLeft(){
		char msg[20];
		char right = 0xC1, left= 0xC9;
		SPEED_RIGHT += DIFF_ALLOWED;
		sprintf(msg,"%c%c%c%c",right,SPEED_LEFT,left,SPEED_RIGHT);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,4,1);
}

void TiltRight(){
		char msg[20];
		char right = 0xC1, left= 0xC9;
		SPEED_LEFT += DIFF_ALLOWED;
		sprintf(msg,"%c%c%c%c",right,SPEED_LEFT,left,SPEED_RIGHT);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,4,1);
}
void TEST_COLOR(){
	HAL_UART_Receive(&huart1,(uint8_t *) color,25,500);
  //strncpy(col_out,color+1,9);
	COLOR_PARSE();
  HAL_UART_Transmit(&huart2,(uint8_t *) col_out,11,500);
  HAL_UART_Transmit(&huart2,(uint8_t *) " ",1,500);
}
void COLOR_PARSE(void){
	memset( col_out,'\0', sizeof(char)*11);
	gpgllstr = strstr(color, word2);
	if(gpgllstr){
		strncpy(col_out,gpgllstr,10);
	}
}
void SystemClock_Config(void){

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* USART1 init function */
static void MX_USART1_UART_Init(void){
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 4800; //change when using gps to 9600 else law color 4800 else 19200
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void){

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
    _Error_Handler(__FILE__, __LINE__);
  }

}
static void MX_GPIO_Init(void){

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
void _Error_Handler(char *file, int line){
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