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
#include "stm32f4xx_hal.h"
#include "print.h"
#include "xprintf.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

enum{
	WHO_AM_I=0x0f,
	CTRL1_XL=0x10,
	CTRL2_G=0x11,
	CTRL3_C=0x12,
	CTRL4_C=0x13,
	CTRL5_C=0x14,
	CTRL6_C=0x15,
	CTRL7_G=0x16,
	CTRL8_XL=0x17,
	CTRL9_XL=0x18,
	CTRL10_C=0x19,
	OUT_TEMP_L=0x20,
	OUT_TEMP_H=0x21,
	OUTX_L_G=0x22,
	OUTX_H_G=0x23,
	OUTY_L_G=0x24,
	OUTY_H_G=0x25,
	OUTZ_L_G=0x26,
	OUTZ_H_G=0x27,
	OUTX_L_XL=0x28,
	OUTX_H_XL=0x29,
	OUTY_L_XL=0x2A,
	OUTY_H_XL=0x2B,
	OUTZ_L_XL=0x2C,
	OUTZ_H_XL=0x2D
};
void USR_LSM6DSL_Set_Data(I2C_HandleTypeDef *hi2c,uint8_t add,uint8_t dat);
uint8_t USR_LSM6DSL_Get_Data(I2C_HandleTypeDef *hi2c,uint8_t add);
void USR_LSM6DSL_XL_Read_ALL(I2C_HandleTypeDef *hi2c,int16_t *dat);
void USR_LSM6DSL_GY_Read_ALL(I2C_HandleTypeDef *hi2c,int16_t *dat);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//if unuse semihosting, comment out "#define debug"
//#define debug
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
void uart_putc_wrap(uint8_t c){
	uart_putc(&huart2,c);
}
int main(void)
{

  /* USER CODE BEGIN 1 */
	int i=0;
	uint8_t tmp=0;
	int16_t accdata[3]={0};
	int16_t gyrdata[3]={0};
	int16_t gyravg[3]={0};

	float angspdata[3]={0.0};

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	xdev_out(uart_putc_wrap);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
//	xprintf("Hello,STM%d!\r\n",32);

	//LSM6DSL(ADD:0b1101011<<1)
//	HAL_I2C_Mem_Read(&hi2c1,0xD6,0x0f,I2C_MEMADD_SIZE_8BIT,&tmp,1,10);
	tmp=USR_LSM6DSL_Get_Data(&hi2c1,WHO_AM_I);
	USR_LSM6DSL_Set_Data(&hi2c1,CTRL1_XL,0b10000101);
	USR_LSM6DSL_Set_Data(&hi2c1,CTRL2_G,0b01100000);
	USR_LSM6DSL_Set_Data(&hi2c1,CTRL4_C,0b00000010);
	USR_LSM6DSL_Set_Data(&hi2c1,CTRL6_C,0b00000010);
	USR_LSM6DSL_Set_Data(&hi2c1,CTRL7_G,0b00000000);
	HAL_Delay(1000);
	
	for(i=0;i<10;i++){
		USR_LSM6DSL_GY_Read_ALL(&hi2c1,gyrdata);
		gyravg[0]+=gyrdata[0];
		gyravg[1]+=gyrdata[1];
		gyravg[2]+=gyrdata[2];
	}
	gyravg[0]/=10;
	gyravg[1]/=10;
	gyravg[2]/=10;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		USR_LSM6DSL_XL_Read_ALL(&hi2c1,accdata);
		USR_LSM6DSL_GY_Read_ALL(&hi2c1,gyrdata);
		if(!HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin)){
			for(i=0;i<10;i++){
					USR_LSM6DSL_GY_Read_ALL(&hi2c1,gyrdata);
					gyravg[0]+=gyrdata[0];
					gyravg[1]+=gyrdata[1];
					gyravg[2]+=gyrdata[2];
				}
			gyravg[0]/=10;
			gyravg[1]/=10;
			gyravg[2]/=10;
		}
		//gyro->angle speed
		angspdata[0]+=((float)(gyrdata[0]-gyravg[0]))*0.001/245.0;
		angspdata[1]+=((float)(gyrdata[1]-gyravg[1]))*0.001/245.0;
		angspdata[2]+=((float)(gyrdata[2]-gyravg[2]))*0.001/245.0;
	
	/* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		xprintf("%d %d %d "
				,(int16_t)(angspdata[0])
				,(int16_t)(angspdata[1])
				,(int16_t)(angspdata[2]));
//		xprintf("%d %d %d ",gyrdata[0],gyrdata[1],gyrdata[2]);	
//		xprintf("%d %d %d "
//			,gyrdata[0]-gyravg[0]
//			,gyrdata[1]-gyravg[1]
//			,gyrdata[2]-gyravg[2]);
//		xprintf("%d %d %d ",accdata[0],accdata[1],accdata[2]);
		xprintf("\n");
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		HAL_Delay(1);
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* I2C1 init function */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void USR_LSM6DSL_Set_Data(I2C_HandleTypeDef *hi2c,uint8_t add,uint8_t dat){
	HAL_I2C_Mem_Write(hi2c,0xD6,add,I2C_MEMADD_SIZE_8BIT,&dat,1,10);
}
uint8_t USR_LSM6DSL_Get_Data(I2C_HandleTypeDef *hi2c,uint8_t add){
	uint8_t tmp=0;
	HAL_I2C_Mem_Read(&hi2c1,0xD6,add,I2C_MEMADD_SIZE_8BIT,&tmp,1,10);
	return tmp;
}
void USR_LSM6DSL_XL_Read_ALL(I2C_HandleTypeDef *hi2c,int16_t *data){
	data[0] =USR_LSM6DSL_Get_Data(hi2c,OUTX_L_XL);
	data[0]|=USR_LSM6DSL_Get_Data(hi2c,OUTX_H_XL)<<8;
	data[1] =USR_LSM6DSL_Get_Data(hi2c,OUTY_L_XL);
	data[1]|=USR_LSM6DSL_Get_Data(hi2c,OUTY_H_XL)<<8;
	data[2] =USR_LSM6DSL_Get_Data(hi2c,OUTZ_L_XL);
	data[2]|=USR_LSM6DSL_Get_Data(hi2c,OUTZ_H_XL)<<8;
}
void USR_LSM6DSL_GY_Read_ALL(I2C_HandleTypeDef *hi2c,int16_t *data){
	data[0] =USR_LSM6DSL_Get_Data(hi2c,OUTX_L_G);
	data[0]|=USR_LSM6DSL_Get_Data(hi2c,OUTX_H_G)<<8;
	data[1] =USR_LSM6DSL_Get_Data(hi2c,OUTY_L_G);
	data[1]|=USR_LSM6DSL_Get_Data(hi2c,OUTY_H_G)<<8;
	data[2] =USR_LSM6DSL_Get_Data(hi2c,OUTZ_L_G);
	data[2]|=USR_LSM6DSL_Get_Data(hi2c,OUTZ_H_G)<<8;
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
  while(1) 
  {
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
