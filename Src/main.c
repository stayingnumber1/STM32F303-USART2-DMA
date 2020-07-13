/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "string.h"
#include "stdio.h"
#include "stm32f3xx_it.h"
  
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
    int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
    while((USART2->ISR&0X40)==0);  //循环发送,直到发送完毕
    USART2->TDR = (uint8_t) ch;
    return ch;
}
#endif

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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 static void MX_DMA_RX_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
	
	huart2.hdmarx=  &hdma_usart2_rx;
    __HAL_LINKDMA(&huart2,hdmarx,hdma_usart2_rx);                   //将DMA与USART2联系起来(发送DMA)
    
    //Tx DMA配置
    hdma_usart2_rx.Instance=DMA1_Channel6;                          //数据流选择
    hdma_usart2_rx.Init.Direction=DMA_PERIPH_TO_MEMORY;             //存储器到外设
    hdma_usart2_rx.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    hdma_usart2_rx.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    hdma_usart2_rx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    hdma_usart2_rx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    hdma_usart2_rx.Init.Mode=DMA_NORMAL;                            //外设普通模式
    hdma_usart2_rx.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
	
          
    //DMA_SetConfig(&hdma_usart2_rx, (uint32_t)USART2->RDR, (uint32_t)recv_buffer, 10);
    //HAL_DMA_DeInit(&hdma_usart2_rx);   
    HAL_DMA_Init(&hdma_usart2_rx);	
	
	__HAL_DMA_ENABLE_IT(&hdma_usart2_rx,DMA_IT_TC);
	__HAL_UART_ENABLE_IT(&huart2,UART_DMA_RX_ENABLE);

	
	

	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	 //hdma_usart2_rx.Instance->CCR |= DMA_CCR_EN;
	__HAL_DMA_ENABLE(&hdma_usart2_rx);	 

}
/* USER CODE END 0 */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 static void MX_DMA_TX_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
	
	huart2.hdmatx=  &hdma_usart2_tx;
    __HAL_LINKDMA(&huart2,hdmatx,hdma_usart2_tx);                   //将DMA与USART2联系起来(发送DMA)
    
    //Tx DMA配置
    hdma_usart2_tx.Instance=DMA1_Channel7;                          //数据流选择
    hdma_usart2_tx.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
    hdma_usart2_tx.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    hdma_usart2_tx.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    hdma_usart2_tx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    hdma_usart2_tx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    hdma_usart2_tx.Init.Mode=DMA_NORMAL;                            //外设普通模式
    hdma_usart2_tx.Init.Priority=DMA_PRIORITY_MEDIUM;               //中等优先级
	
          
    //DMA_SetConfig(&hdma_usart2_rx, (uint32_t)USART2->RDR, (uint32_t)recv_buffer, 10);
    //HAL_DMA_DeInit(&hdma_usart2_rx);   
    HAL_DMA_Init(&hdma_usart2_tx);	
	
	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx,DMA_IT_TC);
	__HAL_UART_ENABLE_IT(&huart2,UART_DMA_TX_ENABLE);

	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	 //hdma_usart2_rx.Instance->CCR |= DMA_CCR_EN;
	__HAL_DMA_ENABLE(&hdma_usart2_tx);	

}


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart ==&huart2)
	{
		
		for(int i=0;i<10;i++)
		{
		  printf("%c\r\n",recv_buffer[i]);
	    }
		printf("\r\n rx call back \r\n");
        __HAL_UART_DISABLE(&huart2);	
        __HAL_UART_ENABLE(&huart2);
		HAL_Delay(10);
		
		HAL_UART_Transmit_DMA(&huart2,(uint8_t *)send_buffer,5);
		//HAL_UART_Receive_DMA(&huart2,(uint8_t *)recv_buffer,10);

       		
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart == &huart2)
	{
		__HAL_UART_DISABLE(&huart2);	
        __HAL_UART_ENABLE(&huart2);
//		uint8_t send_cargo=send_buffer[1];
		send_buffer[1]=send_buffer[1]+1;
	    printf("\r\n tx call back \r\n");
	}
 
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  uint8_t cargo[10];
  uint8_t *point_cargo=cargo;
  

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
  /* USER CODE BEGIN 2 */
  
  MX_DMA_RX_Init();
  MX_DMA_TX_Init(); 
  printf("gooooood \r\n");
  //HAL_DMA_Start_IT(&hdma_usart2_rx,(uint32_t)USART2->RDR,(uint32_t)recv_buffer,10);

  HAL_UART_Receive_DMA(&huart2,(uint8_t *)recv_buffer,10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  //huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  //huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  //huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
	
  	
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* USER CODE BEGIN USART2_Init 2 */
   __HAL_UART_ENABLE(&huart2);
  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
