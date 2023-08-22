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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "buzzer.h"
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	EXAMPLE_BUZZER_START,
	EXAMPLE_BUZZER_STOP,
	EXAMPLE_BUZZER_RINGTONE,
	EXAMPLE_BUZZER_NO_LOOP,
	EXAMPLE_BUZZER_LOOP
}example_buzzer_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MOTOR_send cmd[2];   //以全局变量声明电机控制结构体和电机数据结构体，方便在故障时通过debug查看变量值
MOTOR_recv data[2];
#define USB_MAX_RECEIVE_LEN 64
uint8_t rxData[USB_MAX_RECEIVE_LEN] = {0};
uint32_t rxLen = 0;

#define PWM_TIM		&htim12
#define PWM_CHN		TIM_CHANNEL_1
#ifndef USE_STATIC_MEM_ALLOCATION
buzzer_t mBuzzer;
#endif
buzzer_t *Buzzer;
example_buzzer_e example = EXAMPLE_BUZZER_STOP;
uint8_t nextPattern;

extern volatile float VELX[IMU_SAMPLE_FRAME], ANGX[IMU_SAMPLE_FRAME];
static uint32_t timer1_count;
static uint32_t gait50_count[GAIT_SAMPLE_FRAME];
static uint32_t gait85_count[GAIT_SAMPLE_FRAME];
static uint32_t GAIT_PERIOD;
extern DebugTypeDef usbDebug;	

uint8_t uart6Data = 0;
uint8_t ucRxBuffer6[30] = {0};
static int LED_ON[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void pwm_start(){
	HAL_TIM_PWM_Start(PWM_TIM, PWM_CHN);
}

void pwm_set_dc(uint32_t dc){
	uint32_t arr, comp;

	arr = __HAL_TIM_GET_AUTORELOAD(PWM_TIM);
	comp = dc*arr/100;
	__HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CHN, comp);
}

void pwm_set_freq(uint32_t freq){
	uint32_t psc, arr, sys;

	if (freq > 0){
		sys = HAL_RCC_GetSysClockFreq();
		arr = __HAL_TIM_GET_AUTORELOAD(PWM_TIM);
		psc = (sys/(freq*(arr+1)))-1;
		__HAL_TIM_SET_PRESCALER(PWM_TIM, psc);
		pwm_set_dc(50);
	}
	else
		__HAL_TIM_SET_COMPARE(PWM_TIM, PWM_CHN, 0);
}



void buzzer_end_callback(buzzer_t *buzzer){
	nextPattern = 1;
}

void motor_init(int i){
	if(i != 0 && i != 1) return;
	cmd[i].id=i; 			
	cmd[i].mode=1;
	cmd[i].T=0;
	cmd[i].W=0;
	cmd[i].Pos=0;
	cmd[i].K_P=0;
	cmd[i].K_W=0;
}

void motor_stop(int i){
	if(i != 0 && i != 1) return;
	cmd[i].mode=0;
}

void motor_zeroTorque(int i){
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = 0.0; 
	cmd[i].Pos = 0.0; 
	cmd[i].K_P = 0.0; 
	cmd[i].K_W = 0.0; 
	if(SERVO_Send_recv(&cmd[i], &data[i])) //将控制指令发送给电机，同时接收返回值
		LED_ON[7] = 100; //LED1点亮
}

void motor_damping(int i){
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = 0.0; 
	cmd[i].Pos = 0.0; 
	cmd[i].K_P = 0.0; 
	cmd[i].K_W = 0.02; 
	if(SERVO_Send_recv(&cmd[i], &data[i])) //将控制指令发送给电机，同时接收返回值
		LED_ON[7] = 100; //LED1点亮
}

void motor_profilePosition(int i, float pos_rad){
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = 0.0; 
	cmd[i].Pos = pos_rad*6.33f; 
	cmd[i].K_P = 0.02f; 
	cmd[i].K_W = 0.0; 
	if(SERVO_Send_recv(&cmd[i], &data[i])) //将控制指令发送给电机，同时接收返回值
		LED_ON[7] = 100; //LED1点亮
}

void motor_profileVelocity(int i, float vel_rad){
	if(i != 0 && i != 1) return;
	cmd[i].T = 0.0; 
	cmd[i].W = vel_rad*6.33f; 
	cmd[i].Pos = 0; 
	cmd[i].K_P = 0; 
	cmd[i].K_W = 0.05f; 
	if(SERVO_Send_recv(&cmd[i], &data[i])) //将控制指令发送给电机，同时接收返回值
		LED_ON[7] = 100; //LED1点亮
}

void gait_phase_detect(){
	
	uint32_t angmax1_count = 0;
	uint32_t angmin1_count = 0;
	uint32_t angmax2_count = 0;
	uint32_t angmin2_count = 0;
	
	uint32_t velplus1_count = 0;
	uint32_t velminus1_count = 0;
	uint32_t velplus2_count = 0;
	uint32_t velminus2_count = 0;
	
	for(uint32_t i=0; i< MEDIAN; i++){
		if(ANGX[i] < ANGX[MEDIAN]) angmax1_count++;
		else if(ANGX[i] > ANGX[MEDIAN]) angmin1_count++;
		
		if(VELX[i] < 0) velminus1_count++;
		else if(VELX[i] > 0) velplus1_count++;
	}
	for(uint32_t i=MEDIAN; i< IMU_SAMPLE_FRAME; i++){
		if(ANGX[i] < ANGX[MEDIAN]) angmax2_count++;
		else if(ANGX[i] > ANGX[MEDIAN]) angmin2_count++;
		
		if(VELX[i] < 0) velminus2_count++;
		else if(VELX[i] > 0) velplus2_count++;
	}
	
	if(angmin1_count > THRESHOLD && angmin2_count > THRESHOLD && velminus1_count > THRESHOLD && velplus2_count > THRESHOLD)
	{		
		for(int i=0;i<GAIT_SAMPLE_FRAME-1;i++) {gait50_count[i]=gait50_count[i+1];}
		gait50_count[GAIT_SAMPLE_FRAME-1] = timer1_count - MEDIAN;
		LED_ON[0] = 100;
		debugPrintf("gait50: %d, %d, %d\r\n",gait50_count[0],gait50_count[1],gait50_count[2]);
	}
	if(angmax1_count > THRESHOLD && angmax2_count > THRESHOLD && velplus1_count > THRESHOLD && velminus2_count > THRESHOLD)
	{
		for(int i=0;i<GAIT_SAMPLE_FRAME-1;i++) {gait85_count[i]=gait85_count[i+1];}
		gait85_count[GAIT_SAMPLE_FRAME-1] = timer1_count - MEDIAN;
		LED_ON[1] = 100;
		debugPrintf("gait85: %d, %d, %d\r\n",gait85_count[0],gait85_count[1],gait85_count[2]);
	}
}

void gait_update_period(){
	
	GAIT_PERIOD = (gait50_count[2] - gait50_count[0])/2;
}











void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == (&htim9)){	
		buzzer_interrupt(Buzzer);
	}
	
	if (htim == (&htim1))
	{			
//		gait_phase_detect();
//		gait_update_period();
//		uint32_t assist_start = gait50_count[GAIT_SAMPLE_FRAME-1] + 0.7*GAIT_PERIOD;
//		uint32_t assist_stop = gait50_count[GAIT_SAMPLE_FRAME-1] + 1.2*GAIT_PERIOD;
//		if(timer1_count >= assist_start && timer1_count < assist_stop)
//			motor_profileVelocity(0.5);
//		else
//			motor_zeroTorque();
		
		timer1_count++;
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
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_TIM1_Init();
  MX_UART8_Init();
  MX_TIM12_Init();
  MX_TIM9_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(1000);
//	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
	
	initDebugBuf();
//	HAL_UART_Receive_IT(&huart8, (uint8_t *)&ucData_L, 1);
	HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, POWER2_CTRL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH, POWER3_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOH, POWER4_CTRL_Pin, GPIO_PIN_SET);

	HAL_TIM_Base_Start_IT(&htim1);
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

	HAL_UART_Receive_DMA(&huart7, ucData_L, 1);
	

	uartPrintf("StartSTM32\r\n");
	
	cmd[0].id=0; 			//给电机控制指令结构体赋值
	cmd[0].mode=1;
	cmd[0].T=0;
	cmd[0].W=2;
	cmd[0].Pos=0;
	cmd[0].K_P=0;
	cmd[0].K_W=0.05;
	cmd[1].id=1; 			//给电机控制指令结构体赋值
	cmd[1].mode=1;
	cmd[1].T=0;
	cmd[1].W=4;
	cmd[1].Pos=0;
	cmd[1].K_P=0;
	cmd[1].K_W=0.05;

//	cmd.T=0;
//	cmd.W=0;
//	cmd.Pos=0;
//	cmd.K_P=0;
//	cmd.K_W=0;
	unsigned int count = 0;
	
	Buzzer = &mBuzzer;
	Buzzer->interruptMs = 10; // interrupt will be triggered every 10ms
  Buzzer->fnx.pwmOut = pwm_set_freq; // pass set frequency function
	buzzer_init(Buzzer);
  HAL_TIM_Base_Start_IT(&htim9);
  pwm_start();
	nextPattern = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		sendAllDebugMsg(&usbDebug);
//		beep_set_times(1);
//		beep_set_tick(0);
		SERVO_Send_recv(&cmd[0], &data[0]);	//将控制指令发送给电机，同时接收返回值
		HAL_Delay(10);
		SERVO_Send_recv(&cmd[1], &data[1]);	//将控制指令发送给电机，同时接收返回值
//		uartPrintf("count: %d\r\n",count);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
		count++;	//每0.2秒循环一次，3秒后发送指令使电机停止转动
		if(count % 6 == 0)
		{
			cmd[0].W=0;
			cmd[0].K_W=0;
			cmd[1].W=0;
			cmd[1].K_W=0;
			HAL_Delay(400);
		}
		else{
			cmd[0].W=2;
			cmd[0].K_W=0.05;
			cmd[1].W=2;
			cmd[1].K_W=0.05;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		



//  SEGGER_RTT_printf(0, "printf Test: %%c,         'S' : %c.\r\n", 'S');
//  SEGGER_RTT_printf(0, "printf Test: %%5c,        'E' : %5c.\r\n", 'E');
//  SEGGER_RTT_printf(0, "printf Test: %%-5c,       'G' : %-5c.\r\n", 'G');
//  SEGGER_RTT_printf(0, "printf Test: %%5.3c,      'G' : %-5c.\r\n", 'G');
//  SEGGER_RTT_printf(0, "printf Test: %%.3c,       'E' : %-5c.\r\n", 'E');
//  SEGGER_RTT_printf(0, "printf Test: %%c,         'R' : %c.\r\n", 'R');
//	HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	if (htim == (&htim1))
//	{		
//		static unsigned int count = 0;
////		uartPrintf("count: %d\r\n",count);
////		sprintf(uartBuf,"count: %d\r\n",count);
////		HAL_UART_Transmit(&huart7,(uint8_t *)uartBuf, (COUNTOF(uartBuf)-1), 55);
////		SERVO_Send_recv(&cmd, &data);	//将控制指令发送给电机，同时接收返回值
////		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
////		count++;	//每0.5秒循环一次，3秒后发送指令使电机停止转动
////		if(count%10 == 0)
////		{
////			cmd.W=0;
////			cmd.K_W=0;
////			count=0;
////		}
//	
//	
//	}
//	
//	
//	
//}


static int sys_count = 0;
void sysTickTask(void)
{
	sys_count++;
	for(int i=0; i<8; i++){
		LED_ON[i]--;
		if(LED_ON[i] > 0) HAL_GPIO_WritePin(GPIOG, (uint16_t)2<<i, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(GPIOG, (uint16_t)2<<i, GPIO_PIN_SET);
		if(LED_ON[i] < 0) LED_ON[i] = 0;
	}
	if(sys_count%1000 == 0){
		HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
		debugPrintf("sys_time: %.3f sec\r\n",sys_count/1000.f);	
				
	}
	if(sys_count%5000 == 0){
//		buzzer_start(Buzzer, 2500, 500, BUZZER_LOOP_OFF);
		debugPrintf("buzzer_start\r\n");	
	}
	
	
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
