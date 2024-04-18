/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h" 
#include "string.h"
#include "stdarg.h"
//#include "mpu6050.h"
#include "driver_ms5837_basic.h"
#include "mpu6050/driver_mpu6050_basic.h"
#include "mpu6050/driver_mpu6050_dmp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENDLINE_CHAR ';'
#define LOOP_TIME 5
#define LOOP_TIME2 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sendPiMessage(const char* fmt, ...);
void sendLoraMessage(const char* fmt, ...);
void solve_Command(char* rec_msg);
void processMsg(char* msg, char* streamingMsg);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*UartHandle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t pwm_val = 50;	// 初始转速
uint16_t pwm_val_arr[6] = {50,50,50,50,50,50};
uint8_t lora_rec[50] = {""};
uint8_t pi_rec[50] = {""};
uint8_t lora_buff[1];
uint8_t pi_buff[1];
int RxLine = 0;
int sendToLora = 0;
uint8_t read_pressure = 0;
extern float pressure;
uint8_t start = 0;	// start=0时不能获取motion和pressure

// mpu6050
uint8_t res;
uint32_t i;
uint32_t times;
float g[3];
float dps[3];
int16_t acc_buff[3];
int16_t gyro_buff[3];
int32_t q[4];
float pitch;
float roll;
float yaw;
uint16_t l;
float degrees;
mpu6050_address_t addr;
uint8_t read_motion = 0;
uint8_t read_motion_dmp = 0;



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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	// PWM
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_val);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_val);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm_val);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_val);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_val);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_val);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	// MPU6050初始化
	addr = MPU6050_ADDRESS_AD0_LOW;
	res = mpu6050_basic_init(addr);
	if (res != 0)
	{
    sendPiMessage("MPU6050 Not Find");
	}else{
		sendPiMessage("find mpu6050");
		
	}
	
	
	// LED初始亮灭
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	uint8_t led1_count = 0;
	
	// 初始化接收消息中断
	HAL_UART_Receive_IT(&huart2, (uint8_t *)lora_buff, 1);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)pi_buff, 1);
	
	// 初始化压力传感器
	uint8_t res;
	uint32_t i;
	uint32_t times;
	res = ms5837_basic_init(MS5837_TYPE_02BA21);
	if(res == 0){
		sendPiMessage("find ms5837");
	}else{
		sendPiMessage("failed to find ms5837");
	}
	start=1;
  while (1)
  {
		// 调试的LED

		led1_count++;
		if(led1_count > 50){
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led1_count = 0;
		}

		if(read_pressure == 1){
			// 获取压力
			float temperature_c = -1;
			float pressure_mbar;
			uint8_t res;
    /* read data */
			res = ms5837_basic_read(&temperature_c, &pressure_mbar);
			if(res == 0){
				sendPiMessage("data pressure %f", pressure_mbar);
			}else{
				sendPiMessage("error Failed to Read Pressure: Unknown error");
			}
			read_pressure = 0;
		}
		
		if(read_motion == 1){
			if(mpu6050_basic_read(g, dps)==0){
				sendPiMessage("data motion %f %f %f %f %f %f", g[0], g[1], g[2], dps[0], dps[1], dps[2]);
			}else{
				sendPiMessage("error Failed to Read Motion: Unknown error");
			}
			read_motion = 0;
		}
		if(read_motion_dmp == 1){
			if(mpu6050_dmp_read_all(&acc_buff, &g, &gyro_buff, &dps, &q, &pitch, &roll, &yaw, &l)){
				sendPiMessage("data motion %f %f %f", pitch, roll, yaw);
			}else{
				sendPiMessage("error Failed to Read Motion: Unknown error");
			}
			read_motion_dmp = 0;
		}
		HAL_Delay(10);
		//uint8_t rec[50] = {""};
		//HAL_UART_Receive(&huart1,rec, 50, LOOP_TIME);
		
		//processMsg(pi_rec, (char*)rec);	// 执行指令
		
		//sendLoraMessage("hello lora");
		// 接收lora的指令
		//uint8_t rec2[50] = {""};
		//HAL_UART_Receive(&huart2,rec2, 50, LOOP_TIME2);
		//processMsg(lora_rec, (char*)rec2);
		
		
		
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

/* USER CODE BEGIN 4 */
// 读取消息的中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*UartHandle)
{
		if(UartHandle == &huart2){
			if(lora_buff[0]==ENDLINE_CHAR)            //接收结束标志位，这个数据可以自定义，根据实际需求，这里只做示例使用，不一定是0xff
			{
				sendToLora = 1;
				solve_Command((char*)lora_rec); 		
				sendToLora = 0;
				memset(lora_rec,0,sizeof(lora_rec));  //清空缓存数组
				RxLine=0;  //清空接收长度
			}else{
				RxLine++;                      //每接收到一个数据，进入回调数据长度加1
				lora_rec[RxLine-1]=lora_buff[0];  //把每次接收到的数据保存到缓存数组
			}
			
			lora_buff[0]=0;
			HAL_UART_Receive_IT(&huart2, (uint8_t *)lora_buff, 1); //每接收一个数据，就打开一次串口中断接收，否则只会接收一个数据就停止接收
		}else if(UartHandle == &huart1){
			// pi message
			if(pi_buff[0]==ENDLINE_CHAR)            //接收结束标志位，这个数据可以自定义，根据实际需求，这里只做示例使用，不一定是0xff
			{
				solve_Command((char*)pi_rec);                           
				memset(pi_rec,0,sizeof(pi_rec));  //清空缓存数组
				RxLine=0;  //清空接收长度
			}else{
				RxLine++;                      //每接收到一个数据，进入回调数据长度加1
				pi_rec[RxLine-1]=pi_buff[0];  //把每次接收到的数据保存到缓存数组
			}
			
			pi_buff[0]=0;
			HAL_UART_Receive_IT(&huart1, (uint8_t *)pi_buff, 1); //每接收一个数据，就打开一次串口中断接收，否则只会接收一个数据就停止接收
		}
    
}
void sendPiMessage(const char* fmt, ...){
	va_list args;
	char fmt2[102] = {""};
	char send_msg[102] = {""};
	strcpy(fmt2, fmt);
	strcat(fmt2, "\r\n");
	va_start(args, fmt);
	vsprintf(send_msg, fmt2, args);
	va_end(args);
	HAL_UART_Transmit(&huart1, (uint8_t*)send_msg, strlen(send_msg), 50);
	if(sendToLora == 1){
		HAL_UART_Transmit(&huart2, (uint8_t*)send_msg, strlen(send_msg), 50);
	}
}
void sendLoraMessage(const char* fmt, ...){
	va_list args;
	char fmt2[52] = {""};
	char send_msg[102] = {""};
	strcpy(fmt2, fmt);
	strcat(fmt2, "\r\n");
	va_start(args, fmt);
	vsprintf(send_msg, fmt2, args);
	va_end(args);
	HAL_UART_Transmit(&huart2, (uint8_t*)send_msg, strlen(send_msg), 50);
}
void processMsg(char* msg, char* streamingMsg)
{
		if(strlen(streamingMsg) == 0){
			return;
		}
    strcat(msg, streamingMsg);
    int i = 0;
    for (;i<strlen(msg);i++)
    {
        if (msg[i] == ENDLINE_CHAR)
        {
            char command[50] = {""};
            strncpy(command, msg, i);
            solve_Command(command);
            if (i == strlen(msg)-1)
            {
                memset(msg, 0, sizeof(*msg));
            }else
            {
                strcpy(msg, &msg[i+1]);
                break;
            }
            
        }
    }
    
    
}
void solve_Command(char* rec_msg){
		if(start==0){
			sendPiMessage("error Excute Command Error: System starting");
			return;
		}
		char rec_msg_copy[50] = {""};
		strcpy(rec_msg_copy, rec_msg);
		
		// 指令分发
		char* command;
		const char split_token[1] = " ";
		command = strtok(rec_msg, split_token);
		char ret_msg[100];
		if(!strcmp(command, "led")){
			// 控制led灯
			char* status = strtok(NULL, split_token);
			if(!strncmp(status, "on", 2)){
				strcpy(ret_msg, "ret_msg led on");
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			}else{
				strcpy(ret_msg, "ret_msg led off");
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			}
			sendPiMessage(ret_msg);
		}else if(!strcmp(command, "motion")){
			// 获取加速度和角速度
			read_motion = 1;	
		}else if(!strcmp(command, "motion_dmp")){
			read_motion_dmp = 1;
		}else if(!strcmp(command, "pwm")){
			char* command_split= strtok(NULL, split_token);
			if(!strncmp(command_split, "set", 3)){
				// 如果是设置电机
				int tim_channel_n = 0;
				int pwm_val2 = 50;
				sscanf(rec_msg_copy, "pwm set %d %d", &tim_channel_n, &pwm_val2);
				if(pwm_val2 >= 0 && pwm_val2 < 500 && tim_channel_n >= 0 && tim_channel_n < 6){
					pwm_val_arr[tim_channel_n] = pwm_val2;
					pwm_val = (uint16_t) pwm_val2;
					
					switch(tim_channel_n){
						case 0:
							__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_val);
							break;
						case 1:		
							__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pwm_val);
							break;
						case 2:
							__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwm_val);
							break;
						case 3:
							__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, pwm_val);
							break;
						case 4:
							__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_val);
							break;
						case 5:
							__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwm_val);
							break;
						default:
							break;
					}
					sprintf(ret_msg, "ret_msg pwm %d, %d", tim_channel_n, pwm_val);
					sendPiMessage(ret_msg);
				}else{
					// 设置电机的参数不合法
					sendPiMessage("error Setting PWM Error: illegal parameter");
				}			
			}else{
				// 获取电机占空比
				sprintf(ret_msg, "data pwm %d %d %d %d %d %d", pwm_val_arr[0], pwm_val_arr[1], pwm_val_arr[2], pwm_val_arr[3], pwm_val_arr[4], pwm_val_arr[5]);
				sendPiMessage(ret_msg);
			}
		}else if(!strcmp(command, "lora")){
			// 向lora传消息
			sendLoraMessage(rec_msg_copy);
			sendPiMessage("ret_msg lora msg sent");
		}else if(!strcmp(command, "pressure")){
			// 获取压力
			// 在中断里使用莫名会卡住，读取部分放在主函数里
				read_pressure = 1;
			
		}else{
			strcpy(ret_msg, "error Unknown Message: ");
			strcat(ret_msg, rec_msg);
			sendPiMessage(ret_msg);
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
