/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Networking Interface Node Main Code
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RCC_AHB1_ENR 0x40023830
#define GPIOA_BASE 0x40020000
#define GPIOA_EN 0

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)

#define IDLE_STATE 0
#define BUSY_STATE 1
#define ERR_STATE 2

#define TIMER_MAX 0xFFFFFFFF

#define MASK_16_BITS 0x0000FFFF
#define MASK_17_BITS 0x00007FFF
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct GPIO_t {
	uint32_t moder;
	uint32_t otyper;
	uint32_t ospeedr;
	uint32_t pupdr;
	uint32_t idr;
	uint32_t odr;
	uint32_t bsrr;
	uint32_t lckr;
	uint32_t afrl;
	uint32_t afrh;

} GPIO_t;

volatile GPIO_t* gpioa = (GPIO_t*)GPIOA_BASE;

volatile uint32_t capture_val = 0;
volatile uint32_t compare_val = 0;
volatile uint32_t delay_us = 1131;
volatile int CurrentState = IDLE_STATE;
uint8_t pinValue = 0;
volatile bool transmitting = false;

volatile char transmit_buffer[255];
volatile uint32_t manchester_buffer = 0;
volatile uint8_t manchester_bit_count = 0;
volatile uint8_t transmit_buffer_index = 0;
volatile bool end_of_transmission = false;

volatile bool blue_debug_mode = false;

PUTCHAR_PROTOTYPE
{
 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
 return ch;
}
GETCHAR_PROTOTYPE
{
 uint8_t ch = 0;
 __HAL_UART_CLEAR_OREFLAG(&huart2);
 HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
 return ch;
}

void updateStateLights();
uint16_t getNextTransmissionChar(bool first);
uint8_t charToBinary(char c);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setvbuf(stdin, NULL, _IONBF, 0);


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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  CurrentState = IDLE_STATE;
  updateStateLights();
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //printf("Captured Val: %i\tCurrent State: %i\tPin Value: %d\n", capture_val, CurrentState, pinValue);
	  //HAL_Delay(1000);
	  if(!transmitting) {
		  char temp_input[255];
		  if(blue_debug_mode) {
			  //invalid characters are skipped without inserting a 0
			  printf("Enter debug text to transmit (hex, no \"0x\"): ");
			  fgets(temp_input, 255, stdin);
			  strncpy(transmit_buffer, temp_input, 255);
			  printf("Message sent\n");
			  char hex_conversion[255];
			  uint16_t index = 0;
			  uint16_t conversion_index = 0;
			  //Converts each hex character to binary, adds them to regular transmit
			  //buffer so that 1 char = 2 hex characters
			  while((index < 255) && (transmit_buffer[index] != '\n') && (transmit_buffer[index] != '\r')) {
				  uint8_t hex1 = 16;
				  while((index < 255) && (hex1 >= 16)) {
					  hex1 = charToBinary(transmit_buffer[index]);
					  index++;
				  }
				  uint8_t hex2 = 16;
				  while((index < 255) && (hex2 >= 16)) {
					  hex2 = charToBinary(transmit_buffer[index]);
					  index++;
				  }
				  if(hex1 >= 16) {
					  hex1 = 0;
				  }
				  if(hex2 >= 16) {
					  hex2 = 0;
				  }
				  hex_conversion[conversion_index] = (hex1<<4)|(hex2);
				  conversion_index++;
			  }
			  strncpy(transmit_buffer, hex_conversion, 255);
			  transmit_buffer[conversion_index] = '\n';
		  } else {
			  printf("Enter regular text to transmit: ");
			  fgets(temp_input, 255, stdin);
			  strncpy(transmit_buffer, temp_input, 255);
			  printf("Message sent\n");
		  }
		  transmitting = true;
		  manchester_buffer = 0;
		  transmit_buffer_index = 0;
		  end_of_transmission = false;
		  manchester_buffer = getNextTransmissionChar(true);
		  manchester_bit_count += 16;
		  uint16_t temp = getNextTransmissionChar(false);
		  if(temp != 0) {
			  manchester_buffer |= (temp<<16);
			  manchester_bit_count += 16;
		  } else {
			  end_of_transmission = true;
		  }
		  if(CurrentState == IDLE_STATE) {
			  __HAL_TIM_SET_AUTORELOAD(&htim3, HALF_PERIOD);
			  __HAL_TIM_SET_COUNTER(&htim3, 0);
			  HAL_TIM_Base_Start_IT(&htim3);
		  }

	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void updateStateLights(){
	if(CurrentState == 0){
		//IDLE LED
		//gpioa->odr |= (001<<IDLE_LED_Pin);
		HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, 0);
		HAL_GPIO_WritePin(BUSY_LED_GPIO_Port, BUSY_LED_Pin, 0);
		HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, 1);
	} else if(CurrentState == 1){
		//BUSY LED
		//gpioa->odr |= (010<<IDLE_LED_Pin);
		HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, 0);
		HAL_GPIO_WritePin(BUSY_LED_GPIO_Port, BUSY_LED_Pin, 1);
		HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, 0);
	} else {
		//ERR
		//gpioa->odr |= (100<<IDLE_LED_Pin);
		HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
		HAL_GPIO_WritePin(BUSY_LED_GPIO_Port, BUSY_LED_Pin, 0);
		HAL_GPIO_WritePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin, 0);
	}
}

uint16_t getNextTransmissionChar(bool first) {
	if(!first) {
		//transmit_buffer_index++;
		if((transmit_buffer_index == 0) || (transmit_buffer[transmit_buffer_index] == '\n') || (transmit_buffer[transmit_buffer_index] == '\r')) {
			end_of_transmission = true;
			return 0;
		}
	} else if((transmit_buffer[transmit_buffer_index] == '\n') || (transmit_buffer[transmit_buffer_index] == '\r')) {
		end_of_transmission = true;
		return 0;
	}
	uint16_t reverse_manchester = 0;
	for(uint8_t i = 0; i<8; i++) {
		if(((transmit_buffer[transmit_buffer_index]>>(7-i))&0b1)==0b1) {
			reverse_manchester |= (0b10<<(i*2));
		} else {
			reverse_manchester |= (0b01<<(i*2));
		}
	}
	transmit_buffer_index++;
	return reverse_manchester;
}

uint8_t charToBinary(char c) {
	if((c >= '0') && (c <= '9')) {
		return c - '0';
	} else if((c >= 'a') && (c <= 'f')) {
		return c - 'a' + 10;
	} else if((c >= 'A') && (c <= 'F')) {
		return c - 'A' + 10;
	}
	return 255;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	//BUSY!
	if (htim->Instance == TIM2) { // Ensure it's TIM2
	        capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	        // Compute the next compare value with delay
	        compare_val = (capture_val + delay_us) % TIMER_MAX;

	        // Set compare value for output compare event
	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare_val);

	        // Start Output Compare interrupt
	        HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);

	        //Changes LED's for busy state
	    	CurrentState = BUSY_STATE;
	    	updateStateLights();
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_Base_Stop_IT(&htim3);
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        //Error or Idle, do Idle pattern if line is high

    	//pinValue = gpioa->idr(0b1 & (1<<15));
    	pinValue = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    	if(pinValue == 1){
    		//IDLE
    		CurrentState = IDLE_STATE;
    		updateStateLights();
    		if(transmitting) {
    			HAL_TIM_Base_Start_IT(&htim3);
    		}
    	} else {
    		//HAL_TIM_Base_Stop_IT(&htim3);
    		CurrentState = ERR_STATE;
    		updateStateLights();
    		transmitting = false;
    	}

    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
	if (htim->Instance == TIM3) {
		 if(manchester_bit_count == 0) {
			HAL_TIM_Base_Stop_IT(&htim3);
			transmitting = false;
			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 1);
		} else {
			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, (manchester_buffer & 0b1));
			manchester_buffer = manchester_buffer>>1;
			manchester_bit_count--;
			if(!end_of_transmission && (manchester_bit_count <= 16)) {
				uint16_t reverse_manchester = getNextTransmissionChar(false);
				if(manchester_bit_count == 16) {
					manchester_buffer &= MASK_16_BITS;
				} else {
					manchester_buffer &= MASK_17_BITS;
				}
				if(reverse_manchester == 0) {
					end_of_transmission = true;
				} else {
					manchester_bit_count += 16;
					manchester_buffer |= (reverse_manchester<<(manchester_bit_count-16));
				}
			}

		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_13){
		//enable busy LED & OFF IDLE
		blue_debug_mode = !blue_debug_mode;
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
