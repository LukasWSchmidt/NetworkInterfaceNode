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
#include <stdlib.h>

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

//min an max based off of 1.32% tolerance
#define HALF_BIT_DELTA_MIN 487
#define HALF_BIT_DELTA_MAX 520

#define FULL_BIT_DELTA_MIN 974
#define FULL_BIT_DELTA_MAX 1046

#define PREAMBLE 0x55
#define SENDER_ADDR1 0x34
#define SENDER_ADDR2 0x35
#define SENDER_ADDR3 0x36
#define SENDER_ADDR4 0x37
#define BLANK_CRC 0xAA

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

volatile char transmit_buffer[260];
volatile uint32_t manchester_buffer = 0;
volatile uint8_t manchester_bit_count = 0;
volatile uint8_t transmit_buffer_index = 0;
volatile bool end_of_transmission = false;
uint8_t destination_addr = 0xFF; //default to broadcast

volatile bool blue_debug_mode = false;

//Receiver Variables
volatile uint32_t previous_capture_val = 0;
volatile uint8_t current_partial_byte = 0;
volatile uint8_t bit_count = 0;
volatile char receive_buffer[260];
volatile uint8_t receive_index = 0;
volatile bool receiving = false;
volatile uint32_t current_pin_state = 1;
volatile uint8_t change_lights_flag = 0; //1 when lights need changing
volatile bool end_reception_flag = false;
uint16_t backoff_counter = 0;
bool backoff_delay = false;
bool receive_printed = false;
volatile uint8_t receive_addr = 0;

volatile bool middle_bit = true;

volatile bool console_up = true;
volatile bool restart_transmission = false;
volatile bool update_transmission_bit = false;

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
void end_reception();
void begin_transmission();
void console_input();

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  CurrentState = IDLE_STATE;
  updateStateLights();

  HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	//Receiver Code
	  //printf("Sanity Check");

//	  if(end_reception_flag){
//		  end_reception();
//	  }

	  if(update_transmission_bit) {
		  update_transmission_bit = false;
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
	  //NEED TO REWORK - WAIT FOR IDLE
	  //backoff code
	  if(backoff_delay) {
		  backoff_delay = false;
		  if(backoff_counter < 10) {
			//delay random time between 0 and 1000ms
			//HAL_Delay((rand() % 1000));
			__HAL_TIM_SET_AUTORELOAD(&htim4, rand()%1000);
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			//attempt to transmit again
			printf("\n--> Message failed to send. Retrying, attempt %d (waiting for idle)\n", backoff_counter+1);
			//begin_transmission();
			backoff_counter++;
			HAL_TIM_Base_Start_IT(&htim4);
//			transmitting = true;
//			manchester_buffer = 0;
//		  	transmit_buffer_index = 0;
//		  	end_of_transmission = false;
//		 	manchester_buffer = getNextTransmissionChar(true);
//		  	manchester_bit_count += 16;
//		  	uint16_t temp = getNextTransmissionChar(false);
//		  	if(temp != 0) {
//			  manchester_buffer |= (temp<<16);
//			  manchester_bit_count += 16;
//		  	} else {
//			  end_of_transmission = true;
//		  	}
//		  	if(CurrentState == IDLE_STATE) {
//			  __HAL_TIM_SET_AUTORELOAD(&htim3, HALF_PERIOD);
//			  __HAL_TIM_SET_COUNTER(&htim3, 0);
//			  HAL_TIM_Base_Start_IT(&htim3);
//		  	}
		} else {
			//backoff failed after 10 attempts
			//backoff_delay = false;
			backoff_counter = 0;
			//print failed to transmit message
			printf("\n--> Failed to transmit message after 10 attempts\n");
			transmitting = false;
			console_up = true;
			//return to idle state
			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 1);
			//CurrentState = IDLE_STATE;
			change_lights_flag = 1;
		}
	  }

	  if(restart_transmission) {
		  restart_transmission = false;
		  transmitting = true;
		  begin_transmission();
	  }

	  if(end_of_transmission && !transmitting && !restart_transmission && !backoff_delay) {
		  end_of_transmission = false;
		  console_up = true;
		  backoff_counter = 0;
		  printf("--> Message sent\n");
	  }
	  if(change_lights_flag == 1){
		  updateStateLights();
	  }
	  if(console_up) {
		  printf("CMD> ");
		  char temp_input[263];
		  //updateStateLights();
		  fgets(temp_input, 262, stdin);
		  console_input(temp_input);
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


//helper function for ending reception and printing buffered message
void end_reception() {
	//end_reception_flag = false;
	receiving = false;
//	if ((receive_index > 0) && (CurrentState == 0)) {
//		printf("Received: %s\n", receive_buffer);
//		for (int i = 0; i < receive_index; i++) {
//			receive_buffer[i] = 0;
//		}
//	}
	if(receive_index < 259) {
		receive_buffer[receive_index] = '\0';
	}
	receive_index = 0;
	bit_count = 0;
	current_partial_byte = 0;
	middle_bit = true;
}


void updateStateLights(){
	change_lights_flag = 0;
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

void begin_transmission() {
	  //transmitting = true;
	  //backoff_counter = 0;
	  manchester_buffer = 0;
	  manchester_bit_count = 0;
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
	  } else if((CurrentState == ERR_STATE) && (backoff_counter == 0)) {
		  backoff_delay = true;
	  }
}

//check preamble?
void console_input(char* input) {
	printf(input);
	if(input[0] == 'r') {
		if(!receive_printed) {
			if((receive_buffer[2] == SENDER_ADDR1) || (receive_buffer[2] == SENDER_ADDR2) || (receive_buffer[2] == SENDER_ADDR3) || (receive_buffer[2] == SENDER_ADDR4)) {
				printf("--> Last received message (from node %d): %s\n", receive_buffer[1], receive_buffer+5);
				for (int i = 0; i < 260; i++) {
					receive_buffer[i] = 0;
				}
			} else if(receive_buffer[2] == 0xFF) {
				printf("--> Last received message (broadcast): %s\n", receive_buffer+5);
				for (int i = 0; i < 260; i++) {
					receive_buffer[i] = 0;
				}
			} else {
				printf("--> No messages at this time\n");
			}
		} else {
			printf("--> No messages at this time\n");
		}
	} else if(input[0] == 's') {
		char cmd[3];
		uint8_t dest_addr = 0;
		//char message[256];
		//sscanf(input, "%s %hhd %255s", cmd, &dest_addr, message);
		char temp_input[263];
		strncpy(temp_input, input, 255);
		char* space_ptr = strchr(temp_input, ' ');
		space_ptr++;
		if(space_ptr != NULL) {
			strncpy(cmd, temp_input, 2);
		}
		char temp_input2[263];
		strncpy(temp_input2, space_ptr, 263);
		space_ptr = strchr(temp_input2, ' ');
		space_ptr++;
		char dest_addr_str[10];
		if(space_ptr != NULL) {
			strncpy(dest_addr_str, temp_input2, space_ptr-temp_input2);
			dest_addr = (uint8_t)strtol(dest_addr_str, NULL, 10);
		}
		if(cmd[1] == 'x') {
			if(dest_addr == 0xFF) {
				printf("--> Broadcasting hex message: 0x%s", space_ptr);
			} else {
				printf("--> Sending hex message to node %d: 0x%s", dest_addr, space_ptr);
			}
			// Add preamble + source addr
			transmit_buffer[0] = PREAMBLE;
			transmit_buffer[1] = SENDER_ADDR1;
			transmit_buffer[2] = dest_addr;

			// Length of message
			//uint8_t length = strlen(space_ptr + 1); // Skips past the space
			transmit_buffer[3] = strlen(space_ptr)/2;
			transmit_buffer[4] = BLANK_CRC;

			char hex_conversion[256];
			uint16_t index = 0;
			uint16_t conversion_index = 0;
			//Converts each hex character to binary, adds them to regular transmit
			//buffer so that 1 char = 2 hex characters
			while((index < 255) && (space_ptr[index] != '\n') && (space_ptr[index] != '\r')) {
				uint8_t hex1 = 16;
				while((index < 255) && (hex1 >= 16)) {
					hex1 = charToBinary(space_ptr[index]);
					index++;
				}
				uint8_t hex2 = 16;
				while((index < 255) && (hex2 >= 16)) {
					hex2 = charToBinary(space_ptr[index]);
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
			hex_conversion[conversion_index] = '\n';
			strncpy((char*)&transmit_buffer[5], hex_conversion, 255);
			transmit_buffer[conversion_index+5] = '\n';
			transmit_buffer[conversion_index+6] = BLANK_CRC;
			transmitting = true;
		} else if((cmd[1] == '\0') || (cmd[1] == ' ')) {
			if(dest_addr == 0xFF) {
				printf("--> Broadcasting message: %s", space_ptr);
			} else {
				printf("--> Sending message to node %d: %s", dest_addr, space_ptr);
			}
			// Add preamble + source addr
			transmit_buffer[0] = PREAMBLE;
			transmit_buffer[1] = SENDER_ADDR1;
			transmit_buffer[2] = dest_addr;

			// Length of message
			uint8_t length = strlen(space_ptr); // Skips past the space
			transmit_buffer[3] = length;
			transmit_buffer[4] = BLANK_CRC;
			// Copy input to transmit buffer starting from the 6th position
			strncpy((char*)&transmit_buffer[5], space_ptr, length); // Skip destination address

			// Empty CRC8 field after message for now
			if(length < 256) {
				transmit_buffer[length + 5] = BLANK_CRC;
			}
			transmitting = true;
		} else {
			printf("--> Error: Invalid command\n");
		}
		if(transmitting) {
			begin_transmission();
			console_up = false;
		}

	} else {
		printf("--> Error: Invalid command\n");
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	//BUSY!
	if (htim->Instance == TIM2) { // Ensure it's TIM2
		capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		if (CurrentState == BUSY_STATE && receiving){


			//capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	        // Compute the next compare value with delay
	        compare_val = (capture_val + delay_us) % TIMER_MAX;

	        // Set compare value for output compare event
	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare_val);

	        // Start Output Compare interrupt
	        HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);

	        current_pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
	        //Changes LED's for busy state
	    	CurrentState = BUSY_STATE;
	    	change_lights_flag = 1;

	    	if(!end_reception_flag) {
			//Receiver Code
				uint32_t delta = capture_val - previous_capture_val;
				previous_capture_val = capture_val;


			//check for edge timing in correct range
				if((delta >= HALF_BIT_DELTA_MIN && delta <= HALF_BIT_DELTA_MAX) || (delta >= FULL_BIT_DELTA_MIN && delta <= FULL_BIT_DELTA_MAX)) {
					//build current received byte
					if(delta >= HALF_BIT_DELTA_MIN && delta <= HALF_BIT_DELTA_MAX){
						if(middle_bit) {
							current_partial_byte = (current_partial_byte << 1) | current_pin_state; // Repeat the previous bit
							bit_count++;
						}
						middle_bit = !middle_bit;
					} else {
						current_partial_byte = (current_partial_byte << 1) | current_pin_state;
						middle_bit = false;
						bit_count++;
					}

					//Once 8 bits put byte into buffer
					if(bit_count >= 8){
						receive_buffer[receive_index] = current_partial_byte;
						receive_index++;
						current_partial_byte = 0;
						bit_count = 0;

						//check for buffer full
						if(receive_index > 260) {
							end_reception_flag = true;
							end_reception();
						}

						//check byte for message detail
						//byte 0 is preamble (ignore, just helpful for collision detection)
						//byte 1 is sender address
						//byte 2 is destination address (check if broadcast or for this node)
						if(receive_index == 3) {
							receive_addr = receive_buffer[2];
							if((receive_addr != SENDER_ADDR1) && (receive_addr != SENDER_ADDR2) && (receive_addr != SENDER_ADDR3) && (receive_addr != SENDER_ADDR4) && (receive_addr != 0xFF)) { //not this node or not broadcast
								//not for this node
								end_reception_flag = true;
								//end_reception();
								//CurrentState = IDLE_STATE;
								//change_lights_flag = 1;
								//clear buffer
	//							for (int i = 0; i < receive_index; i++) {
	//								receive_buffer[i] = 0;
	//							}
							}
						}
						//Ignore CRC for now



					}
				} else {
					//timing was out of expected range
					CurrentState = ERR_STATE;
					change_lights_flag = 1;
					if(receiving) {
						end_reception_flag = true;
						end_reception();
					}
				}
	    	}
		} else /*if(CurrentState == IDLE_STATE)*/ {
			//First edge (starting receiving)
			 // Initial edge detection (start of reception)
			 //capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			 compare_val = (capture_val + delay_us) % TIMER_MAX;
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare_val);
			 HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
 
			 // Transition to BUSY_STATE and start reception
			 CurrentState = BUSY_STATE;
			 change_lights_flag = 1;
			 receiving = true;
			 end_reception_flag = false;
			 receive_index = 0;
			 current_partial_byte = 0;
			 bit_count = 1;
			 previous_capture_val = capture_val;
			 middle_bit = true;
		}
		if(console_up) {
			updateStateLights();
		}
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	//HAL_TIM_Base_Stop_IT(&htim3);
	//uint32_t test_capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        //Error or Idle, do Idle pattern if line is high

		//uint32_t test_capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    	pinValue = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    	if(pinValue == 1){
    		//IDLE
    		CurrentState = IDLE_STATE;
    		change_lights_flag = 1;
    		//updateStateLights();
    		if(receiving) {
    			end_reception_flag = true;
    			end_reception();
    		}
    		if(restart_transmission) {
    			restart_transmission = false;

    		}
   			if(transmitting) {
    			HAL_TIM_Base_Start_IT(&htim3);
   			}
    	} else {
    		CurrentState = ERR_STATE;
    		change_lights_flag = 1;
    		HAL_TIM_Base_Stop_IT(&htim3);
			//Random backoff on collision
			//backoff_counter++;

    		if(receiving) {
    			end_reception_flag = true;
    			end_reception();
    		}
    		if(transmitting) {
    			transmitting = false;
    			backoff_delay = true;
    		}
    	}
    	if(console_up) {
			updateStateLights();
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
			console_up = false;
			backoff_counter = 0;
			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 1);
		} else {
			update_transmission_bit = true;
			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, (manchester_buffer & 0b1));
			/*manchester_buffer = manchester_buffer>>1;
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
			}*/

		}
	} else if (htim->Instance == TIM4) {
		HAL_TIM_Base_Stop_IT(&htim4);
		restart_transmission = true;
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
