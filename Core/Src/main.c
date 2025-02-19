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

//min an max based off of 1.32% tolerance
#define HALF_BIT_DELTA_MIN 487
#define HALF_BIT_DELTA_MAX 513

#define FULL_BIT_DELTA_MIN 974
#define FULL_BIT_DELTA_MAX 1026
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

volatile uint16_t test_input[255] = {0};
volatile uint8_t test_index = 0;
volatile uint8_t test_index_2 = 0;

//Receiver Variables
volatile uint32_t previous_capture_val = 0;
volatile uint8_t current_partial_byte = 0;
volatile uint8_t bit_count = 0;
volatile char receive_buffer[256];
volatile uint8_t receive_index = 0;
volatile bool receiving = false;
volatile uint32_t current_pin_state = 1;
//volatile uint32_t previous_pin_state = 1; // idle is high
volatile uint8_t change_lights_flag = 0; //1 when lights need changing
volatile bool end_reception_flag = false;;

volatile bool middle_bit = true;
//volatile uint32_t delta = 0;

//volatile uint32_t print_test_man[512] = {0};
//volatile uint16_t print_test_index = 0;
//volatile uint8_t print_test_ind[512] = {0};
//volatile uint16_t print_test_ind_index = 0;

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
  //printf("Hello World!\n");
  CurrentState = IDLE_STATE;
  updateStateLights();
  //RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN; //enable MCU debug module clock
//  __HAL_DBGMCU_FREEZE_TIM2();
//  __HAL_DBGMCU_FREEZE_TIM3();

  //Start timer for channel monitor/receiver
//  printf("Sanity Check\n");
//
//  receiving = true;
//  receive_buffer[0] = 'a';
//  receive_buffer[1] = 'b';
//  receive_buffer[2] = 'c';
//  receive_index = 3;
//  end_reception();


  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	//Receiver Code
	  //printf("Sanity Check");

	  if(end_reception_flag){
		  //end_reception();
		  end_reception_flag = false;
		  //if (receiving) {
			 // receiving = false;
//			  if (bit_count > 0) {
//				  // Discard incomplete byte
//			  }
		  if (receive_index > 0) {
			  printf("Received: %s\n", receive_buffer);
			  for (int i = 0; i < receive_index; i++) {
				  receive_buffer[i] = 0;
			  }
//				  printf("\n");

		  }
		  receive_index = 0;
//		  bit_count = 0;
//		  current_partial_byte = 0;
//  //        previous_pin_state = 1;
//		  middle_bit = true;
		 // }
//		  __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
//		  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	  }
	  if(change_lights_flag == 1){
		  updateStateLights();
	  }



	  //printf("Captured Val: %i\tCurrent State: %i\tPin Value: %d\n", capture_val, CurrentState, pinValue);
	  //HAL_Delay(1000);
//	  if(!transmitting) {
////		  if(print_test_man[0] != 0) {
////			  printf("Past transmitted values:\n");
////			  for(int i = 0; i < print_test_index; i++) {
////				  printf("%#08lX\n", print_test_man[i]);
////			  }
////		  }
//		  printf("Enter text to transmit: ");
//		  //fgets(transmit_buffer, 255, stdin);
//		  char temp_input[255];
//		  //scanf("%[^\n]s", temp_input);
//		  fgets(temp_input, 255, stdin);
//		  strncpy(transmit_buffer, temp_input, 255);
//		  //printf("Stuff transmitted: %s\n", transmit_buffer);
//		  printf("Message sent\n");
//		  for(int i = 0; i <= 255; i++) {
//			  test_input[i] = 0;
//		  }
//		  for(int i = 0; i <= 512; i++) {
//			  //print_test_man[i] = 0;
//			  //print_test_ind[i] = 0;
//		  }
//		  transmitting = true;
//		  //print_test_index = 0;
//		  //print_test_ind_index = 0;
//		  test_index = 0;
//		  test_index_2 = 0;
//		  manchester_buffer = 0;
//		  transmit_buffer_index = 0;
//		  end_of_transmission = false;
//		  manchester_buffer = getNextTransmissionChar(true);
//		  //print_test_man[print_test_index] = manchester_buffer;
//		  //print_test_index++;
//		  //test_input[0] = getNextTransmissionChar(true);
//		  manchester_bit_count += 16;
//		  uint16_t temp = getNextTransmissionChar(false);
////		  while(!end_of_transmission) {
////			  test_input[transmit_buffer_index] = getNextTransmissionChar(false);
////		  }
//		  if(temp != 0) {
//			  manchester_buffer |= (temp<<16);
//			  manchester_bit_count += 16;
//			  //print_test_man[print_test_index] = manchester_buffer;
//			  //print_test_index++;
//		  } else {
//			  end_of_transmission = true;
//		  }
////		  if((manchester_buffer & 0b1) == 0b1) {
////				HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 1);
////		  } else {
////				HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 0);
////		  }
//
//		  /*if((manchester_buffer & 0b1) != ((manchester_buffer>>1) & 0b1)) {
//			manchester_buffer = manchester_buffer>>1;
//			manchester_bit_count--;
//			__HAL_TIM_SET_AUTORELOAD(&htim3, HALF_PERIOD);
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, HALF_PERIOD);
//		  } else {
//			manchester_buffer = manchester_buffer>>2;
//			manchester_bit_count -= 2;
//			__HAL_TIM_SET_AUTORELOAD(&htim3, FULL_PERIOD);
//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, FULL_PERIOD);
//		  }*/
//		  if(CurrentState == IDLE_STATE) {
////			  HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, (manchester_buffer & 0b1));
////			  print_test_ind[0] = (manchester_buffer & 0b1);
////			  print_test_ind_index++;
////			  test_input[test_index_2] |= ((manchester_buffer&0b1)<<(15-test_index));
////			  test_index++;
////			  //manchester_buffer = manchester_buffer>>1;
////			  manchester_bit_count--;
////			  print_test_man[print_test_index] = manchester_buffer;
////			  print_test_index++;
//
//
//			  __HAL_TIM_SET_AUTORELOAD(&htim3, HALF_PERIOD);
//			  __HAL_TIM_SET_COUNTER(&htim3, 0);
//			  HAL_TIM_Base_Start_IT(&htim3);
//		  }
//
//	  }
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
//void end_reception() {
//	end_reception_flag = 0;
//    if (receiving) {
//        receiving = false;
//        if (bit_count > 0) {
//            // Discard incomplete byte
//        }
//        if (receive_index > 0) {
//            printf("Received: ");
//            for (int i = 0; i < receive_index; i++) {
//                printf("%c", receive_buffer[i]);
//            }
//            printf("\n");
//        }
//        receive_index = 0;
//        bit_count = 0;
//        current_partial_byte = 0;
////        previous_pin_state = 1;
//        middle_bit = true;
//    }
//    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
//    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
//}


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

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	//BUSY!
	if (htim->Instance == TIM2) { // Ensure it's TIM2
		if (CurrentState == BUSY_STATE && receiving){

			capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	        // Compute the next compare value with delay
	        compare_val = (capture_val + delay_us) % TIMER_MAX;

	        // Set compare value for output compare event
	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare_val);

	        // Start Output Compare interrupt
	        HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);

	        //Changes LED's for busy state
	    	CurrentState = BUSY_STATE;
	    	change_lights_flag = 1;
	    	//updateStateLights();


			//Receiver Code
	    	uint32_t delta = capture_val - previous_capture_val;
	    	previous_capture_val = capture_val;
	    	//uint32_t previous_pin_state = current_pin_state;
			current_pin_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

            //uint8_t edge_direction = 0; // 0 = falling, 1 = rising
//			if (current_pin_state != previous_pin_state) {
				//edge_direction = current_pin_state;
//			} else {
//				edge_direction = 2; // invalid because no edge
//			}

			//check for valid bit before continuing
//			if(edge_direction == 2) {
//				//error
//				CurrentState = ERR_STATE;
//				updateStateLights();
//				end_reception();
//				return;
//			}

			//check for edge timing in correct range
			if((delta >= HALF_BIT_DELTA_MIN && delta <= HALF_BIT_DELTA_MAX) || (delta >= FULL_BIT_DELTA_MIN && delta <= FULL_BIT_DELTA_MAX)) {
				//build current received byte
				if(delta >= HALF_BIT_DELTA_MIN && delta <= HALF_BIT_DELTA_MAX){
					if(middle_bit) {
						current_partial_byte = (current_partial_byte << 1) | current_pin_state; // Repeat the previous bit
					}
					middle_bit = !middle_bit;
				} else {
					current_partial_byte = (current_partial_byte << 1) | current_pin_state;
					middle_bit = false;
				}
				bit_count++;

				//Once 8 bits put byte into buffer
				if(bit_count >= 8){
					receive_buffer[receive_index] = current_partial_byte;
					receive_index++;
					current_partial_byte = 0;
					bit_count = 0;

					//check for buffer full
					if(receive_index > 254) {
						end_reception_flag = true;
						receiving = false;
		//			    if (bit_count > 0) {
		//				    // Discard incomplete byte
		//			    }
//					    if (receive_index > 0) {
//						    printf("Received: %s\n", receive_buffer);
//		//				    for (int i = 0; i < receive_index; i++) {
//		//					    printf("%c", receive_buffer[i]);
//		//				    }
//		//				    printf("\n");
//					    }
					    //receive_index = 0;
					    bit_count = 0;
					    current_partial_byte = 0;
			  //        previous_pin_state = 1;
					    middle_bit = true;
						//end_reception();
						//will probably break stuff now if this happens but we're not
						// expecting that big of packets anyway
					    __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
					    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
					}

				}
			} else {
					//timing was out of expected range
					CurrentState = ERR_STATE;
					change_lights_flag = 1;
					//updateStateLights();
					//end_reception_flag = 1;
					//end_reception();
					end_reception_flag = true;
					receiving = false;
	//			    if (bit_count > 0) {
	//				    // Discard incomplete byte
	//			    }
//					    if (receive_index > 0) {
//						    printf("Received: %s\n", receive_buffer);
//		//				    for (int i = 0; i < receive_index; i++) {
//		//					    printf("%c", receive_buffer[i]);
//		//				    }
//		//				    printf("\n");
//					    }
					//receive_index = 0;
					bit_count = 0;
					current_partial_byte = 0;
		  //        previous_pin_state = 1;
					middle_bit = true;
					//end_reception();
					//will probably break stuff now if this happens but we're not
					// expecting that big of packets anyway
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
					HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
				
				}
		} else if(CurrentState == IDLE_STATE) {
			//First edge (starting receiving)
			 // Initial edge detection (start of reception)
			 capture_val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			 compare_val = (capture_val + delay_us) % TIMER_MAX;
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, compare_val);
			 HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
 
			 // Transition to BUSY_STATE and start reception
			 CurrentState = BUSY_STATE;
			 change_lights_flag = 1;
			 //updateStateLights();
			 //HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
			 receiving = true;
			 receive_index = 0;
			 current_partial_byte = 0;
			 bit_count = 1;
			 previous_capture_val = capture_val;
//			 previous_pin_state = 1;
			 middle_bit = true;
		}
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
    		change_lights_flag = 1;
    		end_reception_flag = 1;
    		//updateStateLights();
			//end_reception();
    		if(transmitting) {
    			HAL_TIM_Base_Start_IT(&htim3);
    		}
    	} else {
    		//HAL_TIM_Base_Stop_IT(&htim3);
    		CurrentState = ERR_STATE;
    		change_lights_flag = 1;
    		//updateStateLights();
    		end_reception_flag = 1;
    		//end_reception();

    		//transmitting = false;
    		//HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_4);
    	}

    } /*else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
//    	if((manchester_buffer & 0b1) == 0b1) {
//			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 1);
//		} else {
//			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 0);
//		}
    	HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, (manchester_buffer & 0b1));
    	if((manchester_buffer & 0b1) != ((manchester_buffer>>1) & 0b1)) {
        	manchester_buffer = manchester_buffer>>1;
        	manchester_bit_count--;
        	__HAL_TIM_SET_AUTORELOAD(&htim3, HALF_PERIOD);
	        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, HALF_PERIOD);
        } else {
        	manchester_buffer = manchester_buffer>>2;
        	manchester_bit_count -= 2;
        	__HAL_TIM_SET_AUTORELOAD(&htim3, FULL_PERIOD);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, FULL_PERIOD);
        }
		//HAL_GPIO_TogglePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin);
    	if(!end_of_transmission && (manchester_bit_count <= 16)) {
    		uint16_t reverse_manchester = getNextTransmissionChar(false);
    		manchester_bit_count += 16;
    		manchester_buffer &= 0xFF;
    		if(reverse_manchester == 0) {
    			end_of_transmission = true;
    		} else {
    			manchester_buffer |= (reverse_manchester<<16);
    		}
    	} else if(manchester_bit_count == 0) {
    		HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_4);
    		transmitting = false;
    		//HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_4);
    	}

    }*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
	if (htim->Instance == TIM3/* && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4*/) {
//    	if((manchester_buffer & 0b1) == 0b1) {
//			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 1);
//		} else {
//			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 0);
//		}
		 if(manchester_bit_count == 0) {
			HAL_TIM_Base_Stop_IT(&htim3);
			transmitting = false;
			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, 1);
			//HAL_TIM_OC_Stop(&htim3, TIM_CHANNEL_4);
		} else {




			//print_test_ind[print_test_ind_index] = (manchester_buffer & 0b1);
			//print_test_ind_index++;
			HAL_GPIO_WritePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin, (manchester_buffer & 0b1));
			manchester_buffer = manchester_buffer>>1;
			manchester_bit_count--;
			//print_test_man[print_test_index] = manchester_buffer;
			//print_test_index++;
			if(test_index > 15) {
				test_index = 0;
				test_index_2++;
			}
			test_input[test_index_2] |= ((manchester_buffer&0b1)<<(15-test_index));
			test_index++;

	//		if((manchester_buffer & 0b1) != ((manchester_buffer>>1) & 0b1)) {
	//			manchester_buffer = manchester_buffer>>1;
	//			manchester_bit_count--;
	//			__HAL_TIM_SET_AUTORELOAD(&htim3, HALF_PERIOD);
	//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, HALF_PERIOD);
	//		} else {
	//			manchester_buffer = manchester_buffer>>2;
	//			manchester_bit_count -= 2;
	//			__HAL_TIM_SET_AUTORELOAD(&htim3, FULL_PERIOD);
	//			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, FULL_PERIOD);
	//		}
			//HAL_GPIO_TogglePin(TRANSMIT_GPIO_Port, TRANSMIT_Pin);
			if(!end_of_transmission && (manchester_bit_count <= 16)) {
				uint16_t reverse_manchester = getNextTransmissionChar(false);
				//manchester_bit_count += 16;
				if(manchester_bit_count == 16) {
					manchester_buffer &= MASK_16_BITS;
					//print_test_man[print_test_index] = manchester_buffer;
					//print_test_index++;
				} else {
					manchester_buffer &= MASK_17_BITS;
					//print_test_man[print_test_index] = manchester_buffer;
					//print_test_index++;
				}
				if(reverse_manchester == 0) {
					end_of_transmission = true;
				} else {
					manchester_bit_count += 16;
					manchester_buffer |= (reverse_manchester<<(manchester_bit_count-16));
					//print_test_man[print_test_index] = manchester_buffer;
					//print_test_index++;
				}
			}

		}
	}
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == GPIO_PIN_11){
//		//enable busy LED & OFF IDLE
//		HAL_GPIO_TogglePin(IDLE_LED_GPIO_Port, IDLE_LED_Pin);
//		HAL_GPIO_TogglePin(BUSY_LED_GPIO_Port, BUSY_LED_Pin);
//	}
//}


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
