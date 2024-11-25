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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "pca9685.h"
#include "ina219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INA219_ADDRESS (0x41)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  uint8_t ActiveServo;
  uint16_t adcValues[4];  // Array per i valori convertiti dall'ADC (2 canali)
  uint8_t angles[4];
  uint8_t gripper=0;

  INA219_t ina219;

  // PID parameters
  float Kp = 10.0;  // Proporzionale
  float Ki = 10.0;  // Integrale
  float Kd = 0.01; // Derivativo
  // PID variables
  float previous_error = 0;
  float integral = 0;
  float derivative = 0;
  // Thresholds and limits
  float pwm_max = 4095;  // Valore massimo di PWM (ipotizziamo un timer a 12 bit)
  float pwm_min = 0;     // Valore minimo di PWM

  float pwm_value = 0;
  float error = 0;
  float setpoint = 0;
  float current_value = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == Button_Pin) {
    	if(gripper == 0){
    		gripper = 1;
    	}
    	else{
    		gripper = 0;
    	}
    }
}

// PID control loop
float PI_Control(float setpoint, float current_value) {
    error = setpoint - current_value;
    integral += error;
    derivative = error - previous_error;  // Derivata dell'errore
    previous_error = error;                    // Aggiorna l'errore precedente

    // Calcola il valore di uscita PID
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Limita l'output tra i valori massimi e minimi del PWM
    if (output > pwm_max) {
        output = pwm_max;
    } else if (output < pwm_min) {
        output = pwm_min;
    }
    // Anti Wind-Up
    /*
    if (output == pwm_max || output == pwm_min) {
    	integral -= error; // riduci l'accumulo integrale
    }
    */
    return output;
}

void Control_Loop(float setpoint) {

    // Leggi la corrente attuale dal sensore
    current_value = INA219_ReadCurrent(&ina219);

    // Applica il controllo PID per regolare il PWM in base alla corrente
    pwm_value = PI_Control(setpoint, current_value);

    // Imposta il valore di PWM calcolato per il servo
    PCA9685_SetPin(4, pwm_value, 0);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void _putchar(char c)
{
  ITM_SendChar(c);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 4);  // Avvia l'ADC in modalità DMA

  PCA9685_Init(&hi2c1);
  while(!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS, 0.500)){}

  // Debug I2C
  if (HAL_I2C_IsDeviceReady(&hi2c1, PCA9685_ADDRESS, 1, HAL_MAX_DELAY) != HAL_OK) {
        // Errore: il dispositivo non risponde
        Error_Handler();
    }

  if (HAL_I2C_IsDeviceReady(&hi2c1, INA219_ADDRESS<<1, 1, HAL_MAX_DELAY) != HAL_OK) {
      // Errore: il dispositivo non risponde
      Error_Handler();
  }

  PCA9685_SetServoAngle(0, 0);
  PCA9685_SetServoAngle(1, 0);
  PCA9685_SetServoAngle(2, 0);
  PCA9685_SetServoAngle(3, 0);
  PCA9685_SetServoAngle(4, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  angles[0] = (uint8_t)((adcValues[0]*180)/4096);
	  angles[1] = (uint8_t)((adcValues[1]*180)/4096);
	  angles[2] = (uint8_t)((adcValues[2]*180)/4096);
	  angles[3] = (uint8_t)((adcValues[3]*180)/4096);

	  PCA9685_SetServoAngle(0, angles[0]);
	  PCA9685_SetServoAngle(1, angles[1]);
	  PCA9685_SetServoAngle(2, angles[2]);
	  PCA9685_SetServoAngle(3, angles[3]);


	  if(gripper == 1){
		  setpoint = 0.300;
		  Control_Loop(setpoint);
	  }
	  else if (gripper == 0){
		  setpoint = 0;
		  PCA9685_SetServoAngle(4, 0);
	  }

	  printf("PWM: %f\n", pwm_value);
	  printf("ERROR: %f\n", error);
	  printf("CURRENT: %f\n", current_value);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
