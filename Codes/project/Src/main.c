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
#include "hrtim.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myself_Oled.h"
#include "myself_pid.h"
#include "myself_spll_sogi.h"
#include "myself_droop_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP        400.f
#define PI          3.14159265358979323846f

// hrtim
#define MEDIAN      28800.f
#define AMPLITUDE   28800.f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float Vref_Peak = 20;
float Iref_Peak = 0.2/0.6;
float Vref = 0;
float Iref = 0;
float Wref = 2*PI/STEP;

float Theta = 0;
float Modulating_Wave = 0;

uint16_t Adc1_Vlaue[4] = {0};
float U_In_Normal   = 0;
float U_Out_Normal  = 0;
float I_In_Normal   = 0;
float I_Out_Normal  = 0;

float U_In  = 30;
float U_Out = 0;
// float I_In  = 0;
// float I_Out = 0;

PidPosition Pid_V;
PidPosition Pid_I;

SinglePhase Sample_I;
SinglePhase Sample_U;
SinglePhasePower Power_Out;
DroopControl Droop;

Pll Pll_U;
uint8_t Pll_Flag = 0;

float Monito = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx)
{
  if ((hhrtim == &hhrtim1)&&(TimerIdx == HRTIM_TIMERINDEX_TIMER_C))
  {    
    // // open loop
    // Modulating_Wave = 0.95f * sinf(Theta);

    if (Pll_Flag)
    {
      // voltage loop
      Vref = (Vref_Peak/U_In) * sinf(Theta);
      Droop.out_theta = Theta;
    }
    else
    {
      // droop
      Droop.amplitude = Vref_Peak/U_In;
      Vref = Droop_Control_Run(&Droop, Power_Out.power_pq);
    }
    Pid_V.set_value = Vref;

    Iref = PID_Position_Run(&Pid_V, U_Out_Normal);
    Iref = Limit_Value(Iref, 0.5, -0.5);
    
    // current loop
    // Iref = Iref_Peak * sinf(Theta); 
    Pid_I.set_value = Iref;
    Modulating_Wave = PID_Position_Run(&Pid_I, I_Out_Normal);
    
    // feedforward
    Modulating_Wave += U_Out_Normal;
    Modulating_Wave = Limit_Value(Modulating_Wave, 0.95, -0.95);

    // pwm
    int PWM_compare1 = MEDIAN - AMPLITUDE*(1 + Modulating_Wave)/2;
    int PWM_compare2 = MEDIAN + AMPLITUDE*(1 + Modulating_Wave)/2;
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x2, HRTIM_COMPAREUNIT_1, PWM_compare1);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, 0x2, HRTIM_COMPAREUNIT_2, PWM_compare2);

    // Theta += Wref;
    // Theta = fmod(Theta, 2*PI);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc == &hadc1)
  {
    // normalization
    U_In_Normal  = ((float)Adc1_Vlaue[0]/4096.f);
    U_Out_Normal = ((float)Adc1_Vlaue[1]-2048.f)/2048.f;
    I_In_Normal  = ((float)Adc1_Vlaue[2]-2048.f)/2048.f;
    I_Out_Normal = ((float)Adc1_Vlaue[3]-2048.f)/2048.f;

    U_In = U_In_Normal *3.3f * 30.f;
    U_Out = U_Out_Normal*1.65f*30;
    // I_in  = (I_In_Normal *1.65f)/0.132f;
    // I_Out = (I_Out_Normal*1.65f)/0.132f;

    Sample_U.value = U_Out_Normal;
    Sample_I.value = I_Out_Normal;
    
    Single_Phase_Calculated_Power(&Power_Out, Sample_U, Sample_I);

    if (Pll_Flag)
    {
      PLL_Run(&Pll_U, Power_Out.sogi_u.clarke, Wref);
      Theta = Pll_U.lock_theta;
    }
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_HRTIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  // oled
  WS_OLED_Init();

  // pid
  PID_Position_Init(&Pid_V, Vref, 1, 0.005, 0);
  PID_Position_Init(&Pid_I, Iref, 1, 0, 0);

  // power
  Single_Phase_Power_Init(&Power_Out);
  Sample_U.w = Wref;
  Sample_I.w = Wref;

  // pll
  PLL_Init(&Pll_U);

  // droop
  Droop_Control_Init(&Droop);

  // adc
  HAL_ADCEx_Calibration_Start(&hadc1, 0xffffffff);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc1_Vlaue, 4);

  // pwm
  HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_TIMER_C);
  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_C);

  HAL_Delay(100);
  for (uint32_t i = 0xffffff; i; i--)
  {
    if((U_Out > 3)||(U_Out < 3))
    {
      Pll_Flag = 1;
      break;
    }
  }
  if (Pll_Flag)
  {
    HAL_Delay(500);
    Pll_Flag = 0;
  }
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    WS_OLED_Printf(1, 1, 0, "U_In  = %.2f", U_In);
    WS_OLED_Printf(1, 2, 0, "P     = %f", Power_Out.power_pq.p);
    WS_OLED_Printf(1, 3, 0, "Q     = %f", Power_Out.power_pq.q);
    // WS_OLED_Printf(1, 4, 0, "I_Out = %.2f", (I_Out_Normal*1.65f)/0.132f);
    WS_OLED_Printf(1, 5, 0, "q     = %f", Pll_U.park.q);
    // WS_OLED_Printf(1, 6, 0, "Iref  = %.2f", Iref);
    // WS_OLED_Printf(1, 7, 0, "Monito= %.2f", Monito);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
