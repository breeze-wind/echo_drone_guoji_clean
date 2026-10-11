/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 3 electromagnets controlled by one button
  ******************************************************************************
  *
  * Function:
  *   press button once: state 0 -> 1 -> 2 -> 3 -> 0
  *
  *   state 0: M1 M2 M3 all work
  *   state 1: M1 stops, M2 M3 work
  *   state 2: M1 M2 stop, M3 works
  *   state 3: M1 M2 M3 all stop
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "gpio.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PD */

/* Button pin: PA0, pull-up input, button connects PA0 to GND when pressed. */
#define APP_KEY_GPIO_Port          GPIOA
#define APP_KEY_Pin                GPIO_PIN_0
#define KEY_PRESSED_LEVEL          GPIO_PIN_RESET
#define KEY_DEBOUNCE_MS            20U

/* Recommended magnet control pins. Configure these pins as GPIO_Output in CubeMX. */
#define APP_MAG1_GPIO_Port         GPIOB
#define APP_MAG1_Pin               GPIO_PIN_0
#define APP_MAG2_GPIO_Port         GPIOB
#define APP_MAG2_Pin               GPIO_PIN_1
#define APP_MAG3_GPIO_Port         GPIOB
#define APP_MAG3_Pin               GPIO_PIN_10

/*
 * Driver trigger type:
 *   1: high level = magnet works. This is common for MOSFET driver modules.
 *   0: low level  = magnet works. This is common for some relay modules.
 *
 * If the output is reversed during testing, only change this line.
 */
#define DRIVER_ACTIVE_HIGH         1

#if DRIVER_ACTIVE_HIGH
  #define MAGNET_WORK_LEVEL        GPIO_PIN_SET
  #define MAGNET_STOP_LEVEL        GPIO_PIN_RESET
#else
  #define MAGNET_WORK_LEVEL        GPIO_PIN_RESET
  #define MAGNET_STOP_LEVEL        GPIO_PIN_SET
#endif

#define MAX_DISABLE_COUNT          3U

/* USER CODE END PD */

/* USER CODE BEGIN PV */

static uint8_t magnet_disable_count = 0;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */

static void Magnet_ApplyState(uint8_t disabled_count);
static void Magnet_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t should_work);
static uint8_t Button_GetPressedOnce(void);

/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  Magnet_ApplyState(magnet_disable_count);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (Button_GetPressedOnce())
    {
      magnet_disable_count++;

      if (magnet_disable_count > MAX_DISABLE_COUNT)
      {
        magnet_disable_count = 0;
      }

      Magnet_ApplyState(magnet_disable_count);
    }
  }
  /* USER CODE END WHILE */
}

/* USER CODE BEGIN 4 */

static void Magnet_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t should_work)
{
  HAL_GPIO_WritePin(port, pin, should_work ? MAGNET_WORK_LEVEL : MAGNET_STOP_LEVEL);
}

static void Magnet_ApplyState(uint8_t disabled_count)
{
  if (disabled_count > MAX_DISABLE_COUNT)
  {
    disabled_count = MAX_DISABLE_COUNT;
  }

  Magnet_Write(APP_MAG1_GPIO_Port, APP_MAG1_Pin, disabled_count < 1U);
  Magnet_Write(APP_MAG2_GPIO_Port, APP_MAG2_Pin, disabled_count < 2U);
  Magnet_Write(APP_MAG3_GPIO_Port, APP_MAG3_Pin, disabled_count < 3U);
}

static uint8_t Button_GetPressedOnce(void)
{
  if (HAL_GPIO_ReadPin(APP_KEY_GPIO_Port, APP_KEY_Pin) != KEY_PRESSED_LEVEL)
  {
    return 0;
  }

  HAL_Delay(KEY_DEBOUNCE_MS);

  if (HAL_GPIO_ReadPin(APP_KEY_GPIO_Port, APP_KEY_Pin) != KEY_PRESSED_LEVEL)
  {
    return 0;
  }

  while (HAL_GPIO_ReadPin(APP_KEY_GPIO_Port, APP_KEY_Pin) == KEY_PRESSED_LEVEL)
  {
  }

  HAL_Delay(KEY_DEBOUNCE_MS);
  return 1;
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
