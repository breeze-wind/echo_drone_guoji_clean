/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 3-channel electromagnet controller
  ******************************************************************************
  *
  * Hardware:
  *   MCU: STM32F103C8T6 minimum system board
  *   Driver: N-MOSFET module, high-level trigger
  *
  * Pin map:
  *   PA0  -> button input, pull-up, pressed = low
  *   PB0  -> electromagnet 1 driver IN
  *   PB1  -> electromagnet 2 driver IN
  *   PB10 -> electromagnet 3 driver IN
  *
  * State:
  *   0: M1 M2 M3 work
  *   1: M1 stops, M2 M3 work
  *   2: M1 M2 stop, M3 works
  *   3: M1 M2 M3 stop
  *
  * N-MOSFET high-level trigger:
  *   GPIO_PIN_SET   = magnet works
  *   GPIO_PIN_RESET = magnet stops
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "gpio.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PD */

#define APP_KEY_GPIO_Port          GPIOA
#define APP_KEY_Pin                GPIO_PIN_0
#define APP_KEY_PRESSED_LEVEL      GPIO_PIN_RESET
#define APP_KEY_DEBOUNCE_MS        20U

#define APP_MAG1_GPIO_Port         GPIOB
#define APP_MAG1_Pin               GPIO_PIN_0
#define APP_MAG2_GPIO_Port         GPIOB
#define APP_MAG2_Pin               GPIO_PIN_1
#define APP_MAG3_GPIO_Port         GPIOB
#define APP_MAG3_Pin               GPIO_PIN_10

#define APP_MAGNET_WORK_LEVEL      GPIO_PIN_SET
#define APP_MAGNET_STOP_LEVEL      GPIO_PIN_RESET
#define APP_MAX_STATE              3U

/* USER CODE END PD */

/* USER CODE BEGIN PV */

static uint8_t app_state = 0;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */

static void App_SetMagnet(GPIO_TypeDef *port, uint16_t pin, uint8_t work);
static void App_ApplyState(uint8_t state);
static uint8_t App_ButtonPressedOnce(void);

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

  App_ApplyState(app_state);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (App_ButtonPressedOnce())
    {
      app_state++;

      if (app_state > APP_MAX_STATE)
      {
        app_state = 0;
      }

      App_ApplyState(app_state);
    }
  }
  /* USER CODE END WHILE */
}

/* USER CODE BEGIN 4 */

static void App_SetMagnet(GPIO_TypeDef *port, uint16_t pin, uint8_t work)
{
  HAL_GPIO_WritePin(port, pin, work ? APP_MAGNET_WORK_LEVEL : APP_MAGNET_STOP_LEVEL);
}

static void App_ApplyState(uint8_t state)
{
  if (state > APP_MAX_STATE)
  {
    state = APP_MAX_STATE;
  }

  App_SetMagnet(APP_MAG1_GPIO_Port, APP_MAG1_Pin, state < 1U);
  App_SetMagnet(APP_MAG2_GPIO_Port, APP_MAG2_Pin, state < 2U);
  App_SetMagnet(APP_MAG3_GPIO_Port, APP_MAG3_Pin, state < 3U);
}

static uint8_t App_ButtonPressedOnce(void)
{
  if (HAL_GPIO_ReadPin(APP_KEY_GPIO_Port, APP_KEY_Pin) != APP_KEY_PRESSED_LEVEL)
  {
    return 0;
  }

  HAL_Delay(APP_KEY_DEBOUNCE_MS);

  if (HAL_GPIO_ReadPin(APP_KEY_GPIO_Port, APP_KEY_Pin) != APP_KEY_PRESSED_LEVEL)
  {
    return 0;
  }

  while (HAL_GPIO_ReadPin(APP_KEY_GPIO_Port, APP_KEY_Pin) == APP_KEY_PRESSED_LEVEL)
  {
  }

  HAL_Delay(APP_KEY_DEBOUNCE_MS);
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
