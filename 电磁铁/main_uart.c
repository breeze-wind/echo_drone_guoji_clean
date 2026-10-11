/* UART version of the electromagnet controller. */
#include "main.h"
#include "gpio.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

void SystemClock_Config(void);

#define APP_MAX_STATE 3U
#define APP_UART_RX_BUFFER_SIZE 128U
#define APP_UART_LINE_MAX 32U

#define APP_KEY_GPIO_Port GPIOA
#define APP_KEY_Pin GPIO_PIN_0
#define APP_KEY_PRESSED_LEVEL GPIO_PIN_RESET
#define APP_KEY_DEBOUNCE_MS 20U

#define APP_MAG1_GPIO_Port GPIOB
#define APP_MAG1_Pin GPIO_PIN_0
#define APP_MAG2_GPIO_Port GPIOB
#define APP_MAG2_Pin GPIO_PIN_1
#define APP_MAG3_GPIO_Port GPIOB
#define APP_MAG3_Pin GPIO_PIN_10

/* High-level trigger: GPIO high means the magnet works. */
#define APP_MAGNET_WORK_LEVEL GPIO_PIN_SET
#define APP_MAGNET_STOP_LEVEL GPIO_PIN_RESET
#define APP_UART_TX_TIMEOUT_MS 100U

static uint8_t app_state = 0U;
static GPIO_PinState app_key_stable_level;
static GPIO_PinState app_key_sample_level;
static uint32_t app_key_last_change_tick;

static uint8_t app_uart_rx_byte;
static volatile uint8_t app_uart_rx_buffer[APP_UART_RX_BUFFER_SIZE];
static volatile uint16_t app_uart_rx_head = 0U;
static volatile uint16_t app_uart_rx_tail = 0U;
static volatile uint8_t app_uart_rx_overflow = 0U;

static void App_SetMagnet(GPIO_TypeDef *port, uint16_t pin, uint8_t work);
static void App_ApplyState(uint8_t state);
static void App_ButtonInit(void);
static uint8_t App_ButtonPressedOnce(void);
static void App_SetState(uint8_t state);
static void App_NextState(void);
static void App_UartStartReceive(void);
static uint8_t App_UartReadByte(uint8_t *value);
static void App_UartPoll(void);
static void App_UartSend(const char *text);
static void App_SendState(void);
static void App_ProcessCommand(const char *line);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();

  App_ButtonInit();
  App_ApplyState(app_state);
  App_UartStartReceive();

  App_UartSend("READY\r\n");
  App_SendState();

  while (1)
  {
    App_UartPoll();

    if (App_ButtonPressedOnce())
    {
      App_NextState();
      App_SendState();
    }
  }
}

static void App_SetMagnet(GPIO_TypeDef *port, uint16_t pin, uint8_t work)
{
  HAL_GPIO_WritePin(
      port,
      pin,
      work ? APP_MAGNET_WORK_LEVEL : APP_MAGNET_STOP_LEVEL);
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

static void App_ButtonInit(void)
{
  app_key_stable_level = HAL_GPIO_ReadPin(APP_KEY_GPIO_Port, APP_KEY_Pin);
  app_key_sample_level = app_key_stable_level;
  app_key_last_change_tick = HAL_GetTick();
}

static uint8_t App_ButtonPressedOnce(void)
{
  GPIO_PinState sample_level =
      HAL_GPIO_ReadPin(APP_KEY_GPIO_Port, APP_KEY_Pin);
  uint32_t now = HAL_GetTick();

  if (sample_level != app_key_sample_level)
  {
    app_key_sample_level = sample_level;
    app_key_last_change_tick = now;
  }

  if ((app_key_sample_level != app_key_stable_level) &&
      ((now - app_key_last_change_tick) >= APP_KEY_DEBOUNCE_MS))
  {
    app_key_stable_level = app_key_sample_level;

    if (app_key_stable_level == APP_KEY_PRESSED_LEVEL)
    {
      return 1U;
    }
  }

  return 0U;
}

static void App_SetState(uint8_t state)
{
  if (state > APP_MAX_STATE)
  {
    return;
  }

  app_state = state;
  App_ApplyState(app_state);
}

static void App_NextState(void)
{
  app_state++;

  if (app_state > APP_MAX_STATE)
  {
    app_state = 0U;
  }

  App_ApplyState(app_state);
}

static void App_UartStartReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &app_uart_rx_byte, 1U);
}

static uint8_t App_UartReadByte(uint8_t *value)
{
  uint16_t tail = app_uart_rx_tail;

  if (tail == app_uart_rx_head)
  {
    return 0U;
  }

  *value = app_uart_rx_buffer[tail];
  app_uart_rx_tail =
      (uint16_t)((tail + 1U) % APP_UART_RX_BUFFER_SIZE);

  return 1U;
}

static void App_UartSend(const char *text)
{
  (void)HAL_UART_Transmit(
      &huart1,
      (uint8_t *)text,
      (uint16_t)strlen(text),
      APP_UART_TX_TIMEOUT_MS);
}

static void App_SendState(void)
{
  char response[48];

  (void)snprintf(
      response,
      sizeof(response),
      "OK STATE=%u ACTIVE=%u\r\n",
      (unsigned int)app_state,
      (unsigned int)(3U - app_state));

  App_UartSend(response);
}

static void App_ProcessCommand(const char *line)
{
  if (strcmp(line, "GET") == 0)
  {
    App_SendState();
    return;
  }

  if (strcmp(line, "NEXT") == 0)
  {
    App_NextState();
    App_SendState();
    return;
  }

  if (strncmp(line, "SET ", 4U) == 0)
  {
    const char *value = line + 4;

    if ((value[0] >= '0') &&
        (value[0] <= '4') &&
        (value[1] == '\0'))
    {
      App_SetState((uint8_t)(value[0] - '0'));
      App_SendState();
      return;
    }
  }

  App_UartSend("ERR COMMAND\r\n");
}

static void App_UartPoll(void)
{
  static char line[APP_UART_LINE_MAX];
  static uint8_t line_length = 0U;
  static uint8_t discard_line = 0U;
  uint8_t value;

  if (app_uart_rx_overflow != 0U)
  {
    app_uart_rx_overflow = 0U;
    App_UartSend("ERR RX_OVERFLOW\r\n");
  }

  while (App_UartReadByte(&value) != 0U)
  {
    if ((value == '\r') || (value == '\n'))
    {
      if (discard_line != 0U)
      {
        App_UartSend("ERR LINE_TOO_LONG\r\n");
      }
      else if (line_length > 0U)
      {
        line[line_length] = '\0';
        App_ProcessCommand(line);
      }

      line_length = 0U;
      discard_line = 0U;
      continue;
    }

    if (discard_line != 0U)
    {
      continue;
    }

    if (line_length < (APP_UART_LINE_MAX - 1U))
    {
      line[line_length++] = (char)value;
    }
    else
    {
      discard_line = 1U;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next_head =
        (uint16_t)((app_uart_rx_head + 1U) % APP_UART_RX_BUFFER_SIZE);

    if (next_head == app_uart_rx_tail)
    {
      app_uart_rx_overflow = 1U;
    }
    else
    {
      app_uart_rx_buffer[app_uart_rx_head] = app_uart_rx_byte;
      app_uart_rx_head = next_head;
    }

    App_UartStartReceive();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    App_UartStartReceive();
  }
}

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  (void)file;
  (void)line;
}
#endif
