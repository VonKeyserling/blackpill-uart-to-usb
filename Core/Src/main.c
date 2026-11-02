
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   STM32F401CCU6 - UART to USB CDC
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE 2048U
#define UART_BUFFER_MASK (UART_BUFFER_SIZE - 1U)
#define USB_PACKET_SIZE 64U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t uart_rx_byte;

static uint8_t uart_buffer[UART_BUFFER_SIZE];

static volatile uint16_t uart_head = 0;
static volatile uint16_t uart_tail = 0;

static volatile uint32_t uart_dropped = 0;

static uint8_t usb_tx_buffer[USB_PACKET_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void USB_Log_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Executado quando um byte chega pela USART2.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART2)
    return;

  uint16_t next = (uart_head + 1U) & UART_BUFFER_MASK;

  if (next != uart_tail) {
    uart_buffer[uart_head] = uart_rx_byte;
    uart_head = next;
  } else {
    uart_dropped++;
  }

  /* Receber próximo byte */
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

/*
 * Tratamento de erros da UART.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance != USART2)
    return;

  __HAL_UART_CLEAR_OREFLAG(huart);

  if (huart->RxState == HAL_UART_STATE_READY) {
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
  }
}

/*
 * Envia os dados do buffer UART para USB CDC.
 */
static void USB_Log_Task(void) {
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    return;

  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  if (hcdc == NULL || hcdc->TxState != 0)
    return;

  uint16_t index = uart_tail;
  uint16_t head_snapshot = uart_head;
  uint16_t count = 0;

  while ((index != head_snapshot) && (count < USB_PACKET_SIZE)) {
    usb_tx_buffer[count++] = uart_buffer[index];

    index = (index + 1U) & UART_BUFFER_MASK;
  }

  if (count == 0)
    return;

  /*
   * CDC_Transmit_FS é não bloqueante.
   * O buffer usb_tx_buffer permanece válido
   * até a transmissão ser concluída.
   */
  if (CDC_Transmit_FS(usb_tx_buffer, count) == USBD_OK) {
    uart_tail = index;
  }
}

/* USER CODE END 0 */

/**
 * @brief Application entry point.
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  uart_head = 0;
  uart_tail = 0;
  uart_dropped = 0;

  if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    USB_Log_Task();

    /* USER CODE END 3 */
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure voltage regulator */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* HSE + PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;

  RCC_OscInitStruct.HSEState = RCC_HSE_ON;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* 84 MHz CPU, 48 MHz USB */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief USART2 Initialization Function
 */
static void MX_USART2_UART_Init(void) {
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 */
static void MX_GPIO_Init(void) {
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief Error Handler
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */

  __disable_irq();

  while (1) {
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}

#endif
