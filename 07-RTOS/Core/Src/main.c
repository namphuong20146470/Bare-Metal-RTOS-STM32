/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - RGB LED Control with FreeRTOS (Simple State Machine)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// LED state enum
typedef enum
{
  LED_STATE_RED = 0,
  LED_STATE_BLUE,
  LED_STATE_GREEN
} LedState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE 128
#define GPIOAEN (1UL << 0) // GPIOA clock enable
#define GPIOCEN (1UL << 2) // GPIOC clock enable

// LED Pins on GPIOA
#define RED_LED_PIN (1UL << 8)    // RED LED on PA8
#define BLUE_LED_PIN (1UL << 9)   // BLUE LED on PA9
#define GREEN_LED_PIN (1UL << 10) // GREEN LED on PA10

// LED states
#define LED_ON 1
#define LED_OFF 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// Global variable to track the current LED state
volatile LedState_t currentLedState = LED_STATE_RED;

// Task handles
TaskHandle_t redTaskHandle;
TaskHandle_t blueTaskHandle;
TaskHandle_t greenTaskHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vRedLedTask(void *pvParameters);   // Task for RED LED
void vBlueLedTask(void *pvParameters);  // Task for BLUE LED
void vGreenLedTask(void *pvParameters); // Task for GREEN LED
void SEGGER_UART_init(uint32_t baud);
void SEGGER_SYSVIEW_Conf(void);
void SEGGER_SYSVIEW_PrintfTarget(const char *fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DWT_CTRL (*(volatile uint32_t *)0xE0001000)

// Helper function to control LED - updated with proper ON/OFF control
void controlLed(uint32_t pin, uint8_t state)
{
  if (state == LED_ON)
  {
    GPIOA->BSRR = pin; // Set bit (turn ON LED)
  }
  else
  {
    GPIOA->BSRR = (pin << 16); // Reset bit (turn OFF LED)
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  BaseType_t status;
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
  /* USER CODE BEGIN 2 */
  // Enable GPIO clocks
  RCC->AHB1ENR |= GPIOAEN | GPIOCEN;

  // Configure PA8, PA9, PA10 (RGB LEDs) as outputs
  GPIOA->MODER &= ~(3UL << (8 * 2)); // Clear mode bits for PA8
  GPIOA->MODER |= (1UL << (8 * 2));  // Set PA8 to output mode (RED)

  GPIOA->MODER &= ~(3UL << (9 * 2)); // Clear mode bits for PA9
  GPIOA->MODER |= (1UL << (9 * 2));  // Set PA9 to output mode (BLUE)

  GPIOA->MODER &= ~(3UL << (10 * 2)); // Clear mode bits for PA10
  GPIOA->MODER |= (1UL << (10 * 2));  // Set PA10 to output mode (GREEN)

  // Configure PC13 (Button) as input with pull-up
  GPIOC->MODER &= ~(3UL << (13 * 2)); // Clear mode bits for pin 13 (input mode = 00)
  GPIOC->PUPDR &= ~(3UL << (13 * 2)); // Clear pull-up/pull-down
  GPIOC->PUPDR |= (1UL << (13 * 2));  // Set to pull-up (01)

  // Ensure all LEDs are OFF initially
  controlLed(RED_LED_PIN, LED_OFF);
  controlLed(BLUE_LED_PIN, LED_OFF);
  controlLed(GREEN_LED_PIN, LED_OFF);

  // Initialize Segger debug
  DWT_CTRL |= (1 << 0);     // Enable DWT
  SEGGER_UART_init(500000); // Initialize SEGGER UART
  SEGGER_SYSVIEW_Conf();    // Initialize SEGGER SystemView

  /* Create the three LED tasks */
  status = xTaskCreate(vRedLedTask, "Task 1", 200, "Hello world task 1 ", 2, &redTaskHandle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(vBlueLedTask, "Task 2", 200, "Hello world task 2 ", 2, &blueTaskHandle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(vGreenLedTask, "Task 3", 200, "Hello world task 3 ", 2, &greenTaskHandle);
  configASSERT(status == pdPASS);

  // Initially suspend the BLUE and GREEN tasks, start with RED only
  vTaskSuspend(blueTaskHandle);
  vTaskSuspend(greenTaskHandle);

  // Start the scheduler
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Code will never reach here if the scheduler is running */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
/* RED LED Task */
void vRedLedTask(void *pvParameters)
{
  while (1)
  {
    // Turn ON the RED LED
    SEGGER_SYSVIEW_PrintfTarget("RED LED ON - Only RED Task running");
    controlLed(RED_LED_PIN, LED_ON);

    // Keep LED ON for exactly 1 second without using vTaskDelay
    // Instead, keep track of time manually
    TickType_t xStartTime = xTaskGetTickCount();
    while ((xTaskGetTickCount() - xStartTime) < pdMS_TO_TICKS(1000))
    {
      // Do nothing, just wait
    }

    // Turn OFF the RED LED
    controlLed(RED_LED_PIN, LED_OFF);

    // Critical section to ensure atomic operations
    taskENTER_CRITICAL();
    // Resume BLUE task first
    vTaskResume(blueTaskHandle);
    // Then immediately suspend self
    vTaskSuspend(NULL);
    taskEXIT_CRITICAL();
  }
}

/* BLUE LED Task */
void vBlueLedTask(void *pvParameters)
{
  while (1)
  {
    // Turn ON the BLUE LED
    SEGGER_SYSVIEW_PrintfTarget("BLUE LED ON - Only BLUE Task running");
    controlLed(BLUE_LED_PIN, LED_ON);

    // Keep LED ON for exactly 1 second without using vTaskDelay
    TickType_t xStartTime = xTaskGetTickCount();
    while ((xTaskGetTickCount() - xStartTime) < pdMS_TO_TICKS(1000))
    {
      // Do nothing, just wait
    }

    // Turn OFF the BLUE LED
    controlLed(BLUE_LED_PIN, LED_OFF);

    // Critical section to ensure atomic operations
    taskENTER_CRITICAL();
    // Resume GREEN task first
    vTaskResume(greenTaskHandle);
    // Then immediately suspend self
    vTaskSuspend(NULL);
    taskEXIT_CRITICAL();
  }
}

/* GREEN LED Task */
void vGreenLedTask(void *pvParameters)
{
  while (1)
  {
    // Turn ON the GREEN LED
    SEGGER_SYSVIEW_PrintfTarget("GREEN LED ON - Only GREEN Task running");
    controlLed(GREEN_LED_PIN, LED_ON);

    // Keep LED ON for exactly 1 second without using vTaskDelay
    TickType_t xStartTime = xTaskGetTickCount();
    while ((xTaskGetTickCount() - xStartTime) < pdMS_TO_TICKS(1000))
    {
      // Do nothing, just wait
    }

    // Turn OFF the GREEN LED
    controlLed(GREEN_LED_PIN, LED_OFF);

    // Critical section to ensure atomic operations
    taskENTER_CRITICAL();
    // Resume RED task first
    vTaskResume(redTaskHandle);
    // Then immediately suspend self
    vTaskSuspend(NULL);
    taskEXIT_CRITICAL();
  }
}
/* FreeRTOS Hook Functions */
void vApplicationIdleHook(void)
{
  /* This function is called when the idle task is running */
  /* You can add power saving code here */
}

void vApplicationMallocFailedHook(void)
{
  /* This function is called when memory allocation fails */
  /* Handle error here, e.g., restart the system */
  printf("Memory allocation failed!\r\n");
  Error_Handler();
}
/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called when TIM9 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef USE_FULL_ASSERT
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