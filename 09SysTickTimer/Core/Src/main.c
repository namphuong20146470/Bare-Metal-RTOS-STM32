/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - ADC with FreeRTOS
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
#include "usart.h"
#include "adc.h"
#include "i2c.h"  // Add I2C header
// #include "spi.h"   // Add SPI header
#include <string.h>
#include "systickTimer.h" //
/* USER CODE END Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TaskHandle_t adcTaskHandle;
TaskHandle_t sht30TaskHandle;
// TaskHandle_t loraTaskHandle; 
TaskHandle_t ledBlinkTaskHandle; 
char buffer[30];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vAdcTask(void *pvParameters);       // Task for ADC reading
void vSHT30Task(void *pvParameters);  
// void vLoRaTask(void *pvParameters); 
void vLedBlinkTask(void *pvParameters);  // Task for LED blinking

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
  // Initialize UART and ADC
  USART2_TXRX_Init();
ADC1_Init();
I2C1_Init();
I2C_ScanBus();
// Initialize I2C and SHT30 sensor
if (SHT30_Init() == 0) {
  USART2_SendString("SHT30 sensor initialized successfully\r\n");
} else {
  USART2_SendString("SHT30 sensor initialization failed\r\n");
}
LED_PA5_Init();
// Initialize SysTick for 1ms ticks (assuming 84MHz processor clock)
SysTick_Init(84000); // 84MHz / 1000Hz = 84000 ticks per ms

USART2_SendString("FreeRTOS ADC, SHT30, and LoRa Demo\r\n");

// Create the tasks
status = xTaskCreate(vAdcTask, "ADC Task", 200, NULL, 1, &adcTaskHandle);
configASSERT(status == pdPASS);

status = xTaskCreate(vSHT30Task, "SHT30 Task", 256, NULL, 1, &sht30TaskHandle);
configASSERT(status == pdPASS);

// Create the LoRa task
// status = xTaskCreate(vLoRaTask, "LoRa Task", 256, NULL, 1, &loraTaskHandle);
// configASSERT(status == pdPASS);
// Create the LED Blink task
status = xTaskCreate(vLedBlinkTask, "LED Blink Task", 128, NULL, 1, &ledBlinkTaskHandle);
configASSERT(status == pdPASS);

// Start the scheduler
vTaskStartScheduler();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* This code will not be reached if the scheduler is running */
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

/* ADC Task */
void vAdcTask(void *pvParameters)
{
  uint32_t lastWakeTime = xTaskGetTickCount();
  
  while (1)
  {
    // Read ADC value
    uint16_t adcValue = ADC1_Read();
    
    // Convert to millivolts
    uint32_t millivolts = (adcValue * 3300) / 4096;
    uint16_t whole = millivolts / 1000;
    uint16_t frac = (millivolts % 1000) / 10; // Two decimal places
    
    // Format and send over UART
    sprintf(buffer, "ADC: %u raw, %u.%02u V\r\n", adcValue, whole, frac);
    USART2_SendString(buffer);
    
    // Delay for 500ms with better timing precision
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(500));
  }
}
/* SHT30 Task */
/* SHT30 Task */
/* SHT30 Task */
void vSHT30Task(void *pvParameters)
{
  float temperature = 0.0f;
  float humidity = 0.0f;
  uint8_t result;
  uint32_t lastWakeTime = xTaskGetTickCount();
  char sht30_buffer[50]; // Buffer riêng cho task này
  
  while (1)
  {
    // Read temperature and humidity
    result = SHT30_Read_Temperature_Humidity(&temperature, &humidity);
    
    if (result == 0) {
      // Chuyển đổi float thành phần nguyên và phần thập phân
      int temp_int = (int)temperature;
      int temp_frac = (int)((temperature - temp_int) * 100);
      if (temp_frac < 0) temp_frac = -temp_frac;
      
      int hum_int = (int)humidity;
      int hum_frac = (int)((humidity - hum_int) * 100);
      
      // Format với số nguyên
      sprintf(sht30_buffer, "SHT30: %d.%02d C, %d.%02d %%RH\r\n", 
              temp_int, temp_frac, hum_int, hum_frac);
      USART2_SendString(sht30_buffer);
    } else {
      // Error reading sensor
      sprintf(sht30_buffer, "SHT30 read error: %u\r\n", result);
      USART2_SendString(sht30_buffer);
    }
    
    // Delay for 2 seconds
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(2000));
  }
}
/* LoRa Task to send data to ESP32 via SX1278 */
// void vLoRaTask(void *pvParameters)
// {
//     uint32_t lastWakeTime;
//     char lora_buffer[60];
//     uint8_t packet_number = 0;
//     float temperature = 0.0f;
//     float humidity = 0.0f;
//     uint16_t adcValue = 0;
    
//     // Wait a bit before initializing SX1278
//     vTaskDelay(pdMS_TO_TICKS(1000));
    
//     USART2_SendString("Starting LoRa task initialization...\r\n");
    
//     // Initialize SX1278 LoRa module (using register-level implementation)
//     if (SX1278_Init() != 0) {
//         USART2_SendString("Failed to initialize SX1278 module\r\n");
//         vTaskDelete(NULL); // Delete this task if initialization fails
//     }
    
//     // Wait for 1 second after initialization
//     vTaskDelay(pdMS_TO_TICKS(1000));
    
//     // Initialize lastWakeTime after all setup is done
//     lastWakeTime = xTaskGetTickCount();
//     USART2_SendString("LoRa task entering main loop\r\n");
    
//     // First packet before delay
//     packet_number = 1; // Start with packet 1
    
//     // Initial packet send - don't wait
//     // Get latest sensor readings
//     adcValue = ADC1_Read();
//     SHT30_Read_Temperature_Humidity(&temperature, &humidity);
    
//     // Format data packet
//     sprintf(lora_buffer, "PKT,%d,%d,%.1f,%.1f", 
//             packet_number, adcValue, temperature, humidity);
    
//     // Send immediately
//     USART2_SendString("Sending FIRST LoRa packet: ");
//     USART2_SendString(lora_buffer);
//     USART2_SendString("\r\n");
    
//     USART2_SendString("Calling SX1278_Send for first packet...\r\n");
//     SX1278_Send((uint8_t*)lora_buffer, strlen(lora_buffer));
//     USART2_SendString("First SX1278_Send completed\r\n");
    
//     packet_number++;
    
//     while (1)
//     {
//         // Get latest sensor readings
//         adcValue = ADC1_Read();
        
//         // Calculate voltage and display it
//         uint32_t millivolts = (adcValue * 3300) / 4096;
//         char voltage_str[20];
//         sprintf(voltage_str, "Voltage: %lu.%lu V\r\n", millivolts/1000, (millivolts%1000)/10);
//         USART2_SendString(voltage_str);
        
//         // Get SHT30 readings
//         SHT30_Read_Temperature_Humidity(&temperature, &humidity);
        
//         // Create data packet to send to ESP32 - simplified decimal places
//         sprintf(lora_buffer, "PKT,%d,%d,%.1f,%.1f", 
//                 packet_number, adcValue, temperature, humidity);
        
//         // Send data via LoRa
//         USART2_SendString("Sending LoRa packet: ");
//         USART2_SendString(lora_buffer);
//         USART2_SendString("\r\n");
        
//         // Send the packet via SX1278
//         USART2_SendString("Calling SX1278_Send...\r\n");
//         SX1278_Send((uint8_t*)lora_buffer, strlen(lora_buffer));
//         USART2_SendString("SX1278_Send completed\r\n");
        
//         // Increment packet number
//         packet_number++;
        
//         // Delay for 5 seconds before next transmission
//         USART2_SendString("Waiting 5 seconds before next transmission...\r\n");
//         vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5000));
//     }
// }
/* LED Blink Task using SysTick timer */
void vLedBlinkTask(void *pvParameters)
{
    uint32_t lastWakeTime = xTaskGetTickCount();
    char led_buffer[30];
    
    USART2_SendString("LED Blink Task Started\r\n");
    
    while (1)
    {
        // Toggle LED
        LED_PA5_Toggle();
        
        // Print status
        sprintf(led_buffer, "LED toggled at %lu ms\r\n", SysTick_GetTick());
        USART2_SendString(led_buffer);
        
        // Different ON/OFF times using direct timing
        if (GPIOA_ODR & GPIO_PIN_5) {
            // LED is ON - we'll delay using FreeRTOS task delay (non-blocking)
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            // LED is OFF - we'll delay using FreeRTOS task delay (non-blocking)
            vTaskDelay(pdMS_TO_TICKS(800));
        }
        
        // Additional delay to maintain overall cycle timing
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000));
    }
}
/* FreeRTOS Hook Functions */
void vApplicationIdleHook(void)
{
  /* This function is called when the idle task is running */
}

void vApplicationMallocFailedHook(void)
{
  /* This function is called when memory allocation fails */
  USART2_SendString("Memory allocation failed!\r\n");
  Error_Handler();
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM9)
  {
    HAL_IncTick();
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
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */