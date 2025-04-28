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
#include "i2c.h"  // Include this to access I2C and SHT30 functions
#include <string.h>
#include "spi.h"
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
TaskHandle_t sx1278TaskHandle; // Add task handle for SX1278
char buffer[30];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vAdcTask(void *pvParameters);       // Task for ADC reading
void vSHT30Task(void *pvParameters);  
void vSX1278Task(void *pvParameters); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Stubs for I2C and SHT30 functions if not already defined elsewhere

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
  
  // Allow time for peripherals to stabilize
  HAL_Delay(500);
  
  // Initialize SX1278 LoRa module
  SX1278_Init();
  USART2_SendString("SX1278 LoRa module initialized\r\n");

  USART2_SendString("FreeRTOS ADC, SHT30 and SX1278 Demo\r\n");

  // Create the tasks
  status = xTaskCreate(vAdcTask, "ADC Task", 200, NULL, 1, &adcTaskHandle);
  configASSERT(status == pdPASS);
  
  status = xTaskCreate(vSHT30Task, "SHT30 Task", 256, NULL, 1, &sht30TaskHandle);
  configASSERT(status == pdPASS);
  
  // Create SX1278 LoRa task with larger stack
  status = xTaskCreate(vSX1278Task, "SX1278 Task", 512, NULL, 2, &sx1278TaskHandle);
  configASSERT(status == pdPASS);

  // Start the scheduler
  vTaskStartScheduler();

  /* USER CODE END 2 */

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
void vSHT30Task(void *pvParameters)
{
  float temperature = 0.0f;
  float humidity = 0.0f;
  uint8_t result;
  uint32_t lastWakeTime = xTaskGetTickCount();
  char sht30_buffer[50]; // Buffer for this task
  
  // Wait a bit before starting
  vTaskDelay(pdMS_TO_TICKS(100));
  
  while (1)
  {
    // Read temperature and humidity
    result = SHT30_Read_Temperature_Humidity(&temperature, &humidity);
    
    if (result == 0) {
      // Convert float to integer and fractional parts
      int temp_int = (int)temperature;
      int temp_frac = (int)((temperature - temp_int) * 100);
      if (temp_frac < 0) temp_frac = -temp_frac;
      
      int hum_int = (int)humidity;
      int hum_frac = (int)((humidity - hum_int) * 100);
      
      // Format output
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

/* SX1278 LoRa Task - Completely revised */
// Replace your current vSX1278Task with this diagnostic version

void vSX1278Task(void *pvParameters)
{
  // Allow ADC and SHT30 tasks to start first
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  uint32_t lastWakeTime;
  uint8_t txData[64];
  uint8_t step = 0;
  char lora_buffer[100];
  
  USART2_SendString("\r\n*** SX1278 DIAGNOSTIC TASK STARTED ***\r\n\r\n");
  
  // First run the hardware test to diagnose problems
  SX1278_HardwareTest();
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  // Try to perform emergency SPI bit-bang init
  SPI1_DirectGPIO_Init();
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Multiple hard resets to ensure module state is cleared
  USART2_SendString("Performing emergency reset sequence...\r\n");
  for (int i = 0; i < 5; i++) {
    // Reset low (PA10)
    GPIOA->BSRR = GPIO_BSRR_BR10;
    Delay_ms(200);
    
    // Reset high (PA10)
    GPIOA->BSRR = GPIO_BSRR_BS10;
    Delay_ms(400);
    
    USART2_SendString("Reset cycle complete\r\n");
  }
  vTaskDelay(pdMS_TO_TICKS(500));
  
  // Initialize manually with direct GPIO control
  USART2_SendString("Attempting emergency manual initialization...\r\n");
  
  // Step 1: Set Sleep Mode (FSK)
  SPI1_CS_Low();
  SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
  SPI1_DirectGPIO_Transfer(0x00);
  SPI1_CS_High();
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Read mode register
  SPI1_CS_Low();
  SPI1_DirectGPIO_Transfer(REG_OP_MODE & 0x7F);
  uint8_t mode = SPI1_DirectGPIO_Transfer(0x00);
  SPI1_CS_High();
  
  sprintf(lora_buffer, "Sleep mode set: 0x%02X (should be 0x00)\r\n", mode);
  USART2_SendString(lora_buffer);
  
  // Step 2: Set Sleep+LoRa mode
  SPI1_CS_Low();
  SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
  SPI1_DirectGPIO_Transfer(0x80);
  SPI1_CS_High();
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Read mode register
  SPI1_CS_Low();
  SPI1_DirectGPIO_Transfer(REG_OP_MODE & 0x7F);
  mode = SPI1_DirectGPIO_Transfer(0x00);
  SPI1_CS_High();
  
  sprintf(lora_buffer, "LoRa mode set: 0x%02X (should be 0x80)\r\n", mode);
  USART2_SendString(lora_buffer);
  
  // Step 3: Set Standby mode
  SPI1_CS_Low();
  SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
  SPI1_DirectGPIO_Transfer(0x81);
  SPI1_CS_High();
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Get initial timestamp
  lastWakeTime = xTaskGetTickCount();
  
  USART2_SendString("\r\n=== STARTING HARDWARE VERIFICATION LOOP ===\r\n");
  USART2_SendString("Will attempt a sequence of tests every 5 seconds\r\n\r\n");
  
  while (1)
  {
    step = (step + 1) % 5;  // Rotate through different tests
    
    switch (step) {
      case 0:
        // Version register test
        USART2_SendString("\r\nTEST 1: Version Register Read\r\n");
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_VERSION & 0x7F);
        uint8_t version = SPI1_DirectGPIO_Transfer(0x00);
        SPI1_CS_High();
        
        sprintf(lora_buffer, "Version register: 0x%02X (expect 0x12)\r\n", version);
        USART2_SendString(lora_buffer);
        break;
        
      case 1:
        // SYNC word write/read test
        USART2_SendString("\r\nTEST 2: SYNC Word Write/Read\r\n");
        
        // Write SYNC word
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_SYNC_WORD | 0x80);
        SPI1_DirectGPIO_Transfer(0x12);
        SPI1_CS_High();
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Read SYNC word
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_SYNC_WORD & 0x7F);
        uint8_t sync = SPI1_DirectGPIO_Transfer(0x00);
        SPI1_CS_High();
        
        sprintf(lora_buffer, "SYNC word: 0x%02X (wrote 0x12)\r\n", sync);
        USART2_SendString(lora_buffer);
        break;
        
      case 2:
        // Mode register test
        USART2_SendString("\r\nTEST 3: Mode Register Write/Read\r\n");
        
        // Sequence of mode changes
        uint8_t modes[] = {0x00, 0x80, 0x81};
        char* mode_names[] = {"Sleep", "Sleep+LoRa", "Standby+LoRa"};
        
        for (int i = 0; i < 3; i++) {
          // Write mode
          SPI1_CS_Low();
          SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
          SPI1_DirectGPIO_Transfer(modes[i]);
          SPI1_CS_High();
          vTaskDelay(pdMS_TO_TICKS(100));
          
          // Read mode
          SPI1_CS_Low();
          SPI1_DirectGPIO_Transfer(REG_OP_MODE & 0x7F);
          uint8_t mode = SPI1_DirectGPIO_Transfer(0x00);
          SPI1_CS_High();
          
          sprintf(lora_buffer, "Set %s: wrote 0x%02X, read 0x%02X %s\r\n", 
                  mode_names[i], modes[i], mode, (mode == modes[i]) ? "OK" : "FAIL");
          USART2_SendString(lora_buffer);
          vTaskDelay(pdMS_TO_TICKS(50));
        }
        break;
        
      case 3:
        // Emergency transmit test
        USART2_SendString("\r\nTEST 4: Emergency Transmit Attempt\r\n");
        
        // Format message
        memset(txData, 0, sizeof(txData));
        sprintf((char*)txData, "SX1278 EMERGENCY TEST");
        uint8_t dataSize = strlen((char*)txData);
        
        USART2_SendString("Sending emergency test message...\r\n");
        
        // Set Standby mode
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
        SPI1_DirectGPIO_Transfer(0x81);
        SPI1_CS_High();
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Clear IRQ flags
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_IRQ_FLAGS | 0x80);
        SPI1_DirectGPIO_Transfer(0xFF);
        SPI1_CS_High();
        
        // Set FIFO pointer
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_FIFO_ADDR_PTR | 0x80);
        SPI1_DirectGPIO_Transfer(0x00);
        SPI1_CS_High();
        
        // Write to FIFO
        for (int i = 0; i < dataSize; i++) {
          SPI1_CS_Low();
          SPI1_DirectGPIO_Transfer(REG_FIFO | 0x80);
          SPI1_DirectGPIO_Transfer(txData[i]);
          SPI1_CS_High();
          Delay_ms(2);
        }
        
        // Set payload length
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_PAYLOAD_LENGTH | 0x80);
        SPI1_DirectGPIO_Transfer(dataSize);
        SPI1_CS_High();
        
        // Enter TX mode
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
        SPI1_DirectGPIO_Transfer(0x83);
        SPI1_CS_High();
        
        // Wait and check
        USART2_SendString("Waiting for TX to complete...\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Read flags
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_IRQ_FLAGS & 0x7F);
        uint8_t flags = SPI1_DirectGPIO_Transfer(0x00);
        SPI1_CS_High();
        
        sprintf(lora_buffer, "TX flags: 0x%02X (TX_DONE is 0x08)\r\n", flags);
        USART2_SendString(lora_buffer);
        
        // Return to standby
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_OP_MODE | 0x80);
        SPI1_DirectGPIO_Transfer(0x81);
        SPI1_CS_High();
        break;
        
      case 4:
        // FIFO and IRQ register test
        USART2_SendString("\r\nTEST 5: FIFO and IRQ Register Test\r\n");
        
        // Read IRQ flags
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_IRQ_FLAGS & 0x7F);
        uint8_t irq = SPI1_DirectGPIO_Transfer(0x00);
        SPI1_CS_High();
        
        sprintf(lora_buffer, "IRQ Flags: 0x%02X\r\n", irq);
        USART2_SendString(lora_buffer);
        
        // Clear IRQ flags
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_IRQ_FLAGS | 0x80);
        SPI1_DirectGPIO_Transfer(0xFF);
        SPI1_CS_High();
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Read again
        SPI1_CS_Low();
        SPI1_DirectGPIO_Transfer(REG_IRQ_FLAGS & 0x7F);
        irq = SPI1_DirectGPIO_Transfer(0x00);
        SPI1_CS_High();
        
        sprintf(lora_buffer, "IRQ after clear: 0x%02X (should be 0x00)\r\n", irq);
        USART2_SendString(lora_buffer);
        break;
    }
    
    // Wait for next cycle using absolute timing
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(5000));
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
