/*
 * systickTimer.c
 *
 *  Created on: Apr 28, 2025
 *      Author: phuong
 */

 #include "systickTimer.h"
 #include "FreeRTOS.h"
 #include "task.h"
 
 /* Private variables */
 static volatile uint32_t ms_counter = 0;
 
 /**
  * @brief Initialize SysTick Timer for use with FreeRTOS
  * @param ticks: Number of clock cycles between SysTick interrupts (ignored in FreeRTOS mode)
  * @note  This is a wrapper for FreeRTOS compatibility. SysTick is configured by FreeRTOS.
  */
 void SysTick_Init(uint32_t ticks)
 {
     /* In FreeRTOS, SysTick is configured by the RTOS kernel */
     /* This function is a compatibility wrapper that doesn't configure SysTick */
     /* Reset our tick counter */
     ms_counter = 0;
     
     /* When using FreeRTOS, we rely on vApplicationTickHook() to count ms */
     /* No actual hardware configuration is done here */
 }
 
 /**
  * @brief Get current tick value
  * @return Current tick count
  */
 uint32_t SysTick_GetTick(void)
 {
     return ms_counter;
 }
 
 /**
  * @brief Custom SysTick initialization - NOT FOR FREERTOS USE
  * Note: This is kept for reference only, do not use with FreeRTOS!
  * @param ticks: Number of clock cycles between SysTick interrupts
  */
 void SysTick_CustomInit(uint32_t ticks)
 {
     /* In FreeRTOS, SysTick is configured by the RTOS kernel */
     /* This function is kept for non-RTOS projects only */
     /* DO NOT call this when using FreeRTOS! */
 
     /* Disable SysTick first */
     SYSTICK_CTRL = 0;
     
     /* Set reload value */
     SYSTICK_LOAD = ticks - 1;
     
     /* Clear current value */
     SYSTICK_VAL = 0;
     
     /* Configure SysTick: processor clock, enable interrupts, enable counter */
     SYSTICK_CTRL = SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT | SYSTICK_CTRL_ENABLE;
 }
 /**
  * @brief Delay function using SysTick timer with FreeRTOS awareness
  * @param ms: number of milliseconds to delay
  */
 void SysTick_DelayMS(uint32_t ms)
 {
     /* When using FreeRTOS, we should use vTaskDelay for longer delays */
     /* This is a busy-wait implementation which should only be used for short delays */
     if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
     {
         /* In FreeRTOS context, use vTaskDelay for non-blocking delay */
         vTaskDelay(pdMS_TO_TICKS(ms));
     }
     else
     {
         /* Fallback to busy-wait if scheduler is not running */
         uint32_t start_tick = ms_counter;
         while ((ms_counter - start_tick) < ms)
         {
             /* Wait */
         }
     }
 }
 
 /**
  * @brief Initialize LED on PA5 pin (usually the onboard LED on many STM32F4 boards)
  */
 void LED_PA5_Init(void)
 {
     /* Enable GPIOA clock */
     RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
     
     /* Configure PA5 as output (Mode = 01) */
     GPIOA_MODER &= ~(3U << (5 * 2));    // Clear bits
     GPIOA_MODER |= (1U << (5 * 2));     // Set as output
     
     /* Configure as push-pull output (default) */
     GPIOA_OTYPER &= ~(1U << 5);
     
     /* Configure low speed (default) */
     GPIOA_OSPEEDR &= ~(3U << (5 * 2));
     
     /* Configure no pull-up, no pull-down (default) */
     GPIOA_PUPDR &= ~(3U << (5 * 2));
     
     /* Turn off LED initially */
     LED_PA5_Off();
 }
 
 /**
  * @brief Toggle the LED on PA5
  */
 void LED_PA5_Toggle(void)
 {
     GPIOA_ODR ^= GPIO_PIN_5;
 }
 
 /**
  * @brief Turn on the LED on PA5
  */
 void LED_PA5_On(void)
 {
     GPIOA_BSRR = GPIO_PIN_5;
 }
 
 /**
  * @brief Turn off the LED on PA5
  */
 void LED_PA5_Off(void)
 {
     GPIOA_BSRR = (GPIO_PIN_5 << 16);
 }
 
 /**
  * @brief SysTick custom handler to update our tick counter
  * Called from FreeRTOS tick callback
  */
 void SysTick_CustomHandler(void)
 {
     ms_counter++;
 }
 
 /* Add FreeRTOS tick hook to increment our counter */
 void vApplicationTickHook(void)
 {
     SysTick_CustomHandler();
 }