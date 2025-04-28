/*
 * systickTimer.h
 *
 *  Created on: Apr 28, 2025
 *      Author: phuong
 */

 #ifndef INC_SYSTICKTIMER_H_
 #define INC_SYSTICKTIMER_H_
 
 #include <stdint.h>
 
 /* Base addresses */
 #define SYSTICK_BASE          0xE000E010U
 #define RCC_BASE              0x40023800U
 #define GPIOA_BASE            0x40020000U
 
 /* RCC registers */
 #define RCC_AHB1ENR          (*(volatile uint32_t *)(RCC_BASE + 0x30U))
 
 /* GPIO registers */
 #define GPIOA_MODER          (*(volatile uint32_t *)(GPIOA_BASE + 0x00U))
 #define GPIOA_OTYPER         (*(volatile uint32_t *)(GPIOA_BASE + 0x04U))
 #define GPIOA_OSPEEDR        (*(volatile uint32_t *)(GPIOA_BASE + 0x08U))
 #define GPIOA_PUPDR          (*(volatile uint32_t *)(GPIOA_BASE + 0x0CU))
 #define GPIOA_ODR            (*(volatile uint32_t *)(GPIOA_BASE + 0x14U))
 #define GPIOA_BSRR           (*(volatile uint32_t *)(GPIOA_BASE + 0x18U))
 
 /* SysTick registers */
 #define SYSTICK_CTRL         (*(volatile uint32_t *)(SYSTICK_BASE + 0x00U))
 #define SYSTICK_LOAD         (*(volatile uint32_t *)(SYSTICK_BASE + 0x04U))
 #define SYSTICK_VAL          (*(volatile uint32_t *)(SYSTICK_BASE + 0x08U))
 #define SYSTICK_CALIB        (*(volatile uint32_t *)(SYSTICK_BASE + 0x0CU))
 
 /* Register bit definitions */
 #define SYSTICK_CTRL_ENABLE       0x00000001U
 #define SYSTICK_CTRL_TICKINT      0x00000002U
 #define SYSTICK_CTRL_CLKSOURCE    0x00000004U
 #define SYSTICK_CTRL_COUNTFLAG    0x00010000U
 
 #define RCC_AHB1ENR_GPIOAEN       0x00000001U
 
 #define GPIO_PIN_5                 (1U << 5)
 
 /* Function prototypes */
 void SysTick_CustomInit(uint32_t ticks);
 void SysTick_DelayMS(uint32_t ms);
 void LED_PA5_Init(void);
 void LED_PA5_Toggle(void);
 void LED_PA5_On(void);
 void LED_PA5_Off(void);
 void SysTick_CustomHandler(void);
 uint32_t SysTick_GetTick(void);
 /* Add this function prototype */
void SysTick_Init(uint32_t ticks);
 #endif /* INC_SYSTICKTIMER_H_ */