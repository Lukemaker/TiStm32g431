#include "stm32g431xx.h"
#include "main.h"

void main(void)
{
  initialize();

  while (1) // endless loop, CPU-core does nothing
  {
  }
}

/**
 * Eventhandler for timer 3. Triggers when the timer reaches its end.
 */
void TIM3_IRQHandler()
{
  TIM3->SR &= ~TIM_SR_UIF; // Clear interrupt-flag (UIF)
  GPIOB->ODR ^= 1 << 8;    // Toggle green LED
}

/**
 * Eventhandler for timer 4. Triggers when the timer reaches its end.
 */
void TIM4_IRQHandler()
{
  TIM4->SR &= ~TIM_SR_UIF; // Clear interrupt-flag (UIF)
  GPIOB->ODR ^= 1 << 8;    // Toggle green LED
}

/**
 * Eventhandler for timer 15. Triggers when the timer reaches its end.
 */
void TIM15_IRQHandler()
{
  TIM15->SR &= ~TIM_SR_UIF; // Clear interrupt-flag (UIF)
  GPIOB->ODR ^= 1 << 8;     // Toggle green LED
}

/**
 * Eventhandler for Pin A0 [PA0]. Triggers when someone touches the pin.
 */
void EXTI0_IRQHandler(void)
{
  EXTI->IMR1 &= ~EXTI_IMR1_IM0; // Deactivate interrupt
  EXTI->PR1 |= EXTI_PR1_PIF0;   // Clear interrupt flag for EXTI line 0
  GPIOB->ODR ^= 1 << 8;         // Toggle green LED
  EXTI->IMR1 &= ~EXTI_IMR1_IM0; // Activate interrupt
}

/**
 * Eventhandler for Pin A2 [PA3]. Triggers when someone touches the pin.
 */
void EXTI3_IRQHandler(void)
{
  EXTI->IMR1 &= ~EXTI_IMR1_IM3; // Deactivate interrupt
  EXTI->PR1 |= EXTI_PR1_PIF3;   // Clear interrupt flag for EXTI line 3
  GPIOB->ODR ^= 1 << 8;         // Toggle green LED
  EXTI->IMR1 &= ~EXTI_IMR1_IM3; // Activate interrupt
}

void initialize()
{
  // Enable RCC clocks for GPIOA and GPIOB
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  // Set PA0 and PA3 as inputs
  GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE3);

  // RCC->CR |= 1 << 24;     // Bit24: Enable RCC clock
  //  Enable system configuration controller clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

  // Configure external interrupt for PA0 (EXTI0) and PA3 (EXTI3)
  SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0 | SYSCFG_EXTICR1_EXTI3);
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA | SYSCFG_EXTICR1_EXTI3_PA;

  // Enable interrupts for EXTI0 and EXTI3
  EXTI->IMR1 |= EXTI_IMR1_IM0 | EXTI_IMR1_IM3;
  EXTI->RTSR1 |= EXTI_RTSR1_RT0 | EXTI_RTSR1_RT3;

  // Enable NVIC for EXTI0 and EXTI3
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_EnableIRQ(EXTI3_IRQn);

  // Enable clock for GPIOB (for LED)
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

  // Set PB8 as output
  GPIOB->MODER &= ~GPIO_MODER_MODE8;
  GPIOB->MODER |= GPIO_MODER_MODE8_0;

  // Initialize timers
  initializeTimer3();
  initializeTimer4();
  initializeTimer15();

  // Enable global interrupts
  __enable_irq();
}

void initializeTimer3()
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; // Bit1: Enable clocks
  TIM3->PSC = 999;                      // Internal clock divided by 999 -> 1000
  TIM3->ARR = 5599;                     // Counts from 0 to 5599 -> 5600
  TIM3->DIER = TIM_DIER_UIE;            // Update interrupt enabled
  TIM3->CR1 = TIM_CR1_OPM;              // Enable one-pulse mode
  NVIC_EnableIRQ(TIM3_IRQn);            // Enable global interrupt
}

void initializeTimer4()
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN; // Bit2: Enable clocks
  TIM4->PSC = 999;                      // Internal clock divided by 999 -> 1000
  TIM4->ARR = 15999;                    // Counts from 0 to 15999 -> 16000
  TIM4->DIER = TIM_DIER_UIE;            // Update interrupt enabled
  TIM4->CR1 = TIM_CR1_OPM;              // Enable one-pulse mode
  NVIC_EnableIRQ(TIM4_IRQn);            // Enable global interrupt
}

void initializeTimer15()
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // Bit16: Enable clocks
  TIM15->PSC = 999;                    // Internal clock divided by 999 -> 1000
  TIM15->ARR = 47999;                  // Counts from 0 to 47999 -> 48000
  TIM15->DIER = TIM_DIER_UIE;          // Update interrupt enabled
  TIM15->CR1 = TIM_CR1_OPM;            // Enable one-pulse mode
  NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn); // Enable global interrupt
}

void startTimer3()
{
  TIM3->CNT &= 0xFFFF0000; // Reset timer counter
  TIM3->CR1 = TIM_CR1_CEN; // Start timer [Enable]
}

void startTimer4()
{
  TIM4->CNT &= 0xFFFF0000; // Reset timer counter
  TIM4->CR1 = TIM_CR1_CEN; // Start timer [Enable]
}

void startTimer15()
{
  TIM15->CNT &= 0xFFFF0000; // Reset timer counter
  TIM15->CR1 = TIM_CR1_CEN; // Start timer [Enable]
}
