#include "stm32g431xx.h"
#include "main.h"

// NVIC (= nested vector interrupt controller)

void main(void)
{
  initialize();

  while (1) // endless loop, CPU-core does nothing
  {
  }
}

void TIM3_IRQHandler()
{
  TIM3->SR &= 0 << 0;   // Clear interrupt-flag (UIF)
  GPIOB->ODR ^= 1 << 8; // Toggle green LED
}
void TIM4_IRQHandler()
{
  TIM4->SR &= 0 << 0;   // Clear interrupt-flag (UIF)
  GPIOB->ODR ^= 1 << 8; // Toggle green LED
}
void TIM15_IRQHandler()
{
  TIM15->SR &= 0 << 0;   // Clear interrupt-flag (UIF)
  GPIOB->ODR ^= 1 << 8; // Toggle green LED
}

void initialize()
{
  // Port Initializiation
  RCC->CR |= 1 << 24;        // Bit24: Enable RCC clock
  RCC->AHB2ENR |= 1 << 1;    // Bit1: Enable clocks for GPIOB
  GPIOB->MODER = 0x00010000; // Bit8: Set as digital output

  // Timer 2, 3 & 4 initialization     |= logical OR
  //  1      = 00000000 00000000 00000000 00000001 = 1
  //  1 << 1 = 00000000 00000000 00000000 00000010 = 2
  //  1 << 8 = 00000000 00000000 00000001 00000000 = 256

  // Enable Interrupts
  TIM3->DIER = 1 << 0;       // TIM3 update interrupt enabled
  NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 interrupt vector in NVIC
  //__enable_irq();               // Enable *all* interrupts (global)
}



void initializeTimer3()
{
  RCC->APB1ENR1 |= 1 << 1;   // Bit1: Enable clocks
  TIM3->PSC = 999;           // Internal clockk divided by 999 -> 1000
  TIM3->ARR = 5599;          // Counts from 0 to 5599 -> 5600
  TIM3->DIER = 1 << 0;       // Update interrupt enabled
  TIM3->CR1 = 1 << 3;        // Enable one-puls mode
  NVIC_EnableIRQ(TIM3_IRQn); // Enable global interrupt
}

void initializeTimer4()
{
  RCC->APB1ENR1 |= 1 << 2;   // Bit1: Enable clocks
  TIM4->PSC = 999;           // Internal clockk divided by 999 -> 1000
  TIM4->ARR = 15999;         // Counts from 0 to 15999 -> 16000
  TIM4->DIER = 1 << 0;       // Update interrupt enabled
  TIM4->CR1 = 1 << 3;        // Enable one-puls mode
  NVIC_EnableIRQ(TIM4_IRQn); // Enable global interrupt
}

void initializeTimer15()
{
  RCC->APB2ENR |= 1 << 16;             // Bit1: Enable clocks
  TIM15->PSC = 999;                    // Internal clockk divided by 999 -> 1000
  TIM15->ARR = 47999;                  // Counts from 0 to 47999 -> 48000
  TIM15->DIER = 1 << 0;                // Update interrupt enabled
  TIM15->CR1 = 1 << 3;                 // Enable one-puls mode
  NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn); // Enable global interrupt
}

void startTimer3()
{
  TIM3->CNT &= 0xFFFF0000; // Reset timer counter
  TIM3->CR1 = 1 << 0;      // Start timer [Enable]
}

void startTimer4()
{
  TIM4->CNT &= 0xFFFF0000; // Reset timer counter
  TIM4->CR1 = 1 << 0;      // Start timer [Enable]
}

void startTimer15()
{
  TIM15->CNT &= 0xFFFF0000; // Reset timer counter
  TIM15->CR1 = 1 << 0;      // Start timer [Enable]
}