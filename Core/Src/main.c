#include <stdbool.h>
#include "stm32g431xx.h"
#include "main.h"

const int PIN_A0 = 0; /** Represents PA0 */
const int PIN_A2 = 2; /** Represents PA2 */

const double TIMER_FREQUENCY = 16000000.0; /** The timer frequency: 16 MHz. */
const int ARR = 4999;                      /** The ARR value. Affects the calculation of the PCR value. */
const int PSC_750MS = 2399;                /** The PSC value for the timer to count 0.75 seconds in combination with the ARR = 4999. [(TIMER_FREQUENCY * 0.75) / (ARR + 1)] */
const int DEFAULT_TIME_IN_SECONDS = 5;     /** The default time in seconds for the LED if Pin A0 wasn't touched at all. */

int amountPinA0Triggered = 0; /** Contains the number of how often Pin A0 was touched. */
int pinTrigger;               /** Contains the number of the pin last triggered/last touched. */

bool isLedBlinking = false; /** True if LED is currently blinking. */
bool debounce0 = false;     /** Debounce flag for Pin A0. */
bool debounce2 = false;     /** Debounce flag for Pin A2. */

/**
 * Returns true if it's the first time touching any pin.
 * Necessary because the first time can be triggered multiple times at the same time. Reason unknown.
 */
bool isFirstTouch = true;

/**
 * Main method. Is getting called
 */
void main(void)
{
    initialize();
    while (1)
        ;
}

/**
 * Event handler for timer 3. Triggers when the timer reaches its end.
 */
void TIM3_IRQHandler()
{
    TIM3->SR &= ~TIM_SR_UIF; // Clear interrupt flag (UIF)
    toggleLed();
}

/**
 * Event handler for timer 4. Triggers when the timer reaches its end.
 */
void TIM4_IRQHandler()
{
    TIM4->SR &= ~TIM_SR_UIF; // Clear interrupt flag (UIF)
    isFirstTouch = false;

    if (pinTrigger == PIN_A0)
    {
        debounce0 = false;
        EXTI->PR1 |= EXTI_PR1_PIF0;  // Clear interrupt flag for EXTI line 0
        EXTI->IMR1 |= EXTI_IMR1_IM0; // Re-enable interrupt for EXTI line 0
    }
    else if (pinTrigger == PIN_A2)
    {
        if (isLedBlinking)
        {
            EXTI->IMR1 |= EXTI_IMR1_IM0; // Enable interrupt for EXTI line 0
        }
        isLedBlinking = !isLedBlinking;
        amountPinA0Triggered = 0;
        debounce2 = false;
        EXTI->PR1 |= EXTI_PR1_PIF2;  // Clear interrupt flag for EXTI line 2
        EXTI->IMR1 |= EXTI_IMR1_IM2; // Re-enable interrupt for EXTI line 2
    }
    pinTrigger = -1;
}

/**
 * Event handler for Pin A0 [PA0]. Triggers when someone touches the pin.
 */
void EXTI0_IRQHandler(void)
{
    if (!debounce0)
    {
        debounce0 = true;
        EXTI->IMR1 &= ~EXTI_IMR1_IM0; // Disable interrupt for EXTI line 0
        EXTI->PR1 |= EXTI_PR1_PIF0;   // Clear interrupt flag for EXTI line 0
        toggleLed();
        pinTrigger = PIN_A0;
        amountPinA0Triggered++;
        if (isFirstTouch)
        {
            amountPinA0Triggered = 1;
        }
        startTimer4();
    }
}

/**
 * Event handler for Pin A2 [PA2]. Triggers when someone touches the pin.
 */
void EXTI2_IRQHandler(void)
{
    if (!debounce2)
    {
        debounce2 = true;
        EXTI->IMR1 &= ~EXTI_IMR1_IM2; // Disable interrupt for EXTI line 2
        EXTI->PR1 |= EXTI_PR1_PIF2;   // Clear interrupt flag for EXTI line 2
        if (!isFirstTouch)
        {
            stopTimer3();
        }
        pinTrigger = PIN_A2;
        if (!isLedBlinking)
        {
            EXTI->IMR1 &= ~EXTI_IMR1_IM0; // Disable interrupt for EXTI line 0
            startTimer3();
        }
        else
        {
            toggleLed();
        }
        startTimer4();
    }
}

void initialize()
{
    initializeLed();
    initializePinInterrupts();
    initializeTimers();

    __enable_irq(); // Enable global interrupts
}

void initializeLed()
{
    // Enable clock for GPIOB
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // Set PB8 as output
    GPIOB->MODER &= ~GPIO_MODER_MODE8;
    GPIOB->MODER |= GPIO_MODER_MODE8_0;
}

void initializePinInterrupts()
{
    // Enable RCC clocks for GPIOA
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Set PA0 and PA2 as inputs
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE2);

    // Enable system configuration controller clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure external interrupt for PA0 (EXTI0) and PA2 (EXTI2)
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0 | SYSCFG_EXTICR1_EXTI2);
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA | SYSCFG_EXTICR1_EXTI2_PA;

    // Enable interrupts for EXTI0 and EXTI2
    EXTI->IMR1 |= EXTI_IMR1_IM0 | EXTI_IMR1_IM2;
    EXTI->RTSR1 |= EXTI_RTSR1_RT0 | EXTI_RTSR1_RT2;

    // Enable NVIC for EXTI0 and EXTI2
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
}

void initializeTimers()
{
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_TIM4EN; // Enable clocks for timer 3 & 4

    TIM3->ARR = ARR;
    TIM3->DIER = TIM_DIER_UIE; // Enable update interrupt for timer 3
    NVIC_EnableIRQ(TIM3_IRQn); // Enable global interrupt for timer 3

    TIM4->ARR = ARR;
    TIM4->PSC = PSC_750MS;
    TIM4->DIER = TIM_DIER_UIE; // Enable update interrupt for timer 4
    TIM4->CR1 = TIM_CR1_OPM;   // Enable one-pulse mode for timer 4
    NVIC_EnableIRQ(TIM4_IRQn); // Enable global interrupt for timer 4
}

void startTimer3()
{
    TIM3->PSC = getPscValue();
    TIM3->CR1 |= TIM_CR1_CEN; // Start the timer
}

void startTimer4()
{
    TIM4->SR &= ~TIM_SR_UIF;  // Clear interrupt flag (UIF)
    TIM4->CR1 |= TIM_CR1_CEN; // Start the timer
}

void stopTimer3()
{
    TIM3->CR1 &= ~TIM_CR1_CEN; // Stop the timer
    TIM3->CNT &= 0xFFFF0000;   // Reset timer counter
}

int getPscValue()
{
    unsigned int frequency = (int)((TIMER_FREQUENCY * DEFAULT_TIME_IN_SECONDS) / (ARR + 1));
    if (amountPinA0Triggered > 0)
    {
        frequency = frequency / (2 * amountPinA0Triggered);
    }
    return frequency - 1;
}

void toggleLed()
{
    GPIOB->ODR ^= 1 << 8;
}
