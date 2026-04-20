/* ============================================================
 *
 * Pin Map:
 *   PA5  → Motor A IN1      (GPIO output)
 *   PA6  → Motor A IN2      (GPIO output)
 *   PB0  → Motor B IN3      (GPIO output)
 *   PB1  → Motor B IN4      (GPIO output)
 *   PA8  → Motor A EN       (TIM1_CH1, PWM, AF1)
 *   PA9  → Motor B EN       (TIM1_CH2, PWM, AF1)
 *   PC0  → Cooling fan      (GPIO output)
 *   PC1  → HC-SR04 TRIG     (GPIO output)
 *   PC2  → HC-SR04 ECHO     (GPIO input)
 *   PA0  → LM35 Vout        (ADC1_IN0, analog)
 *   PA2  → USART2 TX        (AF7, ST-Link virtual COM)
 * ============================================================ */

#include "stm32f4xx.h"      
                               
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define TARGET_MIN_CM       20U
#define TARGET_MAX_CM       40U
#define MOTOR_DUTY_PERCENT  75U
#define FAN_TEMP_THRESHOLD  35.0f
#define ECHO_TIMEOUT_US     30000U

static volatile uint32_t g_ms_ticks = 0;

void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static void     systick_init(void);
static void     gpio_init(void);
static void     tim1_pwm_init(void);
static void     tim2_us_init(void);
static void     adc1_init(void);
static void     usart2_init(void);

static void     delay_ms(uint32_t ms);
static void     delay_us(uint32_t us);

static void     set_motor_speed(uint8_t percent);
static void     motor_forward(void);
static void     motor_backward(void);
static void     motor_stop(void);

static uint32_t ultrasonic_read_cm(void);
static float    lm35_read_temp(void);
static void     uart_print(const char *s);


int main(void)
{
    char     buf[80];
    uint32_t dist;
    float    temp;

    systick_init();   
    gpio_init();      
    tim1_pwm_init();  
    tim2_us_init();   
    adc1_init();
    usart2_init();   

    motor_stop();
    GPIOC->BSRR = GPIO_BSRR_BR0; 

    while (1)
    {
        dist = ultrasonic_read_cm();
        temp = lm35_read_temp();

        snprintf(buf, sizeof(buf),
                 "Dist: %lu cm | Temp: %.1f C\r\n", dist, temp);
        uart_print(buf);


        if (temp >= FAN_TEMP_THRESHOLD)
            GPIOC->BSRR = GPIO_BSRR_BS0;   /*fan ON  */
        else
            GPIOC->BSRR = GPIO_BSRR_BR0;   /*fan OFF */

        if (dist == 0 || dist > 200U) {
            motor_stop();                        /* timeout    */
        } else if (dist < TARGET_MIN_CM) {
            set_motor_speed(MOTOR_DUTY_PERCENT);
            motor_backward();                    /* too close: raise */
        } else if (dist > TARGET_MAX_CM) {
            set_motor_speed(MOTOR_DUTY_PERCENT);
            motor_forward();                     /* too far: lower   */
        } else {
            motor_stop();                        /* dead zone: hold  */
        }

        delay_ms(200);
    }
}


//  1 ms tick

static void systick_init(void)
{
    SysTick->LOAD = 15999U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   
                  | SysTick_CTRL_TICKINT_Msk      
                  | SysTick_CTRL_ENABLE_Msk;      
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms_ticks;
    while ((g_ms_ticks - start) < ms);
}


//  TIM2 — µs counter
static void tim2_us_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 15U;
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->EGR = TIM_EGR_UG;         
    TIM2->CR1 = TIM_CR1_CEN;        
}

static void delay_us(uint32_t us)
{
    TIM2->CNT = 0U;
    while (TIM2->CNT < us);
}


// GPIO INIT

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN;

    /* GPIOA 
     * PA0  → ADC1_IN0  (analog)     
     * PA2  → USART2_TX (AF7)      
     * PA5  → Motor IN1 (output)
     * PA6  → Motor IN2 (output) 
     * PA8  → TIM1_CH1  (AF1)   
     * PA9  → TIM1_CH2  (AF1)   */

    GPIOA->MODER &= ~( GPIO_MODER_MODER0
                     | GPIO_MODER_MODER2
                     | GPIO_MODER_MODER5
                     | GPIO_MODER_MODER6
                     | GPIO_MODER_MODER8
                     | GPIO_MODER_MODER9 );

    GPIOA->MODER |= (3U << GPIO_MODER_MODER0_Pos)   
                  | (2U << GPIO_MODER_MODER2_Pos)   
                  | (1U << GPIO_MODER_MODER5_Pos)   
                  | (1U << GPIO_MODER_MODER6_Pos)   
                  | (2U << GPIO_MODER_MODER8_Pos)   
                  | (2U << GPIO_MODER_MODER9_Pos); 

    GPIOA->AFR[0] &= ~(0xFU << GPIO_AFRL_AFSEL2_Pos);
    GPIOA->AFR[0] |=  (7U   << GPIO_AFRL_AFSEL2_Pos);

    GPIOA->AFR[1] &= ~( (0xFU << GPIO_AFRH_AFSEL8_Pos)
                      | (0xFU << GPIO_AFRH_AFSEL9_Pos) );
    GPIOA->AFR[1] |=   (1U << GPIO_AFRH_AFSEL8_Pos)
                     | (1U << GPIO_AFRH_AFSEL9_Pos);

    /* ── GPIOB ───────────────────────────────────────────────────
     * PB0 → Motor IN3 (output)
     * PB1 → Motor IN4 (output)
     * ─────────────────────────────────────────────────────────── */
    GPIOB->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOB->MODER |=  (1U << GPIO_MODER_MODER0_Pos)
                   | (1U << GPIO_MODER_MODER1_Pos);

    /* ── GPIOC ───────────────────────────────────────────────────
     * PC0 → Fan   (output)
     * PC1 → TRIG  (output)
     * PC2 → ECHO  (input)
     * ─────────────────────────────────────────────────────────── */
    GPIOC->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOC->MODER |=  (1U << GPIO_MODER_MODER0_Pos)
                   | (1U << GPIO_MODER_MODER1_Pos);
}


// TIM1 PWM — CH1 (PA8) and CH2 (PA9) for motor EN pins
static void tim1_pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC  = 15U;
    TIM1->ARR  = 999U;
    TIM1->CR1 |= TIM_CR1_ARPE;           

    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE
                | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;

    TIM1->CCR1  = 0U;
    TIM1->CCR2  = 0U;

    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;  
    TIM1->BDTR |= TIM_BDTR_MOE;                    

    TIM1->EGR   = TIM_EGR_UG;            
    TIM1->CR1  |= TIM_CR1_CEN;
}


// ADC1 INIT
static void adc1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC123_COMMON->CCR &= ~ADC_CCR_ADCPRE;    

    ADC1->CR1  = 0U;                          
    ADC1->CR2  = 0U;                           

    ADC1->SMPR2 |= ADC_SMPR2_SMP0;

    ADC1->SQR1  = 0U;                         
    ADC1->SQR3  = 0U;                          

    ADC1->CR2  |= ADC_CR2_ADON;               
    delay_ms(2);
}


//  USART2 INIT
static void usart2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->BRR  = 139U;
    USART2->CR1  = USART_CR1_UE | USART_CR1_TE;
}


/* ============================================================
 *  MOTOR CONTROL
 *  L298N: IN1=0 IN2=1 → forward | IN1=1 IN2=0 → backward */
 
static void set_motor_speed(uint8_t percent)
{
    uint32_t ccr  = ((uint32_t)percent * 1000U) / 100U;
    TIM1->CCR1    = ccr;
    TIM1->CCR2    = ccr;
}

static void motor_forward(void)
{
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BS6;   
    GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BS1;   
}

static void motor_backward(void)
{
    GPIOA->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BR6;   
    GPIOB->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BR1;   
}

static void motor_stop(void)
{
    set_motor_speed(0U);
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;
    GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1;
}


//  ULTRASONIC READ

static uint32_t ultrasonic_read_cm(void)
{
    uint32_t timeout;

    GPIOC->BSRR = GPIO_BSRR_BR1;
    delay_us(2);
    GPIOC->BSRR = GPIO_BSRR_BS1;
    delay_us(10);
    GPIOC->BSRR = GPIO_BSRR_BR1;

    timeout = 0U;
    while (!(GPIOC->IDR & GPIO_IDR_ID2)) {
        delay_us(1);
        if (++timeout > ECHO_TIMEOUT_US) return 0U;
    }

    TIM2->CNT = 0U;
    timeout   = 0U;
    while (GPIOC->IDR & GPIO_IDR_ID2) {
        delay_us(1);
        if (++timeout > ECHO_TIMEOUT_US) return 0U;
    }

    return TIM2->CNT / 58U;   
}


// LM35 TEMPERATURE

static float lm35_read_temp(void)
{
    uint32_t timeout = 0U;

    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (!(ADC1->SR & ADC_SR_EOC)) {
        if (++timeout > 100000U) return -1.0f;
    }

    uint32_t raw = ADC1->DR;
    return (raw * 3300.0f) / (4095.0f * 10.0f);
}


// UART PRINT
 
static void uart_print(const char *s)
{
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = (uint8_t)(*s++);
    }
}
