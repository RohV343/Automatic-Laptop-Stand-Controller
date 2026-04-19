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

#include "stm32f4xx.h"      /* CMSIS device header — all typedefs,
                               base addresses, bit masks included  */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ── Application constants ───────────────────────────────────── */
#define TARGET_MIN_CM       20U
#define TARGET_MAX_CM       40U
#define MOTOR_DUTY_PERCENT  75U
#define FAN_TEMP_THRESHOLD  35.0f
#define ECHO_TIMEOUT_US     30000U

/* ── SysTick ms counter ──────────────────────────────────────── */
static volatile uint32_t g_ms_ticks = 0;

void SysTick_Handler(void)
{
    g_ms_ticks++;
}

/* ── Forward declarations ────────────────────────────────────── */
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


/* ============================================================
 *  MAIN
 * ============================================================ */
int main(void)
{
    char     buf[80];
    uint32_t dist;
    float    temp;

    systick_init();   /* SysTick  → 1 ms tick (delay_ms)       */
    gpio_init();      /* all GPIO pins direction + AF           */
    tim1_pwm_init();  /* TIM1 CH1/CH2 → motor EN PWM           */
    tim2_us_init();   /* TIM2 free-running → µs delay + echo   */
    adc1_init();      /* ADC1 CH0 → LM35                        */
    usart2_init();    /* USART2 115200 → ST-Link virtual COM    */

    motor_stop();
    GPIOC->BSRR = GPIO_BSRR_BR0;   /* fan off (PC0 reset) */

    uart_print("=== Laptop Stand Ready ===\r\n");

    while (1)
    {
        dist = ultrasonic_read_cm();
        temp = lm35_read_temp();

        snprintf(buf, sizeof(buf),
                 "Dist: %lu cm | Temp: %.1f C\r\n", dist, temp);
        uart_print(buf);

        /* ── Thermal management ── */
        if (temp >= FAN_TEMP_THRESHOLD)
            GPIOC->BSRR = GPIO_BSRR_BS0;   /* PC0 set   → fan ON  */
        else
            GPIOC->BSRR = GPIO_BSRR_BR0;   /* PC0 reset → fan OFF */

        /* ── Bang-bang position controller ──────────────────────
         * Dead zone [TARGET_MIN_CM, TARGET_MAX_CM] prevents
         * the motors hunting/oscillating around a single point.
         * ─────────────────────────────────────────────────────── */
        if (dist == 0 || dist > 200U) {
            motor_stop();                        /* OOR / timeout    */
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


/* ============================================================
 *  SYSTICK — 1 ms tick
 *  HSI = 16 MHz.  LOAD = (16,000,000 / 1000) - 1 = 15999
 *  CTRL: CLKSOURCE=1 (processor clock), TICKINT=1, ENABLE=1
 * ============================================================ */
static void systick_init(void)
{
    SysTick->LOAD = 15999U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk   /* processor clock  */
                  | SysTick_CTRL_TICKINT_Msk      /* enable interrupt */
                  | SysTick_CTRL_ENABLE_Msk;      /* start counting   */
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms_ticks;
    while ((g_ms_ticks - start) < ms);
}


/* ============================================================
 *  TIM2 — free-running µs counter
 *  APB1 clock = 16 MHz (HSI, no PLL).
 *  PSC = 15  →  CK_CNT = 16MHz / (15+1) = 1 MHz  →  1 tick = 1 µs
 *  ARR = 0xFFFFFFFF (TIM2 is 32-bit on F4)
 * ============================================================ */
static void tim2_us_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 15U;
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->EGR = TIM_EGR_UG;         /* force update to load PSC/ARR */
    TIM2->CR1 = TIM_CR1_CEN;        /* start, no other flags needed */
}

static void delay_us(uint32_t us)
{
    TIM2->CNT = 0U;
    while (TIM2->CNT < us);
}


/* ============================================================
 *  GPIO INIT
 *  MODER: 00=input  01=output  10=alternate  11=analog
 *  BSRR upper 16 bits = reset, lower 16 bits = set (atomic)
 * ============================================================ */
static void gpio_init(void)
{
    /* ── Enable GPIO clocks ── */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN;

    /* ── GPIOA ───────────────────────────────────────────────────
     * PA0  → ADC1_IN0  (analog)     MODER = 11
     * PA2  → USART2_TX (AF7)        MODER = 10
     * PA5  → Motor IN1 (output)     MODER = 01
     * PA6  → Motor IN2 (output)     MODER = 01
     * PA8  → TIM1_CH1  (AF1)        MODER = 10
     * PA9  → TIM1_CH2  (AF1)        MODER = 10
     * ─────────────────────────────────────────────────────────── */
    GPIOA->MODER &= ~( GPIO_MODER_MODER0
                     | GPIO_MODER_MODER2
                     | GPIO_MODER_MODER5
                     | GPIO_MODER_MODER6
                     | GPIO_MODER_MODER8
                     | GPIO_MODER_MODER9 );

    GPIOA->MODER |= (3U << GPIO_MODER_MODER0_Pos)   /* PA0 analog      */
                  | (2U << GPIO_MODER_MODER2_Pos)   /* PA2 AF          */
                  | (1U << GPIO_MODER_MODER5_Pos)   /* PA5 output      */
                  | (1U << GPIO_MODER_MODER6_Pos)   /* PA6 output      */
                  | (2U << GPIO_MODER_MODER8_Pos)   /* PA8 AF          */
                  | (2U << GPIO_MODER_MODER9_Pos);  /* PA9 AF          */

    /* AF7 (USART2) on PA2 → AFRL bits [11:8] */
    GPIOA->AFR[0] &= ~(0xFU << GPIO_AFRL_AFSEL2_Pos);
    GPIOA->AFR[0] |=  (7U   << GPIO_AFRL_AFSEL2_Pos);

    /* AF1 (TIM1) on PA8, PA9 → AFRH bits [3:0] and [7:4] */
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
     * PC2 → ECHO  (input — default MODER=00, no change needed)
     * ─────────────────────────────────────────────────────────── */
    GPIOC->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1);
    GPIOC->MODER |=  (1U << GPIO_MODER_MODER0_Pos)
                   | (1U << GPIO_MODER_MODER1_Pos);
}


/* ============================================================
 *  TIM1 PWM — CH1 (PA8) and CH2 (PA9) for motor EN pins
 *
 *  TIM1 is on APB2. APB2 clock = 16 MHz (HSI, no PLL).
 *  PSC = 15  →  CK_CNT = 1 MHz
 *  ARR = 999 →  PWM freq = 1 MHz / 1000 = 1 kHz
 *  CCRx = (duty% × 1000) / 100
 *
 *  TIM1 is an advanced-control timer:
 *  the MOE (Main Output Enable) bit in BDTR must be set
 *  before any CH output appears on the pin.
 * ============================================================ */
static void tim1_pwm_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->PSC  = 15U;
    TIM1->ARR  = 999U;
    TIM1->CR1 |= TIM_CR1_ARPE;           /* buffered auto-reload    */

    /* CCMR1: PWM mode 1 on CH1 and CH2
     * OC1M[2:0] = 110, OC1PE = 1  (bits 6:4 and 3)
     * OC2M[2:0] = 110, OC2PE = 1  (bits 14:12 and 11)            */
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE
                | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;

    TIM1->CCR1  = 0U;
    TIM1->CCR2  = 0U;

    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;  /* enable outputs */
    TIM1->BDTR |= TIM_BDTR_MOE;                    /* main output en */

    TIM1->EGR   = TIM_EGR_UG;            /* load PSC/ARR immediately */
    TIM1->CR1  |= TIM_CR1_CEN;
}


/* ============================================================
 *  ADC1 INIT — single conversion, channel 0 (PA0), 12-bit
 * ============================================================ */
static void adc1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* ADC prescaler: PCLK2/2 = 8 MHz (CCR in ADC_Common) */
    ADC123_COMMON->CCR &= ~ADC_CCR_ADCPRE;    /* 00 = PCLK2/2      */

    ADC1->CR1  = 0U;                           /* 12-bit, no scan   */
    ADC1->CR2  = 0U;                           /* SW trigger        */

    /* Sampling time CH0: 480 cycles (SMP0[2:0] = 111 in SMPR2)   */
    ADC1->SMPR2 |= ADC_SMPR2_SMP0;

    /* Regular sequence: length=1, first conversion = CH0          */
    ADC1->SQR1  = 0U;                          /* L[3:0]=0 → 1 conv */
    ADC1->SQR3  = 0U;                          /* SQ1 = channel 0   */

    ADC1->CR2  |= ADC_CR2_ADON;               /* power on          */
    delay_ms(2);
}


/* ============================================================
 *  USART2 INIT — 115200 8N1 TX
 *  BRR = f_PCLK1 / baud = 16,000,000 / 115200 = 138.88 → 139
 * ============================================================ */
static void usart2_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    USART2->BRR  = 139U;
    USART2->CR1  = USART_CR1_UE | USART_CR1_TE;
}


/* ============================================================
 *  MOTOR CONTROL
 *  L298N: IN1=0 IN2=1 → forward | IN1=1 IN2=0 → backward
 *  Speed controlled by PWM duty on EN pin (TIM1 CCR)
 * ============================================================ */
static void set_motor_speed(uint8_t percent)
{
    uint32_t ccr  = ((uint32_t)percent * 1000U) / 100U;
    TIM1->CCR1    = ccr;
    TIM1->CCR2    = ccr;
}

static void motor_forward(void)
{
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BS6;   /* IN1=0, IN2=1 */
    GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BS1;   /* IN3=0, IN4=1 */
}

static void motor_backward(void)
{
    GPIOA->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BR6;   /* IN1=1, IN2=0 */
    GPIOB->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BR1;   /* IN3=1, IN4=0 */
}

static void motor_stop(void)
{
    set_motor_speed(0U);
    GPIOA->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR6;
    GPIOB->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1;
}


/* ============================================================
 *  HC-SR04 ULTRASONIC
 *  Send 10 µs TRIG pulse → measure ECHO pulse width → /58 = cm
 * ============================================================ */
static uint32_t ultrasonic_read_cm(void)
{
    uint32_t timeout;

    /* 10 µs trigger pulse on PC1 */
    GPIOC->BSRR = GPIO_BSRR_BR1;
    delay_us(2);
    GPIOC->BSRR = GPIO_BSRR_BS1;
    delay_us(10);
    GPIOC->BSRR = GPIO_BSRR_BR1;

    /* Wait for ECHO (PC2) to go high */
    timeout = 0U;
    while (!(GPIOC->IDR & GPIO_IDR_ID2)) {
        delay_us(1);
        if (++timeout > ECHO_TIMEOUT_US) return 0U;
    }

    /* Measure how long ECHO stays high */
    TIM2->CNT = 0U;
    timeout   = 0U;
    while (GPIOC->IDR & GPIO_IDR_ID2) {
        delay_us(1);
        if (++timeout > ECHO_TIMEOUT_US) return 0U;
    }

    return TIM2->CNT / 58U;   /* TIM2 at 1 MHz → CNT value = µs */
}


/* ============================================================
 *  LM35 TEMPERATURE
 *  Vref = 3.3V, 12-bit ADC (0–4095)
 *  LM35 = 10 mV/°C
 *  Temp = (raw × 3300) / (4095 × 10)
 * ============================================================ */
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


/* ============================================================
 *  UART PRINT — polling TX, no interrupts needed for debug
 * ============================================================ */
static void uart_print(const char *s)
{
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = (uint8_t)(*s++);
    }
}
