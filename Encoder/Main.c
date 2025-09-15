//[Encoder]        [STM32F0Discovery]
//  │                 │
 // ├─ V+ ───────────► 3.3V
//  ├─ OutA ─────────► PA6 (TIM3_CH1)
//  ├─ OutB ─────────► PA7 (TIM3_CH2)
//  ├─ OutZ ─────────► PA0 (turns)
//  ├─ COM ──────────► GND
//  └─ SHLD ─────────► GND
// [Seria]
//PA2->RX
//PA3->Tx

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f051x8.h"
#include <math.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// Movement Variables
float W;
float Angle;
char buffer[50];

// Contador de interrupciones (opcional)
volatile int count = 0;

// Global measurements
volatile int32_t total_ticks = 0;  // Accumulated encoder ticks (handles overflows)
volatile int16_t rpm = 0;          // Calculated RPM
volatile float turns = 0;          // Total full revolutions
volatile int8_t direction = 1;     // 1=CW, 0=CCW

// Encoder configuration
#define COUNTS_PER_REV 4000        // 1000 PPR × 4 (quadrature decoding)
#define RPM_SAMPLE_TIME 100        // 100ms sampling period


//------------------Serial------------------
void USART2_Init(void) {
    // 1. Activar el reloj para GPIOA y USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // 2. Configurar PA2 (TX) y PA3 (RX) como Alternate Function
    GPIOA->MODER &= ~((3 << (2*2)) | (3 << (3*2)));   // limpiar
    GPIOA->MODER |=  (2 << (2*2)) | (2 << (3*2));     // modo AF
    GPIOA->AFR[0] |= (1 << (2*4)) | (1 << (3*4));     // AF1 = USART2

    // 3. Configurar USART2
    USART2->BRR = 8000000/9600;  // Baudrate = 9600, con fclk=48 MHz
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE; // Habilitar TX y USART
}

void USART2_SendChar(char c) {
    while (!(USART2->ISR & USART_ISR_TXE)); // Esperar a que buffer esté vacío
    USART2->TDR = c;
}

void USART2_SendString(const char *s) {
    while (*s) {
        USART2_SendChar(*s++);
    }
}

// TIM3 – Encoder overflow interrupt
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;
        direction = (TIM3->CR1 & TIM_CR1_DIR) ? 0 : 1;
        total_ticks += direction ? 0x10000 : -0x10000;
    }
}




// TIM1 – RPM calculation every 100 ms
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF)
    {
        TIM1->SR &= ~TIM_SR_UIF;

        int32_t current_total;
        uint16_t current_cnt;

        do
        {
            current_total = total_ticks;
            current_cnt = TIM3->CNT;
        } while (current_total != total_ticks);

        int32_t current_ticks = current_total + current_cnt;
        static int32_t prev_ticks = 0;
        int32_t delta = current_ticks - prev_ticks;

        // Calculate RPM
        rpm = (delta * 600) / COUNTS_PER_REV;

        // Calculate total turns
        turns = (float)current_ticks / COUNTS_PER_REV;

        prev_ticks = current_ticks;

        // Angular velocity in rad/s
       W = (int)rpm * 2 * 3.14159f / 60.0f;
        //int w_frac = (int)((W) * 100); // 2 decimales
        Angle = (current_ticks * 360.0f)/COUNTS_PER_REV;
        Angle = (int)fmodf(Angle, 360.0f);
        //int angle_frac = (int)((Angle) * 100); // 2 decimales
        int angle_int = (int)Angle;
        int angle_frac = (int)((Angle - angle_int) * 100);

        int w_int = (int)W;
        int w_frac = (int)((W - w_int) * 100);



        // Enviar valores por USART
        //snprintf(buffer, sizeof(buffer), "Angle: %.2f deg\n", W);
        //USART2_SendString(buffer);

        // Convertir W y Angle a enteros para enviar por USART
                //int angle_int = (int)Angle;
                //int angle_frac = (int)((Angle - angle_int) * 100); // 2 decimales

               // int w_int = (int)W;
                //int w_frac = (int)((W - w_int) * 100); // 2 decimales

        snprintf(buffer, sizeof(buffer), "Angle: %d.%02d deg | W: %d.%02d rad/s\n",
        		angle_int, angle_frac, w_int, w_frac);

        USART2_SendString(buffer);

    }
}

// Encoder configuration
void configure_encoder(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER |= 1<<13 | 1<<15;
    GPIOA->PUPDR |= 1<<12 | 1<<14;
    GPIOA->AFR[0] |= 1<<24 | 1<<28;

    RCC->APB1ENR |= 1<<1;
    TIM3->SMCR = 1<<0 | 1<<1;
    TIM3->CCMR1 = 1<<0 | 1<<8;
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->DIER |= 1<<0;
    NVIC_EnableIRQ(16);
    TIM3->CR1 |= 1<<0;
}

// TIM1 – periodic RPM timer
void configure_rpm_timer(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 8000 - 1;         // Prescaler for 1 kHz
    TIM1->ARR = RPM_SAMPLE_TIME - 1;
    TIM1->DIER |= 1<<0;
    TIM1->CR1 |= 1<<0;
    NVIC_EnableIRQ(13);
}

//TIM6 – optional periodic task
void TIM6_IRQHandler(void)
{
    TIM6->SR &= ~TIM_SR_UIF;
   //float dir = direction ? 1.0f : -1.0f;
}



int main(void)
{
    configure_encoder();
    configure_rpm_timer();
    USART2_Init();
    // Ahora angle_now contiene el ángulo instantáneo en grados


    // TIM6 setup (10ms interrupt)
    RCC->APB1ENR |= (1 << 4);
    TIM6->PSC = 8 - 1;
    TIM6->ARR = 999;
    TIM6->CR1 |= (1 << 0);
    TIM6->DIER |= (1 << 0);
    NVIC->ISER[0] |= (1 << 17);

    //USART2_Init();

    //char W_array[10];


    while (1) {
        //sprintf(W_array, "%.2f\n", W);  // actualizar cada ciclo
        //USART2_SendString(W_array);
        //for (volatile int i = 0; i < 500000; i++);
    }


}