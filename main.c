// Ultrasonic distance sensor    07/03/2017
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include <string.h>

uint32_t timer_value = 0;
#define PushButton1    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))  // PF4== PushButton1
#define RED            (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))  // PF1== RED_HW
#define triggerPin     (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))  // PB4
uint8_t edge = 1;
uint16_t distanceCm = 0;
uint16_t distancedeci = 0;
uint16_t unidirectional = 0;
uint32_t RAWTIMER = 4294967295;
uint8_t SOUNDTIME = 29;
char strDist[15];

void GPIOEISR();
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC
            | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE
            | SYSCTL_RCGC2_GPIOF;

    // Configure LED and pushbutton pins
    GPIO_PORTB_DIR_R = 0x1F; // bits 0,1 2 3 are outputs, other pins are inputs                 a,b,c,d connected
    GPIO_PORTB_DR2R_R = 0x1F; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R = 0x1F;  // enable LEDs and pushbuttons
    GPIO_PORTB_PUR_R = 0x00;  // enable internal pull-up for push button

    GPIO_PORTC_DIR_R = 0xF0; // bits 4 5 6 7 are outputs, other pins are inputs        e f g h connected
    GPIO_PORTC_DR2R_R = 0xF0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R = 0xF0;  // enable LEDs and pushbuttons
    GPIO_PORTC_PUR_R = 0x00;  // enable internal pull-up for push button

    GPIO_PORTD_DIR_R = 0x0F;  // bits  are input    to control each 7 segments
    GPIO_PORTD_DR2R_R = 0x0F; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DEN_R = 0x0F;  // enable LEDs and pushbuttons
    GPIO_PORTD_PUR_R = 0x00;  // enable internal pull-up for push button

    GPIO_PORTE_DIR_R = 0x00;  // all bits are input.
    GPIO_PORTE_DR2R_R = 0x00; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R = 0x10;  // enable PE.4
    GPIO_PORTE_PDR_R = 0x10;  // enable internal pull-Down for Edge Trigger.
    GPIO_PORTE_IS_R = 0x00;    // confing for edge detector
    GPIO_PORTE_IBE_R = 0x10; //  to detect both edges. ie to be controlled by GPIOEV.
    GPIO_PORTE_IEV_R = 0x10;   // detect rising edge of PE.4
    GPIO_PORTE_IM_R = 0x10; // Enable interrupt to be sent to Interrupt Controller.
    GPIO_PORTE_RIS_R = 0x00;    // NO interrupt detected.
    NVIC_EN0_R |= 1 << (INT_GPIOE - 16);

    GPIO_PORTF_DIR_R = 0x02;  // PF.1 as output.
    GPIO_PORTF_DR2R_R = 0x02; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x12;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10; // enable internal pull-up for push button// Configure PORT-A UART0 Control

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uart in same status
    GPIO_PORTA_DEN_R |= 3;                        // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                      // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

}
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");
    // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");
    // 6
    __asm("             CBZ  R1, WMS_DONE1");
    // 5+1*3
    __asm("             NOP");
    // 5
    __asm("             B    WMS_LOOP1");
    // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");
    // 1
    __asm("             CBZ  R0, WMS_DONE0");
    // 1
    __asm("             B    WMS_LOOP0");
    // 1*3
    __asm("WMS_DONE0:");
    // ---
    // 40 clocks/us + error
}

void sevenseg4(uint16_t x)
{

    uint16_t temp, j, divider;
    uint8_t segNum = 8;
    // temp = x;
    divider = 1000;
    for (j = 0; j < 4; j++)
    {

        temp = x / divider;       // value divide by 1000;
        GPIO_PORTD_DATA_R = segNum;
        sevenSegment(temp);
        waitMicrosecond(3000);
        temp = x % divider;
        divider = divider / 10;
        x = temp;
        segNum = segNum >> 1;
    }
}

void sevenSegment(uint8_t l)
{
    switch (l)
    {
    case 0:
    {
        GPIO_PORTB_DATA_R = 0xF0;
        GPIO_PORTC_DATA_R = 0xCF;
    }
        break;

    case 1:
    {
        GPIO_PORTB_DATA_R = 0xF9;
        GPIO_PORTC_DATA_R = 0xFF;
    }
        break;

    case 2:
    {
        GPIO_PORTB_DATA_R = 0xF4;
        GPIO_PORTC_DATA_R = 0xAF;
    }
        break;

    case 3:
    {
        GPIO_PORTB_DATA_R = 0xF0;
        GPIO_PORTC_DATA_R = 0xBF;
    }
        break;

    case 4:
    {
        GPIO_PORTB_DATA_R = 0xF9;
        GPIO_PORTC_DATA_R = 0x9F;
    }
        break;
    case 5:
    {
        GPIO_PORTB_DATA_R = 0xF2;
        GPIO_PORTC_DATA_R = 0x9F;
    }
        break;

    case 6:
    {
        GPIO_PORTB_DATA_R = 0xF2;
        GPIO_PORTC_DATA_R = 0x8F;
    }
        break;

    case 7:
    {
        GPIO_PORTB_DATA_R = 0xF8;
        GPIO_PORTC_DATA_R = 0xFF;
    }
        break;

    case 8:
    {
        GPIO_PORTB_DATA_R = 0xF0;
        GPIO_PORTC_DATA_R = 0x80;
    }
        break;

    case 9:
    {
        GPIO_PORTB_DATA_R = 0xF8;
        GPIO_PORTC_DATA_R = 0x9F;
    }
        break;

    default:
    {
        GPIO_PORTB_DATA_R = 0xFF;
        GPIO_PORTC_DATA_R = 0xFF;
    }
        break;
    }

}

void start_timer()
{
    timer_value = 0;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_TAILR_R = 0xFFFFFFFF;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void stop_timer()
{
    timer_value = TIMER1_TAV_R;
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
}

void init_timer1()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;          // configure as 32-bit timer
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT; // configure for oneshot mode (count down)
    TIMER1_TAILR_R = 0xFFFFFFFF;         // set load value to ffff ffff max time
    TIMER1_IMR_R = ~TIMER_IMR_TATOIM;                 // turn-off interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

void configFallingEdge()
{
    GPIO_PORTE_IEV_R = 0x00;   // detect falling edge of PE.4
    edge = 0;
}

void configRisingEdge()
{
    GPIO_PORTE_IEV_R = 0x10;   // detect rising edge of PE.4
    edge = 1;
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;
    UART0_DR_R = c;
}

void putsUart0(char* str)
{
    int i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

void GPIOEISR()
{

    if (edge == 1)
    {     // rising edge detected.
        start_timer();
        edge = 0;
        configFallingEdge();
    }

    else if (edge == 0)
    {
        stop_timer();
        // do math

        unidirectional = (RAWTIMER - timer_value) / 80;
        distanceCm = (unidirectional) / SOUNDTIME;
        distancedeci = (((float)unidirectional /(float) SOUNDTIME)*100);
        distancedeci = distancedeci-(distanceCm*100);
        if (distanceCm > 450)
        {
            putsUart0("Too Far\n");
        }
        else if (distanceCm < 450)
        {
            sprintf(strDist, " %d", distanceCm);
            putsUart0(strDist);
            putcUart0('.');
            sprintf(strDist, "%d", distancedeci);
            putsUart0(strDist);
            putsUart0(" in cm\n");
        }
        edge = 1;
//        configRisingEdge();
    }
    GPIO_PORTE_ICR_R = 0x10;
}

void trigger()
{

    distanceCm = 0;
    triggerPin = 1;
    waitMicrosecond(10);
    triggerPin = 0;
}
int main(void)
{
    initHw();
    init_timer1();
    //  uint16_t i, p;
    triggerPin = 0;

    while (1)
    {

//        if (PushButton1 == 0)
        //       {
        trigger();
        //       }
        waitMicrosecond(300000);
    }

    return 0;
}
