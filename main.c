#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define RED_LED      (1U << 1) // PF1
#define BLUE_LED     (1U << 2) // PF2
#define GREEN_LED    (1U << 3) // PF3

void UART0_Init(void) {
    SYSCTL_RCGCUART_R |= (1U << 0);   // Enable UART0
    SYSCTL_RCGCGPIO_R |= (1U << 0);    // Enable GPIO Port A

    // Configure PA0 and PA1 for UART
    GPIO_PORTA_AFSEL_R |= (1U << 0) | (1U << 1);
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) | 0x00000011;
    GPIO_PORTA_DEN_R |= (1U << 0) | (1U << 1);

    // Configure UART0 for 9600 baud rate, 8-N-1
    UART0_CTL_R &= ~UART_CTL_UARTEN; // Disable UART
    UART0_IBRD_R = 104;
    UART0_FBRD_R = 11;
    UART0_LCRH_R = UART_LCRH_WLEN_8;  // 8 bits, no parity, 1 stop bit
    UART0_CTL_R |= UART_CTL_UARTEN;   // Enable UART
}

char UART0_Read(void) {
    while ((UART0_FR_R & UART_FR_RXFE) != 0);
    return (char)(UART0_DR_R & 0xFF);
}

void UART0_Write(char data) {
    while ((UART0_FR_R & UART_FR_TXFF) != 0);
    UART0_DR_R = data;
}

void LED_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;   // Enable clock for Port F

    GPIO_PORTF_DIR_R |= RED_LED | BLUE_LED | GREEN_LED;
    GPIO_PORTF_DEN_R |= RED_LED | BLUE_LED | GREEN_LED;
}

void LED_Control(char color) {
    GPIO_PORTF_DATA_R &= ~(RED_LED | BLUE_LED | GREEN_LED); // Turn off all LEDs
    if (color == 'R'|color =='r') {
        GPIO_PORTF_DATA_R |= RED_LED; // Turn on RED LED
    } else if (color == 'B'|color =='b') {
        GPIO_PORTF_DATA_R |= BLUE_LED; // Turn on BLUE LED
    } else if (color == 'G'|color =='g') {
        GPIO_PORTF_DATA_R |= GREEN_LED; // Turn on GREEN LED
    }
}

int main(void) {
    UART0_Init();
    LED_Init();

    while (1) {
        char receivedChar = UART0_Read();

        UART0_Write(receivedChar);
        LED_Control(receivedChar); // Only control LEDs based on 'R', 'B', or 'G'
    }
}
