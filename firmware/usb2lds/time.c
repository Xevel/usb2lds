/*
 * time.c
 *
 * Created: 28/01/2014 15:28:58
 *  Author: Xevel
 */ 
#include "time.h"

void micros_init(){
    // Start the global timer with Timer0
    TCCR1B = (0 << CS12) | (1 << CS11) | (0 << CS10); // clock/8 pre-scaler => 1 timer tick every 0.5us
}

uint16_t micros(){
    uint16_t res = TCNT1;
    return res/2;
}

uint16_t micros_reset(){
    cli();
    uint16_t res = TCNT1;
    TCNT1 = 0;
    sei();
    return res/2;
}

