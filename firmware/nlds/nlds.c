/*
 * nlds.c
 *
 * Created: 07/03/2014 09:03:58
 *  Author: Xevel
 */ 

#include <avr/io.h>

typedef enum {
    SEARCH_START = 0,
    GET_ID = 1,
    GET_RPM_L = 2,
    GET_RPM_H = 3,
    GET_DATA_L = 4,
    GET_DATA_H = 5,
    GET_CHECKSUM_L = 6,
    GET_CHECKSUM_H = 7
} state_t ;


volatile uint16_t volatile_rpm;
volatile uint8_t rpm_updated;

uint8_t nlds_rpm_updated_get(){
    return rpm_updated;
}

void nlds_rpm_updated_clear(){
    rpm_updated = 0;
}

uint16_t nlds_rpm_get(){
    return volatile_rpm;
}

state_t state;
uint8_t id;
uint32_t accumulator;
uint16_t checksum;
uint8_t nb_data;
uint16_t rpm_tmp;
uint16_t data;

void nlds_parse(uint8_t c){

    // parse to find the speed the LDS is turning at
    switch(state){
        case SEARCH_START:
            if (c == 0xFA){
                state = GET_ID;
            }
            break;
        case GET_ID:
            if ( c >= 0xA0 && c <= 0xF9 ){ // ID is supposed to be in this bracket
                id = c;
                accumulator = 0xFA + ((uint16_t)id << 8);
                state = GET_RPM_L;
                } else if ( c != 0xFA ){ // if it's not and it's not the start byte either, go back to the start
                state = SEARCH_START;
            } // else it's the start byte and we stay in the same state
            break;
        case GET_RPM_L:
            rpm_tmp = c;
            state = GET_RPM_H;
            break;
        case GET_RPM_H:
            rpm_tmp |= (uint16_t)c << 8;
            nb_data = 2;
            accumulator = (accumulator << 1) + rpm_tmp;
            state = GET_DATA_L;
            break;
        case GET_DATA_L:
            data = c;
            state = GET_DATA_H;
            break;
        case GET_DATA_H:
            data |= (uint16_t)c << 8;
            accumulator = (accumulator << 1) + data;

            nb_data++;
            if (nb_data >= 10){
                state = GET_CHECKSUM_L;
                } else {
                state = GET_DATA_L;
            }
            break;
        case GET_CHECKSUM_L:
            checksum = c;
            state = GET_CHECKSUM_H;
            break;
        case GET_CHECKSUM_H:
            checksum |= ((uint16_t)c << 8);

            accumulator = (accumulator & 0x7FFF) + (accumulator >> 15);
            accumulator = accumulator & 0x7FFF;

            if (accumulator == checksum){
                volatile_rpm = rpm_tmp;
                rpm_updated = 1;
            }
            state = SEARCH_START;
            break;
        default:
            break;
    }

}
