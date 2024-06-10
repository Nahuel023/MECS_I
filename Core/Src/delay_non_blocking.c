/*
 * delay_non_blocking.c
 *
 *  Created on: Jun 6, 2024
 *      Author: Alexis Nahuel Medina
 */

#include "delay_non_blocking.h"



/* TIMECONTROL CODE BEGIN PV */

void delayConfig(_delay_t *delay, uint32_t  interval){
    delay->interval =interval;
    delay->isRunnig = DFALSE;
}

uint8_t delayRead(_delay_t *delay){
    uint8_t timeReach = DFALSE;
    uint32_t elapsedTime;

    if(!delay->isRunnig){
        delay->isRunnig = DTRUE;
        delay->startTime = SysTick->VAL;
    }else{

    	if (SysTick->VAL > delay->startTime) {
    	    // El contador se ha desbordado
    	    elapsedTime = (((SysTick->LOAD +1) - SysTick->VAL) + delay->startTime);
    	} else {
    	    elapsedTime = delay->startTime - SysTick->VAL;
    	}

        if(elapsedTime>=delay->interval){
            timeReach = DTRUE;
            delay->isRunnig = DFALSE;
        }
    }
    return timeReach;
}

void delayWrite(_delay_t *delay, uint16_t interval){
    delay->interval = interval;
    delay->isRunnig = DFALSE;
}




/* TIMECONTROL CODE EDN PV */
