/*
 * delay_non_blocking.h
 *
 *  Created on: Jun 6, 2024
 *      Author: Alexi
 */

#ifndef INC_DELAY_NON_BLOCKING_H_
#define INC_DELAY_NON_BLOCKING_H_

#include "main.h"

#define DFALSE 0
#define DTRUE  1
#define MICROSTOTICKS(micros)	((micros*(SysTick->LOAD+1))/1000)

/* TIMECONTROL CODE BEGIN PV */

/**
 * @brief Estructura para manejar el tiempo
 *
 */
typedef struct{
    int32_t    startTime;   //Almacena el tiempo leido del timer
    uint16_t    interval;   //intervalo de comparación para saber si ya transcurrio el tiempo leido
    uint8_t     isRunnig;   //indica si el delay está activo o no
}_delay_t;


/* TIMECONTROL CODE EDN PV */

/* TIMECONTROL CODE BEGIN PV */

/**
 * @brief Configura el entervalo del Delay
 *
 * @param delay     Estructura para manejar el tiempo
 * @param interval  intervalo de comparación para saber si ya transcurrio el tiempo leido
 */
void delayConfig(_delay_t *delay, uint32_t  interval);

/**
 * @brief Lee el Delay para determinar si se ha cumplido el tiempo establecido
 *
 * @param delay     Estructura para manejar el tiempo
 * @return uint8_t  Returna True o False de acuerdo a si se cumplio el tiempo o no
 */
uint8_t delayRead(_delay_t *delay);

/**
 * @brief PErmite modificar el intervalo establecido para un Delay y lo resetea
 *
 * @param delay     Estructura para manejar el tiempo
 * @param interval  intervalo de comparación para saber si ya transcurrio el tiempo leido
 */
void delayWrite(_delay_t *delay, uint16_t interval);

extern uint32_t lastTick;
extern uint32_t elapsedTicks;
extern uint32_t elapsedTime_us;

static inline void getElapsedTime_us(void){

    if (SysTick->VAL > lastTick) {
        // El contador se ha desbordado
        elapsedTicks = (SysTick->LOAD+1 - SysTick->VAL + lastTick);
    } else {
        elapsedTicks = (lastTick - SysTick->VAL);
    }

    // Convertir ticks a microsegundos
    elapsedTime_us += ((elapsedTicks * 1000000) / SystemCoreClock);

    lastTick = SysTick->VAL;  // Actualizar el último tick registrado

}

/* TIMECONTROL CODE EDN PV */

#endif /* INC_DELAY_NON_BLOCKING_H_ */
