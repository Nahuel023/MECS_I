/*
 * util.h
 *
 *  Created on: 8 nov. 2023
 *      Author: Nahuel Medina
 */

#ifndef SRC_UTIL_H_
#define SRC_UTIL_H_

#include <stdint.h>
#include <stdlib.h>
#include "string.h"
#include "math.h"
#include "main.h"

//Numero de conversiones que van a ser guardadas
#define LENGHT_MAX_CONV_ADC				32



/* USER CODE BEGIN PM */
#define STARTSTOP flag2.bit.b0			//BANDERA DE ARRANQUE

#define BUFSIZE		 		256
#define TM		 		    0.01
#define VEL_MAX				100
#define MASK_IDLE			0x0001
#define MASK_FOLLOW_LINE	0xFFFF
#define DFALSE 0
#define DTRUE  1

#define AUX_USB 			flag1.bit.b0
#define HEARBEAT_STATUS 	flag1.bit.b1
#define DATAREADY 			flag1.bit.b2
#define IS10MS 				flag1.bit.b3              //indica si pasaron 10ms
#define IS5SEG 				flag1.bit.b4              //indica si pasaron 5s
#define IS1S		 		flag1.bit.b6			  //indica si pasaron 1s
#define IS100MS		 		flag1.bit.b7			  //indica si pasaron 100ms

#define IS50MS flag2.bit.b1							  //indica si pasaron 50ms

#define MICROSTOTICKS(micros)	((micros*(SysTick->LOAD+1))/1000)


/* USER CODE END PM */


typedef struct{
    uint8_t *buf;                        //puntero a buffer
    uint8_t header;                      //cabecera
    uint16_t iw;                         //indice escritura
    uint16_t ir;                         //indice lectura
    uint16_t iData;                      //indica donde empieza datos a decodificar
    uint8_t timeout;                     //indica si hay interrupcion, resetea header
    uint8_t cks;                         //XOR de los bytes
    uint8_t nbytes;                      //cantidad de bytes a recibir
    uint8_t nbytesdata;					 //cantidad de bytes de datos
    uint16_t maskSize;                   //depende de tamaño de buffer, mantiene la circularidad
    uint8_t ISCMD;                       //bandera que indica que hay comando
}_rx;

//Estructura de manejo de datos de transmision
typedef struct{
    uint8_t *buf;                        //puntero a buffer
    uint16_t length;                     //largo para poder calcular cks
    uint16_t iw;                         //indice escritura
    uint16_t ir;                         //indice lectura
    uint16_t maskSize;                   //para mantener circularidad de buffer
}_tx;

/**
 * @brief Unión utilizada para definir una bandera de bits.
 *
 * Esta unión se utiliza para representar una bandera de bits, lo que permite almacenar
 * múltiples valores booleanos en un solo byte, sin la necesidad de variables booleanas
 * separadas. La unión consta de dos miembros:
 *
 * 1. Una estructura 'bit' que contiene campos individuales (b0 a b7) para representar
 *    cada bit de la bandera.
 * 2. Un miembro 'byte' que es un valor de 8 bits y puede ser utilizado para acceder al
 *    byte completo que representa la bandera.
 *
 * Ejemplo de uso:
 *
 * _flag flags;
 * flags.bit.b3 = 1; // Establece el cuarto bit de la bandera en 1 (verdadero).
 * flags.byte = 0xFF; // Establece todos los bits del byte en 1.
 */
typedef union{
    struct{
        uint8_t b0:1;
        uint8_t b1:1;
        uint8_t b2:1;
        uint8_t b3:1;
        uint8_t b4:1;
        uint8_t b5:1;
        uint8_t b6:1;
        uint8_t b7:1;
    }bit;
    uint8_t byte;
}_flag;

/**
 * @brief Unión que permite cambiar entre varios tipos de variables.
 *
 * Esta unión, llamada '_work', se utiliza para poder representar los mismos datos en
 * varios formatos diferentes. Los miembros de la unión incluyen:
 *
 * 1. 'u8': Un arreglo de 4 valores uint8_t que permite acceder a los datos como bytes individuales.
 * 2. 'i8': Un arreglo de 4 valores int8_t para acceder a los datos como bytes individuales con signo.
 * 3. 'u16': Un arreglo de 2 valores uint16_t para acceder a los datos como enteros de 16 bits sin signo.
 * 4. 'i16': Un arreglo de 2 valores int16_t para acceder a los datos como enteros de 16 bits con signo.
 * 5. 'u32': Un valor uint32_t que permite acceder a los datos como un entero de 32 bits sin signo.
 * 6. 'i32': Un valor int32_t que permite acceder a los datos como un entero de 32 bits con signo.
 *
 * Ejemplo de uso:
 *
 * _work w;
 * w.u8[0] = 0xFF; // Establece el primer byte con el valor 0xFF.
 * int16_t val = w.i16[1]; // Lee los 16 bits con signo desde los bytes 2 y 3.
 */
typedef union{
    uint8_t u8[4];
    int8_t i8[4];
    uint16_t u16[2];
    int16_t i16[2];
    uint32_t u32;
    int32_t i32;
    float    f;
}_work;

/**
 * @brief Enumeración que define los estados posibles para comandos en una Máquina de Estados Finitos (MEF).
 *
 * Esta enumeración, llamada '_eEstadoMEFcmd', define los estados posibles que pueden ser utilizados
 * para representar comandos en una Máquina de Estados Finitos (MEF). Cada estado tiene un valor entero
 * asociado que se utiliza para identificar el tipo de comando.
 */
typedef enum{
    ALIVE = 0xF0,
    FIRMWARE = 0xF1,
    LEDS = 0x10,
    PULSADORES = 0x12,
    SENSORESINFRAROJOS = 0xA0,
    PRUEBAMOTORES =  0xA1,
    ANGULOSERVO = 0xA2,
    DISTANCIAHCSR04 = 0xA3,
    VELOCIDADMOTORES = 0xA4,
    TIEMPOMOV90SERVO = 0xA5,
    CONFIGCOLORNEGRO = 0xA6,
    CONFIGCOLORBLANCO = 0xA7
} _eEstadoMEFcmd;

typedef enum{
    IDLE,
	FOLLOW_LINE
} _eMODE;

typedef enum{
	DATACONEXION,
	DATAACCEL,
	DATARECORDVEL
} _eOLED;

                         						  //ticker que indica cuando paso 10ms													  //ticker que indica cuando paso 100ms

#endif /* SRC_UTIL_H_ */
