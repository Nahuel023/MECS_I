/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @version        : v1.0_Cube
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief For Usb device.
  * @{
  */

/** @defgroup USBD_CDC_IF USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024
/* USER CODE BEGIN EXPORTED_DEFINES */

/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */
 /**
  * @brief Tipo de puntero a función para recibir datos.
  *
  * El tipo 'ptrFunRx' se utiliza para declarar punteros a funciones que pueden recibir
  * datos representados como un puntero a un arreglo de uint8_t (Buf) y un valor entero (Len).
  * La función apuntada debe aceptar estos parámetros para recibir y procesar los datos.
  *
  * Ejemplo de uso:
  *
  * void recibirDatos(uint8_t *datos, int longitud) {
  *     // Realizar operaciones para procesar los datos recibidos.
  * }
  *
  * ptrFunRx punteroFuncion = recibirDatos; // Asignar la dirección de la función al puntero.
  * punteroFuncion(datosRecibidos, longitudDatos); // Llamar a la función a través del puntero.
  */
typedef void (*ptrFunRx)(uint8_t *Buf, int Len);
/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototype USBD_CDC_IF_Exported_FunctionsPrototype
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */
/**
 * @brief Asocia una función de recepción de datos a la comunicación CDC.
 *
 * Esta función, 'CDC_Attach_RxFun', permite asociar una función de recepción de datos
 * representada por un puntero a función 'aPtrFunRx' a la comunicación CDC. La función
 * especificada será llamada cuando se reciban datos a través de la comunicación CDC.
 *
 * @param[in] aPtrFunRx - Un puntero a función que acepta un puntero a un arreglo de uint8_t
 *                        (Buf) y un valor entero (Len) como argumentos. Esta función se llamará
 *                        para procesar los datos recibidos.
 *
 * Ejemplo de uso:
 *
 * void miFuncionRx(uint8_t *datos, int longitud) {
 *     // Realizar operaciones para procesar los datos recibidos por CDC.
 * }
 *
 * CDC_Attach_RxFun(miFuncionRx); // Asociar 'miFuncionRx' a la comunicación CDC.
 */
void CDC_Attach_RxFun(ptrFunRx aPtrFunRx);

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */

