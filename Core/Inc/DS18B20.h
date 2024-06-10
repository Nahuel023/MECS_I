/* ************************************************************************** */
/** Descriptive File Name

  @Company
    HGE

  @File Name
    DS18B20.h

  @Summary
    Driver for DS18B20.

  @Description
 */
/* ************************************************************************** */

#ifndef _DS18B20_H    /* Guard against multiple inclusion */
#define _DS18B20_H


#include "ONEWire.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


#define SEARCHROM           0xF0
#define READROM             0x33
#define MATCHROM            0x55            
#define SKIPROM             0xCC
#define ALARMSEARCH         0xEC
#define CONVERTEMP          0x44
#define READSCRACHPAD       0xBE
#define WRITESCRACHPAD      0x4E
#define COPYSCRACHPAD       0x48
#define RECALLEEPROM        0xB8
#define READPOWERSUPPLY     0xB4
    
#define DS18B20IDLE         0    
#define DS18B20READINGTEMP  1
#define DS18B20TEMPREADY    2
#define DS18B20TEMPERROR    3

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

/**
 * @brief DS18B20_Init
 * Initialize the ONEWire bus. This function must be called first.
 * @param [in] aOW: Struct that define a ONEWire Bus
 * @return:  
 */
uint8_t DS18B20_Init(_sOWConfig *OW);

/**
 * @brief DS18B20_StartReadTemp
 * Start a new convertion.
 * 
 * @return:  
 */
uint8_t DS18B20_StartReadTemp(void);

/**
 * @brief DS18B20_ReadLastTemp
 * Get the latest temperature measurement.
 * 
 * @return:  
 */
int16_t DS18B20_ReadLastTemp(void);


/**
 * @brief DS18B20_Status
 * Get the latest status.
 * 
 * @return:  
 */
uint8_t DS18B20_Status(void);

/**
 * @brief DS18B20_Task
 * Task that measure a new temperature.
 * 
 */
void DS18B20_Task(uint32_t currentus);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
