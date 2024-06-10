/* ************************************************************************** */
/** Descriptive File Name

  @Company
    HGE

  @File Name
    ONEWire.h

  @Summary
    ONEWire protocol.

  @Description
 */
/* ************************************************************************** */

#ifndef _ONEWIRE_H    /* Guard against multiple inclusion */
#define _ONEWIRE_H


#include <stdint.h>

typedef enum{
    ONEWIRE_ST_OK       = -1,
    ONEWIRE_ST_ERROR    = 0,
    ONEWIRE_ST_READY    = 1,
    ONEWIRE_ST_BUSY     = 2,
}_eONEWIREStatus;

#define ONEWIREERROR            0
#define ONEWIREREADY            1
#define ONEWIREBUSY             2
#define ONEWIREISPRESENT        3
#define ONEWIREREADBYTEREADY    4


typedef struct ONEWireConfiguration{
    void (*SETPinInput)(void);              /*<Pointer to a function that sets a pin as input*/
    void (*SETPinOutput)(void);             /*<Pointer to a function that sets a pin as output*/
    void (*WritePinBit)(uint8_t value);     /*<Pointer to a function that write a pin value*/
    uint8_t (*ReadPinBit)(void);            /*<Pointer to a function that read a pin value*/
    int (*DELAYus)(int usDelay);            /*<Pointer to a function that count 1 us*/
}_sOWConfig;

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ONEWire_Init
 * Initialize the ONEWire bus. This function must be called first.
 * @param [in] aOW: Struct that define a ONEWire Bus
 */
void ONEWire_Init(_sOWConfig *aOW);

/**
 * @brief ONEWireReset
 * Reset ONEWire bus.
 * @return
 */
uint8_t ONEWireReset(void);

/**
 * @brief ONEWireWriteBit
 * Write a bit to ONEWire bus.
 * @param [in] bitValue: bit value to write.
 * @return
 */
uint8_t ONEWireWriteBit(uint8_t bitValue);

/**
 * @brief ONEWireReadBit
 * Read a bit from ONEWire bus.
 * @param [out] bitValue: address where the bit read is saved.
 * @return
 */
uint8_t ONEWireReadBit(uint8_t *bitValue);

/**
 * @brief ONEWireReadByte
 * Read a byte from ONEWire bus.
 * @param [out] byte: address where the byte read is saved.
 * @return
 */
uint8_t ONEWireReadByte(uint8_t *byte);

/**
 * @brief ONEWireWriteByte
 * Write a byte to ONEWire bus.
 * @param [in] byte: byte to wire.
 * @return
 */
uint8_t ONEWireWriteByte(uint8_t byte);

/**
 * @brief ONEWireIsPresent
 * @return
 */
uint8_t ONEWireIsPresent();

/**
 * @brief ONEWireGetCurrentPinValue
 * @return
 */
uint8_t ONEWireGetCurrentPinValue(void);

//Functions for performing non-blocking operations on the ONEWIRE bus
/**
 * @brief ONEWireTask
 * Perfom all the task needed. This function must to be called in principal loop all the time.
 * @param [in] usCurrentTime: these are the current usec.
 */
void ONEWireTask(uint32_t usCurrentTime);

/**
 * @brief ONEWireReadByteTask
 * @return
 */
uint8_t ONEWireReadByteTask(void);

/**
 * @brief ONEWireWriteByteTask
 * @param byteValue
 * @return
 */
uint8_t ONEWireWriteByteTask(uint8_t byteValue);

/**
 * @brief ONEWireResetTask
 * @return
 */
uint8_t ONEWireResetTask(void);

/**
 * @brief ONEWireGetStatusTask
 * @return
 */
uint8_t ONEWireGetStatusTask(void);

/**
 * @brief ONEWireGetLastByteReadTask
 * @return
 */
uint8_t ONEWireGetLastByteReadTask(void);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
