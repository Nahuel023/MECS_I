/* ************************************************************************** */
/** Descriptive File Name

  @Company
    HGE

  @File Name
    DS18B20.c

  @Summary
    Driver DS18B20.

  @Description
*/
#include "DS18B20.h"
#include <stdint.h>

typedef enum {
    IDLE = 0,
    READINGTEMP0,
    READINGTEMP1,
    READINGTEMP2,
    READINGTEMP3,
    READINGTEMP4,
    READINGTEMP5,
    READINGTEMP6,
}_eDB18b20TaskStatus;


static int16_t lastTempValue = 0;

static uint8_t crcDS18B20 = 0, iBytes = 0;
static uint8_t dataDS18B20[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

static _eDB18b20TaskStatus db18b20TaskStatus; 

static uint8_t statusDS18B20 = DS18B20IDLE; 

static uint32_t timeOutReadTemp = 0; 
static uint32_t timeOutLastus = 0; 

static uint8_t DS18B20_CRC(uint8_t *buf, uint8_t length){
	uint8_t crc = 0, inbyte, i, mix;

	while (length--) {
		inbyte = *buf++;
		for (i = 8; i; i--) {
			mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) {
				crc ^= 0x8C;
			}
			inbyte >>= 1;
		}
	}

	return crc;
}


void DS18B20_Task(uint32_t currentus){
    
    ONEWireTask(currentus);

    if(ONEWireGetStatusTask() != ONEWIREREADY)
        return;
    
    switch(db18b20TaskStatus){
        case IDLE:
            break;
        case READINGTEMP0:
            if(ONEWireIsPresent()){
                ONEWireWriteByteTask(SKIPROM);
                db18b20TaskStatus = READINGTEMP1;
            }
            else{
                db18b20TaskStatus = IDLE;
                lastTempValue = 0xFA5A;
            }
            break;
        case READINGTEMP1:
            ONEWireWriteByteTask(CONVERTEMP);
            db18b20TaskStatus = READINGTEMP2;
            timeOutReadTemp = 1000000;
            timeOutLastus = currentus;
            break;
        case READINGTEMP2:
            timeOutLastus = currentus - timeOutLastus;
            timeOutReadTemp -= timeOutLastus;
            timeOutLastus = currentus;
            if(timeOutReadTemp > 0x8000000){
                iBytes = 0;
                db18b20TaskStatus = READINGTEMP3;
                ONEWireResetTask();
            }
            break;
        case READINGTEMP3:
            if(ONEWireIsPresent()){
                ONEWireWriteByteTask(SKIPROM);
                db18b20TaskStatus = READINGTEMP4;
            }
            else
                db18b20TaskStatus = IDLE;
            break;
        case READINGTEMP4:
            iBytes = 255;
            ONEWireWriteByteTask(READSCRACHPAD);
            db18b20TaskStatus = READINGTEMP5;
            break;
        case READINGTEMP5:
            if(iBytes != 255)
                dataDS18B20[iBytes]  = ONEWireGetLastByteReadTask();
            iBytes++;
            if(iBytes == 9){
                db18b20TaskStatus = IDLE;
                crcDS18B20 = DS18B20_CRC(dataDS18B20, 8);
                lastTempValue = 0xFA5A;
                statusDS18B20 = DS18B20TEMPERROR;
                if(dataDS18B20[8] == crcDS18B20){
                    lastTempValue = 0;
                    lastTempValue |= (dataDS18B20[1] << 8);
                    lastTempValue |= dataDS18B20[0];
                    statusDS18B20 = DS18B20TEMPREADY;
                }
            }
            else
                ONEWireReadByteTask();
            break;
        default:
            db18b20TaskStatus = IDLE;
    }
}


uint8_t DS18B20_Init(_sOWConfig *OW){
    lastTempValue = 0xFA5A;
    db18b20TaskStatus = IDLE;
    ONEWire_Init(OW);
    
    return ONEWireGetStatusTask();
}

uint8_t DS18B20_StartReadTemp(void){
    if(ONEWireGetStatusTask() != ONEWIREREADY)
        return ONEWireGetStatusTask();
    
    ONEWireResetTask();
    
    db18b20TaskStatus = READINGTEMP0;
    
    statusDS18B20 = DS18B20READINGTEMP;
    
    return DS18B20READINGTEMP;
}

int16_t DS18B20_ReadLastTemp(void){
    statusDS18B20 = DS18B20IDLE;
    return lastTempValue;
}

uint8_t DS18B20_Status(void){
    return statusDS18B20;
}


/* *****************************************************************************
 End of File
 */
