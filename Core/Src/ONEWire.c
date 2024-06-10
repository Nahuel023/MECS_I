/** Descriptive File Name

  @Company
    HGE

  @File Name
    onewire.c

  @Summary
    Onewire protocol.

  @Description

*/

#include <stdlib.h>
#include <stdint.h>
#include "ONEWire.h"

static void (*mySetPinInputFcn)(void) = NULL;
static void (*mySetPinOutputFcn)(void) = NULL;
static void (*myWritePinBitFcn)(uint8_t value) = NULL;
static uint8_t (*myReadPinBitFcn)(void) = NULL;
static int (*myDelayusFcn)(int usDelay) = NULL;

static uint8_t oneWireTaskStatus = ONEWIREERROR;

static enum eTaskStatus{
    IDLE = 0,
    RESETSTATE0,
    RESETSTATE1,
    RESETSTATE2,        
    RESETSTATE3,        
    WRITEBITSTATE0,
    WRITEBITSTATE1,
    WRITEBITSTATE2,
    WRITEBITSTATE3,
    WRITEBITSTATE4,
    READBITSTATE0,
    READBITSTATE1,
    READBITSTATE2,
    READBITSTATE3,
    READBITSTATE4,
} taskStatus; 

static struct sTaskData{
    uint8_t     byteValue;
    uint8_t     bitIndex;
    uint32_t    usLastTime;
    uint32_t    usWaitFor;    
} taskData;

static uint8_t isPresent = 0;


void ONEWire_Init(_sOWConfig* aOW){
    mySetPinInputFcn = aOW->SETPinInput;
    mySetPinOutputFcn = aOW->SETPinOutput;
    myWritePinBitFcn = aOW->WritePinBit;
    myReadPinBitFcn = aOW->ReadPinBit;
    myDelayusFcn = aOW->DELAYus;
    
    oneWireTaskStatus = ONEWIREERROR;
    
    if(mySetPinInputFcn == NULL)
        return;
    if(mySetPinOutputFcn == NULL)
        return;
    if(myWritePinBitFcn == NULL)
        return;
    if(myDelayusFcn == NULL)
        return;
    
    oneWireTaskStatus = ONEWIREREADY;
}

uint8_t ONEWireReset(void){
    if(oneWireTaskStatus == ONEWIREERROR){
        return ONEWIREERROR;
    }
	uint8_t presence;

    mySetPinOutputFcn();

    myWritePinBitFcn(0);
    
    myDelayusFcn(480);

    mySetPinInputFcn();
    
    myDelayusFcn(80);
    
    presence = myReadPinBitFcn();
    
    myDelayusFcn(400);
    
    if(presence)
        return ONEWIREISPRESENT;
    
    return ONEWIREERROR;
}

uint8_t ONEWireWriteBit(uint8_t bitValue){
    if(oneWireTaskStatus == ONEWIREERROR)
        return ONEWIREERROR;

    mySetPinOutputFcn();

    myWritePinBitFcn(0);
    
    myDelayusFcn(6);
    
    if(bitValue)
        mySetPinInputFcn();

    myDelayusFcn(54);

    mySetPinInputFcn();

    myDelayusFcn(10);
    
    return ONEWIREREADY;
}

uint8_t ONEWireReadBit(uint8_t *bitValue){
    if(oneWireTaskStatus == ONEWIREERROR)
        return ONEWIREERROR;

    mySetPinOutputFcn();
    
    myWritePinBitFcn(0);

    myDelayusFcn(6);
    
    mySetPinInputFcn();
    
    myDelayusFcn(9);
    
    *bitValue = myReadPinBitFcn();
    
    myDelayusFcn(55);

    return ONEWIREREADY;

}

uint8_t ONEWireReadByte(uint8_t *byteValue){
    if(oneWireTaskStatus == ONEWIREERROR){
        return ONEWIREERROR;
    }
	uint8_t i = 8;
    uint8_t byte = 0;
    uint8_t bitValue;

	while (i--) {
		byte >>= 1;
        ONEWireReadBit(&bitValue);
        if(bitValue)
            byte |= 0x80;
	}

    *byteValue = byte;
    
	return ONEWIREREADY;    
}

uint8_t ONEWireWriteByte(uint8_t byte){
    if(oneWireTaskStatus == ONEWIREERROR)
        return ONEWIREERROR;
    
    uint8_t i = 8;
    
	while (i--) {
        ONEWireWriteBit(byte & 0x01);
		byte >>= 1;
	}
    
    return ONEWIREREADY;
}

uint8_t ONEWireGetCurrentPinValue(void){
    return myReadPinBitFcn();
}


//TASK functions
void ONEWireTask(uint32_t usCurrentTime){
    uint32_t aux;

    if(taskData.usWaitFor){
        aux = usCurrentTime - taskData.usLastTime;
        if(aux >= taskData.usWaitFor)
            taskData.usWaitFor = 0;
        else
            return;
    }
    else
        taskData.usLastTime = usCurrentTime;
    
    switch(taskStatus){
        case IDLE:
            oneWireTaskStatus = ONEWIREREADY;
            break;
        case RESETSTATE0:
            myWritePinBitFcn(0);
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 480;
            taskStatus = RESETSTATE1;
            break;
        case RESETSTATE1:
            mySetPinInputFcn();
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 70;
            taskStatus = RESETSTATE2; 
            break;
        case RESETSTATE2:
            isPresent = myReadPinBitFcn();
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 410;
            taskStatus = RESETSTATE3; 
            break;
        case RESETSTATE3:
            taskStatus = IDLE;
            oneWireTaskStatus = ONEWIREREADY;
            break;
        case WRITEBITSTATE0:
//            __builtin_disable_interrupts();
//            mySetPinOutputFcn();
//            _nop();
//            myWritePinBitFcn(0);
//            myDelayusFcn(6);
//            if(taskData.byteValue & taskData.bitIndex)
//                mySetPinInputFcn();
//            __builtin_enable_interrupts();
//            taskData.usLastTime = usCurrentTime;
//            taskData.usWaitFor = 54;
//            taskStatus = WRITEBITSTATE2;

            mySetPinOutputFcn();

            myWritePinBitFcn(0);
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 4;
            taskStatus = WRITEBITSTATE1;    
            break;
        case WRITEBITSTATE1:
            if(taskData.byteValue & taskData.bitIndex)
                mySetPinInputFcn();
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 54;
            taskStatus = WRITEBITSTATE2;
            break;
        case WRITEBITSTATE2:
            mySetPinInputFcn();
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 10;
            taskData.bitIndex <<= 1;
            if(taskData.bitIndex)
                taskStatus = WRITEBITSTATE0;
            else
                taskStatus = WRITEBITSTATE3;
            break;
        case WRITEBITSTATE3:
            taskStatus = IDLE;
            oneWireTaskStatus = ONEWIREREADY;
            break;
        case READBITSTATE0:
//            __builtin_disable_interrupts();
//            mySetPinOutputFcn();
//            _nop();
//            myWritePinBitFcn(0);
//            myDelayusFcn(6);
//            mySetPinInputFcn();
//            myDelayusFcn(10);
//            isPresent = myReadPinBitFcn();
//            __builtin_enable_interrupts();
//            taskData.byteValue >>= 1;
//            if(isPresent == 1)
//                taskData.byteValue |= 0x80;
//            taskData.usLastTime = usCurrentTime;
//            taskData.usWaitFor = 54;
//            taskStatus = READBITSTATE3;    

            mySetPinOutputFcn();

            myWritePinBitFcn(0);
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 4;
            taskStatus = READBITSTATE1;    
            break;
        case READBITSTATE1:
            mySetPinInputFcn();
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 6;
            taskStatus = READBITSTATE2;    
            break;
        case READBITSTATE2:     
            isPresent = myReadPinBitFcn();
            taskData.byteValue >>= 1;
            if(isPresent == 1)
                taskData.byteValue |= 0x80;
            taskData.usLastTime = usCurrentTime;
            taskData.usWaitFor = 60;
            taskStatus = READBITSTATE3;    
            break;
        case READBITSTATE3:
            taskData.bitIndex >>= 1;
            if(taskData.bitIndex)
                taskStatus = READBITSTATE0;
            else{
                taskStatus = IDLE;
                oneWireTaskStatus = ONEWIREREADY;
            }
            break;
        default:
            taskStatus = IDLE;
    }
                
}

uint8_t ONEWireReadByteTask(void){
    if(oneWireTaskStatus == ONEWIREBUSY)
        return ONEWIREBUSY;

    taskData.byteValue = 0;
    taskData.bitIndex = 0x80;
    taskStatus = READBITSTATE0;
    oneWireTaskStatus = ONEWIREBUSY;
    
    return ONEWIREREADY;
}

uint8_t ONEWireWriteByteTask(uint8_t byteValue){
    if(oneWireTaskStatus == ONEWIREBUSY)
        return ONEWIREBUSY;
    
    taskData.bitIndex = 1;
    taskData.byteValue = byteValue;
    taskStatus = WRITEBITSTATE0;
    oneWireTaskStatus = ONEWIREBUSY;
    
    return ONEWIREREADY;
}

uint8_t ONEWireResetTask(void){
    if(oneWireTaskStatus == ONEWIREBUSY)
        return ONEWIREBUSY;
    
    mySetPinOutputFcn();
    myWritePinBitFcn(1);
    taskData.usWaitFor = 3;
    taskStatus = RESETSTATE0;
    isPresent = 0;
    oneWireTaskStatus = ONEWIREBUSY;
    
    return ONEWIREREADY;
}

uint8_t ONEWireGetStatusTask(void){
    return oneWireTaskStatus;
}

uint8_t ONEWireGetLastByteReadTask(void){
    oneWireTaskStatus = ONEWIREREADY;
    return taskData.byteValue;
}


uint8_t ONEWireIsPresent(){
    if(isPresent)
        return 0;
    return 1;
}








/* *****************************************************************************
 End of File
 */
