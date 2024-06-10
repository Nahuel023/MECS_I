/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author  		: A. Nahuel Medina
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "ESP01.h"

#include "util.h"
#include "delay_non_blocking.h"
#include "ONEWire.h"
#include "DS18B20.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	INIT_DS18B20,
	CONFIG1_DS18B20,
	CONFI2_DS18B20,
	RESPONSE_DS18B20
} _eStart;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* COMUNICACION CODE BEGIN PV */
volatile _rx rxUSB;
_tx txUSB;
uint8_t bufUSB_Rx[256];
uint8_t bufUSB_Tx[256];
uint8_t bytes_to_send;

/*Comunicacion USART (comunicacion entre el micro y la ESP01)*/
volatile _rx rxUSART;
_tx txUSART;
uint8_t bufUSART_Rx[256];
uint8_t bufUSART_Tx[256];

/*Transmision desde el micro a la pc mediante WIFI*/
_tx txAUX;
uint8_t bufAUX_Tx[256];

_sESP01Handle ESP01Manager;
//Configuracion de valores para conexion UDP
// FACULTAD

const char SSID[]= "FCAL-Personal";
const char PASSWORD[]= "fcal-uner+2019";
const char RemoteIP[]="172.22.243.121";
uint16_t RemotePORT = 30010;
uint16_t LocalPORT = 30001;

//CASA
/*
const char SSID[]= "ANM2";
const char PASSWORD[]= "anm157523";
const char RemoteIP[]="192.168.0.14";
uint16_t RemotePORT = 30010;
uint16_t LocalPORT = 30001;
*/
/* COMUNICACION CODE EDN PV */


/* MANIPULACION DE DATOS Y BANDERAS CODE BEGIN PV */
uint8_t robotMode = IDLE;
_work w;
_flag flag1;
_flag flag2;

_delay_t  waitMode;

uint8_t countERRORS = 0;
int16_t lastTemp = 0;

uint32_t lastTick;
uint32_t elapsedTicks;
uint32_t elapsedTime_us;

//DS18B20
_sOWHandle ow0;
_sDS18B20Handle ds18b20_0;

int16_t tempDS18B20;
/* MANIPULACION DE DATOS Y BANDERAS CODE END PV */

/* HEARTBEAT CODE BEGIN PV */
uint32_t SECUENCE_LED_IDLE = MASK_IDLE;
uint32_t SECUENCE_LED_FOLLOW_LINE = MASK_FOLLOW_LINE;
uint8_t indexHb = 0;
/* HEARTBEAT CODE END PV */

/*Contador para TICKER 1MS*/
uint8_t time10ms;
uint8_t time100ms;
uint8_t time1seg;
uint8_t time2seg;
uint16_t time5seg;


/* VARIABLES ESPECIFICAS SEGUN COMANDO CODE BEGIN PV */
const char FIRMWAREVERSION[] = "20231007_0102";
/* VARIABLES ESPECIFICAS SEGUN COMANDO CODE END PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* COMUNICACION CODE BEGIN PFP */
void OnUSBRx(uint8_t *buf, int length);
void DecodeHeader(_rx *RX, _tx *tx);                           	  //funcion que decodifica cabecera
void DecodeCMD(_rx *RX, _tx *tx);                                 //decodifica comando
void PutBufOnTx(_tx *TX, uint8_t *buf, uint8_t length);           //escribe datos en buffer de transmision
void PutByteOnTx(_tx *TX, uint8_t value);                         //escribe solo un valor en buffer de transmision
void PutHeaderOnTx(_tx *TX, uint8_t id, uint8_t lCmd);            //cabecera en buf de tx
void CalcAndPutCksOnTx(_tx *TX);                                  //calcula checksum
void PutStrOnTx(_tx *TX, const char *str);                        //escribe texto en buf de tx
uint8_t GetByteFromRx (_rx *RX, int8_t pre, int8_t pos);          //lee datos de buf de recepcion
void TransmitUSB();
void TransmitUSART();
/* COMUNICACION CODE END PFP */

/* SET LEDS CODE BEGIN PFP */
void setMode();
void hearbeatTask();
/* SET LEDS CODE END PFP */

/* ESP01 CODE BEGIN PFP */
void CHENState (uint8_t value);
int PutByteOnTxESP01(uint8_t value);
void GetESP01StateChange(_eESP01STATUS newState);
void GetESP01dbg(const char *dbgStr);
/* ESP01 CODE END PFP */

/* COMUNICACION CODE EDN PV */

/* DS18B20 CODE BEGIN PFP */

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void OneWireSetPinInput(void);

void OneWireSerPinOutput(void);

uint8_t OneWireReadPin(void);

void OneWireWritePin(uint8_t value);

int delay_us(int us);

/* DS18B20 CODE END PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){						//Timer 2 (Se da cada 250us = 42 * 500 / 84Mhz)
	if(htim->Instance == TIM2){
		time10ms--;
		if(!time10ms){
			IS10MS = 1;																//habilita la funcion On1ms() en el main, que maneja todos los tickers
			time10ms = 40;															// 250us * 4 = 1ms
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	ESP01_WriteRX(bufUSART_Rx[rxUSART.iw]);
	rxUSART.iw++;
	rxUSART.iw &= rxUSART.maskSize;									//Mantiene la circularidad del buffer
	HAL_UART_Receive_IT(&huart2, &rxUSART.buf[rxUSART.iw], 1);		//Lo recibido en el uart1, lo almaceno en rxUSART[], recibo de a 1 byte															//Permite que el procesador haga cosas mas importantes mientras el periferico del USART se encarga de la recepcion

}

void OnUSBRx(uint8_t *myBuf, int length){
	for(int i=0; i<length; i++){
	  	rxUSB.buf[rxUSB.iw++] = myBuf[i];			//Cargo el dato recibido en el USB en el buffer de recepcion USB
	  	rxUSB.iw &= rxUSB.maskSize;           	//Mantiene el buffer circular
	}
	DATAREADY = 1;
}

void DecodeHeader(_rx *RX, _tx *tx){												//Decodifico la cabecera de lo que llegue por USB
    uint8_t i;

    i = RX->iw;                         	//auxiliar para que no cambie valor del indice de escritura mientras recorro el buffer de recepcion

    while(RX->ir != i){
        switch(RX->header){
            case 0:
                if(RX->buf[RX->ir] == 'U'){
                    RX->header = 1;
                    RX->timeout = 5;
                }
            break;
            case 1:
                if(RX->buf[RX->ir] == 'N'){
                    RX->header = 2;
                } else {
                    RX->header = 0;
                    RX->ir--;
                }
            break;
            case 2:
                if(RX->buf[RX->ir] == 'E'){
                    RX->header = 3;
                } else {
                    RX->header = 0;
                    RX->ir--;
                }
            break;
            case 3:
                if(RX->buf[RX->ir] == 'R'){
                    RX->header = 4;
                } else {
                    RX->header = 0;
                    RX->ir--;
                }
            break;
            case 4: //length
                RX->nbytes = RX->buf[RX->ir];
                RX->header = 5;
                RX->nbytesdata = RX->buf[RX->ir]-2;
            break;
            case 5: //token
                if(RX->buf[RX->ir] == ':'){
                    RX->header = 6;
                    RX->iData = RX->ir + 1;     //indice a partir de donde se encuentran los datos del comando
                    RX->iData &= RX->maskSize;
                    RX->cks = 'U' ^ 'N' ^ 'E' ^ 'R' ^ RX->nbytes ^ ':';
                } else {
                    RX->header = 0;
                    RX->ir--;
                }
            break;
            case 6: //datos de comando
                RX->nbytes--;
                if(RX->nbytes > 0){
                    RX->cks ^= RX->buf[RX->ir];
                } else {
                    RX->header = 0;
                    if(RX->cks == RX->buf[RX->ir]) 		//llega al checksum
                        RX->ISCMD = 1;					//es un comando valido
                }
            break;
            default:
                RX->header = 0;
            break;
        }
        RX->ir &= RX->maskSize;
        RX->ir++;
        RX->ir &= RX->maskSize;
    }

}

void DecodeCMD(_rx *RX, _tx *tx){												//Decodifico el comando que llegue por USB
	RX->ISCMD = 0;

    switch(RX->buf[RX->iData]){                 //segun el comando enviado
        case 0xF0:  //ALIVE
            PutHeaderOnTx(tx, 0xF0, 2);
            PutByteOnTx(tx, 0x0D);
            CalcAndPutCksOnTx(tx);
            tx->length = 3;
            break;
        case 0xF1: //firmware
            PutHeaderOnTx(tx, 0xF1, strlen(FIRMWAREVERSION) + 1);
            PutStrOnTx(tx, FIRMWAREVERSION);
            CalcAndPutCksOnTx(tx);
            break;
        case 0xA0:
        	PutHeaderOnTx(tx, 0x12, 5);
        	w.i32 = elapsedTime_us;
        	PutByteOnTx(tx, w.u8[0]);
        	PutByteOnTx(tx, w.u8[1]);
        	PutByteOnTx(tx, w.u8[2]);
        	PutByteOnTx(tx, w.u8[3]);
        	CalcAndPutCksOnTx(tx);
        	break;
        default:
            PutHeaderOnTx(tx, RX->buf[RX->iData], 2);        //Devuelve 0xFF si hay un comando que no se interpretó
            PutByteOnTx(tx, 0xFF);
            CalcAndPutCksOnTx(tx);
        break;
    }
}

void PutBufOnTx(_tx *TX, uint8_t *buf, uint8_t length){								//Pongo buffer en Tx (Sirve para payload)
    uint8_t i;

    for(i=0; i < length; i++){
        TX->buf[TX->iw++] = buf[i];
        TX->iw &= TX->maskSize;
    }
}

void PutByteOnTx(_tx *TX, uint8_t value){											//Coloco byte en buffer de transmision
	TX->buf[TX->iw++] = value;
    TX->iw &= TX->maskSize;
}

void PutHeaderOnTx(_tx *TX, uint8_t id, uint8_t lCmd){								//Coloco cabecera hasta id en buffer de transmision
	TX->buf[TX->iw++] = 'U';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'N';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'E';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = 'R';
    TX->iw &= TX->maskSize;
    TX->length = lCmd; //largo del comando
    TX->buf[TX->iw++] = lCmd + 1; // lCmd = ID + payload || 1 byte se le suma por el checksum
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = ':';
    TX->iw &= TX->maskSize;
    TX->buf[TX->iw++] = id;
    TX->iw &= TX->maskSize;
}

void CalcAndPutCksOnTx(_tx *TX){													//Calculo checksum y coloco en buffer de transmision
    uint8_t cks, i;

    //recorro el indice y voy guardando el cks
    cks = 0;
    i = TX->length +6;
    i = TX->iw-i;
    i &= TX->maskSize;
    while(i != TX->iw){
        cks ^= TX->buf[i++];
        i &= TX->maskSize;
    }

    TX->buf[TX->iw++] = cks;
    TX->iw &= TX->maskSize;
}

void PutStrOnTx(_tx *TX, const char *str){                  						//Coloco cadena de caracteres en buffer de transmision
    uint8_t i = 0;

    while(str[i]){
        TX->buf[TX->iw++] = str[i++];
        TX->iw &= TX->maskSize;
    }

}

uint8_t GetByteFromRx (_rx *RX, int8_t pre, int8_t pos){							//Obtengo payload del buffer de recepcion
    uint8_t aux;

    RX->iData += pre;
    RX->iData &= RX->maskSize;
    aux = RX->buf[RX->iData];
    RX->iData += pos;
    RX->iData &= RX->maskSize;

    return aux;
}

void TransmitUSB(){
	uint8_t freeBytes;

	freeBytes = 0;
	if(!AUX_USB){
	   freeBytes = 255 - txUSB.ir;							//Calculo cuanto mas puede avanzar mi indice de lectura para enviar el dato
	  AUX_USB = 1;
	}
	if((txUSB.length+7 <= freeBytes) && (bytes_to_send == 0)){	//Si en mi indice de lectura me da para mandar el largo del comando lo mando directo
	  if(USBD_OK == CDC_Transmit_FS(&txUSB.buf[txUSB.ir], txUSB.length+7)){//Envio dato
		  bytes_to_send = 0;											//Cantidad de datos que faltaron enviar
		  txUSB.ir = txUSB.iw;
		  AUX_USB = 0;
	  }
	}else{
	  if((txUSB.length+7 > freeBytes) && (bytes_to_send == 0)){//Si el largo de mi comando no entra en el indice de lectura envio por partes
		  if(USBD_OK == CDC_Transmit_FS(&txUSB.buf[txUSB.ir], freeBytes)){//Envio primera parte del dato
			  txUSB.ir += freeBytes;
			  bytes_to_send = (txUSB.length+7) - freeBytes;				//Cantidad de datos que faltaron enviar
		  }
	  }
	  if(bytes_to_send > 0){
		  if(USBD_OK == CDC_Transmit_FS(&txUSB.buf[txUSB.ir], bytes_to_send)){//Envio dato bytes_to_send que falto enviar
			  bytes_to_send = 0;											//Cantidad de datos que faltaron enviar
			  txUSB.ir = txUSB.iw;
			  AUX_USB = 0;
		  }
	  }
	}
}

void TransmitUSART(){
	uint16_t len;

	len = txUSART.iw - txUSART.ir;
	len &= 0x00FF;
	if(ESP01_Send(txUSART.buf, txUSART.ir, len, 256) == ESP01_SEND_READY)
		txUSART.ir = txUSART.iw;

}

void setMode(){
	if(STARTSTOP){
		STARTSTOP = 0;
		robotMode = IDLE;
		HEARBEAT_STATUS = 0;
	}else{
		STARTSTOP = 1;
		robotMode = FOLLOW_LINE;
		HEARBEAT_STATUS = 1;
	}
}

void hearbeatTask(){
	if(HEARBEAT_STATUS){
		HAL_GPIO_WritePin(HEARBEAT_LED_GPIO_Port, HEARBEAT_LED_Pin, SECUENCE_LED_IDLE & (1<<indexHb));
		indexHb++;
		if(indexHb >= 32){
			indexHb = 0;
		}
	}else{
		HAL_GPIO_WritePin(HEARBEAT_LED_GPIO_Port, HEARBEAT_LED_Pin, SECUENCE_LED_FOLLOW_LINE & (1<<indexHb));
		indexHb++;
		if(indexHb >= 32){
			indexHb = 0;
		}
	}
}

void On10ms(){	//ticker que indica cuando paso 10ms
	time5seg--;
	if(!time5seg){
		IS5SEG = 1;
		time5seg = 500;
	}

	time1seg--;
	if(!time1seg){
		IS1S = 1;
		time1seg = 100;
	}

	time2seg--;
		if(!time2seg){
			IS2SEG = 1;
			time2seg = 200;
		}

	time100ms--;
	if(!time100ms){
		IS100MS = 1;
		time100ms = 10;
		hearbeatTask();
	}

	ESP01_Timeout10ms();


}

void On5s(){	//ticker que indica cuando paso 5s

	PutHeaderOnTx(&txUSART, 0xF0, 2);
	PutByteOnTx(&txUSART, 0x0D);
	CalcAndPutCksOnTx(&txUSART);

	PutHeaderOnTx(&txUSART, 0xA1, 3);
	w.i16[0]= tempDS18B20;
	PutByteOnTx(&txUSART, w.u8[0]);
	PutByteOnTx(&txUSART, w.u8[1]);
	CalcAndPutCksOnTx(&txUSART);

	w.i16[0]= tempDS18B20;
	PutHeaderOnTx(&txUSB, 0xA1, 3);
	PutByteOnTx(&txUSB, w.u8[0]);
	PutByteOnTx(&txUSB, w.u8[1]);
	CalcAndPutCksOnTx(&txUSB);

}

void On2s(){

	PutHeaderOnTx(&txUSB, 0x12, 5);
	w.u32 = elapsedTime_us;
	PutByteOnTx(&txUSB, w.u8[0]);
	PutByteOnTx(&txUSB, w.u8[1]);
	PutByteOnTx(&txUSB, w.u8[2]);
	PutByteOnTx(&txUSB, w.u8[3]);
	CalcAndPutCksOnTx(&txUSB);

	DS18B20_StartReadTemp(&ds18b20_0);

}

void CHENState (uint8_t value){
	HAL_GPIO_WritePin(AUX1_ESP_GPIO_Port, AUX1_ESP_Pin, value);
}

int PutByteOnTxESP01(uint8_t value)
{
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)){
		USART2->DR = value;
		return 1;
	}
	return 0;
}

void GetESP01StateChange(_eESP01STATUS newState){

	if(newState == ESP01_WIFI_CONNECTED){
		PutStrOnTx(&txUSB, "+&ESPWIFI CONNECTED\n");
	}
	if(newState == ESP01_WIFI_NEW_IP){
		PutStrOnTx(&txUSB, "+&ESPIP: ");
		PutStrOnTx(&txUSB, ESP01_GetLocalIP());
		PutStrOnTx(&txUSB, "\n");
	}
	if(newState == ESP01_UDPTCP_CONNECTED){
		PutStrOnTx(&txUSB, "+&UDP CONNECTED\n");
		//timeOutAliveUDP =10;

	}
	if(newState == ESP01_SEND_OK){
		PutStrOnTx(&txUSB, "+&UDP SEND OK\n");
	}
}

void GetESP01dbg(const char *dbgStr){
	//PutStrOnTx(&txUSB, dbgStr);
}



/*********************************** DS18B20 FUNCTIONS ****************************************/

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void OneWireSetPinInput(void){
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
}

void OneWireSerPinOutput(void){
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
}

uint8_t OneWireReadPin(void){
	return HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN);
}

void OneWireWritePin(uint8_t value){
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, value);
}

int delay_us(int us)
{
	//__HAL_TIM_SET_COUNTER(&htim3,0);  // set the counter value a 0
	//while (__HAL_TIM_GET_COUNTER(&htim3) < us);  // wait for the counter to reach the us input in the parameter

	return 1;
}

uint32_t overflowCount = 0;
uint32_t lastCount = 0;

void GetElapsedTime(void)
{
	uint32_t currentCount = TIM3->CNT;
	if (currentCount < lastCount) {
		overflowCount++;
	}
	lastCount = currentCount;
	elapsedTime_us = currentCount + overflowCount * TIM3->ARR;

}

static inline int delayBlocking(int time_us){
	uint32_t lastCount, currentCount, auSecondsAux;

	time_us *= (HAL_RCC_GetHCLKFreq()/1000000);
	lastCount = SysTick->VAL;

	while (time_us> 0){
		currentCount = SysTick->VAL;
		auSecondsAux = lastCount - currentCount;

		if(auSecondsAux > SysTick->LOAD)
			auSecondsAux += (SysTick->LOAD+1);
		time_us -= auSecondsAux;
		lastCount = currentCount;
	}

	return 1;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /*Flags*/
	flag1.byte = 0;                             				//pongo todos los bits en 0
	flag2.byte = 0;

  //CONFIGURACION DE COMUNICACION
  /*Buffers de recepcion y transmision USB y USART*/

    rxUSB.buf = (uint8_t *)bufUSB_Rx;							//Puntero a bufUSB_Rx
  	rxUSB.ir = 0;
  	rxUSB.iw = 0;
  	rxUSB.maskSize = 0xFF;
  	rxUSB.header = 0;

  	rxUSART.buf = (uint8_t *)bufUSART_Rx;						//Puntero a busUSART_Rx
  	rxUSART.ir = 0;
  	rxUSART.iw = 0;
  	rxUSART.maskSize = 0xFF;
  	rxUSART.header = 0;

  	txAUX.buf = bufAUX_Tx;										//Puntero a bufTx
  	txAUX.ir = 0;
  	txAUX.iw = 0;
  	txAUX.maskSize = 0xFF;

  	txUSB.buf = bufUSB_Tx;										//Puntero a bufUSB_Tx
  	txUSB.ir = 0;
  	txUSB.iw = 0;
  	txUSB.maskSize = 0xFF;

  	txUSART.buf = bufUSART_Tx;									//Puntero a busUSART_Tx
  	txUSART.ir = 0;
  	txUSART.iw = 0;
  	txUSART.maskSize = 0xFF;

  	/*Envio de datos USB*/
  	bytes_to_send = 0;

  	/*Contador para ticker base de 1ms*/
  	time10ms = 40;
  	time5seg = 500;

  	//Configuracion requerida ESP01.h
  	ESP01Manager.aDoCHPD = CHENState;
  	ESP01Manager.aWriteUSARTByte = PutByteOnTxESP01;
  	ESP01Manager.bufRX = bufUSART_Rx;
  	ESP01Manager.iwRX = &txUSART.iw;
  	ESP01Manager.sizeBufferRX = 256;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //DS18B20
  ow0.DELAYus = &delayBlocking;
  ow0.SETPinInput = &OneWireSetPinInput;
  ow0.SETPinOutput = &OneWireSerPinOutput;
  ow0.ReadPinBit = &OneWireReadPin;
  ow0.WritePinBit = &OneWireWritePin;

  ds18b20_0.OW = &ow0;

  DS18B20_Init(&ds18b20_0, NULL);


  //Inicializacion de DELAYNOBLOKING
  delayConfig(&waitMode, MICROSTOTICKS(500));

  //uint16_t countms = 40;
  //COMUNICACION USB
  CDC_Attach_RxFun(OnUSBRx);
  DATAREADY = 0;
  //END COMUNICACION USB

  //CONFIGURACION UDP
  ESP01_Init(&ESP01Manager);
  ESP01_SetWIFI(SSID, PASSWORD);
  ESP01_StartUDP(RemoteIP, RemotePORT, LocalPORT);
  ESP01_AttachChangeState(GetESP01StateChange);
  ESP01_AttachDebugStr(GetESP01dbg);

  HAL_UART_Receive_IT(&huart2, &rxUSART.buf[rxUSART.iw], 1);			//Se habilita la interrupcion del USART1 (Recibo de a 1 byte)
  //END CONFIGURACION UDP

  HAL_TIM_Base_Start_IT (&htim2);										//Se habilita la interrupcion del timer2
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //getElapsedTime_us();
	  GetElapsedTime();
	  ESP01_Task();
	  DS18B20_Task(&ds18b20_0, elapsedTime_us);

	  if(IS10MS){
		  IS10MS = 0;
		  On10ms();
	  }

	  if(IS5SEG){
		  IS5SEG = 0;
		  On5s();
	  }

	  if(IS2SEG){
		  IS2SEG = 0;
		  On2s();
	  }

	  if(rxUSB.ISCMD)                          					//Si se encontro un comando
		  DecodeCMD((_rx *)&rxUSB, (_tx *)&txUSB);

	  if(rxUSB.iw != rxUSB.ir)                   				//Mientras tenga datos en el buffer
		  DecodeHeader((_rx *)&rxUSB, (_tx *)&txUSB);

	  if(txUSB.iw != txUSB.ir)									//Si lo indices del tx son distintos es porque hay algo para enviar
		  TransmitUSB();

	  if(rxUSART.ISCMD)											//Si llego un comando por WIFI lo decodifico
		  DecodeCMD((_rx *)&rxUSART, (_tx *)&txUSART);

	  if(rxUSART.iw != rxUSART.ir)								//Si hay algo para decodificar en el USART
		  DecodeHeader((_rx *)&rxUSART, (_tx *)&txUSART);

	  if(txUSART.iw != txUSART.ir)								//Si hay algo para transmitir por el USART
		  TransmitUSART();

	  /*if(delayRead(&waitMode)){
		  countms--;
		  if(!countms){
			  HAL_GPIO_TogglePin(HEARBEAT_LED_GPIO_Port, HEARBEAT_LED_Pin);
			  countms = 400;
		  }
	  }*/

	  //DS18B20_Task (&ds18020_0, currentus);
	  if(DS18B20_Status(&ds18b20_0)==DS18B20_ST_TEMPOK) {
		  tempDS18B20 = DS18B20_ReadLastTemp(&ds18b20_0);

	  if(DS18B20_Status(&ds18b20_0) == DS18B20_ST_TEMPCRCERROR){

			  tempDS18B20 = DS18B20_ReadLastTemp (&ds18b20_0);
	  }

	 /* w.i8[0] = DS18B20_Status(&ds18b20_0);
	  if(w.i8[0] == DS18B20TEMPREADY || w.i8[0] == DS18B20TEMPERROR){
		  if(w.i8[0] == DS18B20TEMPERROR){
			  countERRORS++;
			  if(countERRORS >= 20){
				  countERRORS = 19;
//				  lastTemp = DS18B20_ReadLastTemp();
			  }
			  lastTemp = DS18B20_ReadLastTemp();
			PutHeaderOnTx(&txUSB, 0xF1, 16);
			PutStrOnTx(&txUSB, "TERROR         ");
			CalcAndPutCksOnTx(&txUSB);

		  }
		  if(w.i8[0] == DS18B20TEMPREADY){
			  lastTemp = DS18B20_ReadLastTemp();
			PutHeaderOnTx(&txUSB, 0xF1, 16);
			PutStrOnTx(&txUSB, "TOK            ");
			CalcAndPutCksOnTx(&txUSB);
			  countERRORS = 0;
		  }
	  }*/
  }}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 18000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HEARBEAT_LED_GPIO_Port, HEARBEAT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AUX1_ESP_GPIO_Port, AUX1_ESP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HEARBEAT_LED_Pin */
  GPIO_InitStruct.Pin = HEARBEAT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HEARBEAT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AUX1_ESP_Pin */
  GPIO_InitStruct.Pin = AUX1_ESP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AUX1_ESP_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
