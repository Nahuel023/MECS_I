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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

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
uint16_t LocalPORT = 30001;*/

/* COMUNICACION CODE EDN PV */

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


_delay_t  waitMode;

/* TIMECONTROL CODE EDN PV */


/* MANIPULACION DE DATOS Y BANDERAS CODE BEGIN PV */
uint8_t robotMode = IDLE;
_work w;
_flag flag1;
_flag flag2;
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

/* TIMECONTROL CODE EDN PV */

/* DS18B20 CODE BEGIN PFP */
void delay (uint16_t time);
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

	time100ms--;
	if(!time100ms){
		IS100MS = 1;
		time100ms = 10;
		//hearbeatTask();
	}

	ESP01_Timeout10ms();


}

void On5s(){	//ticker que indica cuando paso 5s

	PutHeaderOnTx(&txUSART, 0xF0, 2);
	PutByteOnTx(&txUSART, 0x0D);
	CalcAndPutCksOnTx(&txUSART);
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

void delay (uint16_t time)
{
	//uint32_t valorActual = SysTick->VAL;
	/* change your code here for the delay in microseconds */
	//__HAL_TIM_SET_COUNTER(&htim3, 0);
	//while ((__HAL_TIM_GET_COUNTER(&htim3))<time);
}

/*********************************** DS18B20 FUNCTIONS ****************************************/
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

//float Temperature = 0;
//float Humidity = 0;
uint8_t Presence = 0;

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

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay (80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the data pin LOW
		delay (1);  // wait for > 1us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (50);  // wait for 60 us
	}
	return value;
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
  /* USER CODE BEGIN 2 */

  //Inicializacion de DELAYNOBLOKING
  delayConfig(&waitMode, MICROSTOTICKS(500));
  uint8_t countmili = 40;
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ESP01_Task();

	  if(IS10MS){
		  IS10MS = 0;
		  On10ms();
	  }

	  if(IS5SEG){
		  IS5SEG = 0;
		  On5s();
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

	  if(delayRead(&waitMode)){
		  countmili--;
		  if(!countmili){
			  HAL_GPIO_TogglePin(HEARBEAT_LED_GPIO_Port, HEARBEAT_LED_Pin);
			  countmili = 400;
		  }
	  }

	  /********************* DS18B20 *******************/
/*
	  Presence = DS18B20_Start ();
	  HAL_Delay (1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t
	  HAL_Delay (800);

	  Presence = DS18B20_Start ();
	  HAL_Delay(1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad

	  Temp_byte1 = DS18B20_Read();
	  Temp_byte2 = DS18B20_Read();
	  TEMP = (Temp_byte2<<8)|Temp_byte1;
	  //Temperature = (float)TEMP/16;

	  	PutHeaderOnTx(&txUSART, 0xA1, 3);
		w.i16[0]=20;
		PutByteOnTx(&txUSART, w.u8[0]);
		PutByteOnTx(&txUSART, w.u8[1]);
		CalcAndPutCksOnTx(&txUSART);

		w.i16[0]=10;
		PutHeaderOnTx(&txUSB, 0xA1, 3);
		PutByteOnTx(&txUSB, w.u8[0]);
		PutByteOnTx(&txUSB, w.u8[1]);
		CalcAndPutCksOnTx(&txUSB);

	  HAL_Delay(500);*/
  }
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
