/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds18b20.h"
#include "usbd_cdc_if.h"
#include "math.h"
//#include "stdbool.h"
#include "bmp180.h"
#include "radsens.h"
#include "TCS34725.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_STRING_FORMAT "#%d#%.4f#\n" // формат строки отправляемой в ком-порт 
#define USB_STRING_FORMAT_HX "#%d#%d#%.4f#\n" // формат строки отправляемой в ком-порт
#define HX711_GAIN_DEFAULT 128
#define ADDR 0x4A<<1  // gy-49 addres light sensor 

#define AHT10_Adress 0x38 << 1 // сдвигаем адрес влево, т.к последний бит адреса отвечает за режим работы запись или чтение
// если какое-то из i2c устройств не заработает, попробуй убрать или добавить сдвиг адреса 

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t count_tic = 0;

#define    DWT_CYCCNT    *(volatile uint32_t*)0xE0001004
#define    DWT_CONTROL   *(volatile uint32_t*)0xE0001000
#define    SCB_DEMCR     *(volatile uint32_t*)0xE000EDFC
void dwt_init()
{
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk;
  DWT_CYCCNT = 0;
}	
enum modulesId
{
	ID_MODULE_ATM_PRESSURE=1,
	ID_MODULE_HUMIDITY,
	ID_MODULE_TIME,
	ID_MODULE_DIF_PRESSURE,
	ID_MODULE_INDUCTANCE,
	ID_MODULE_RADIATION,
	ID_MODULE_WEIGHT,
	ID_MODULE_LIGHT,
	ID_MODULE_FORCE,
	ID_MODULE_TEMP,
	ID_MODULE_RESISTANCE,
	ID_MODULE_CAPACITY,
	ID_MODULE_CURRENT_2MA,
	ID_MODULE_CURRENT_200MA,
	ID_MODULE_CURRENT_10A,
	ID_MODULE_CURRENT_1A_AC,
	ID_MODULE_VOLTAGE_200MV,
	ID_MODULE_VOLTAGE_30V,
	ID_MODULE_VOLTAGE_30V_AC,
	ID_MODULE_SPIROMETER,
	ID_MODULE_OXYGEN,
	ID_MODULE_NITRATES,
	ID_MODULE_CO_GAS,
	ID_MODULE_TEMP_FAST,
	ID_MODULE_ULTRAV,
	ID_MODULE_RED_CHANNEL,
	ID_MODULE_GREEN_CHANNEL,
	ID_MODULE_BLUE_CHANNEL
};

enum rj45Modules
{
	RJ45_1_N=1,
	RJ45_5_N,
	RJ45_50_N,
	RJ45_WEIGHT,
	RJ45_PRESSURE
};

enum voltageSensors
{
	DC_CURRENT,
	DC_VOLTAGE,
	AC_CURRENT,
	AC_VOLTAGE
};
_Bool  signsAllowed=0;/* включает точку и минус: например - омметр - светодиоды Ом и кОм 
подключены к пинам С14 и С15, к тем же подключены минус и точка на табло. В омметре точка и минус не нужны, 
поэтому просто отключаем их. Если используется датчик температуры, силы, или напряжения, то включаем светодиоды на
"точке" и "минусе" подав лог 1 с пина С13 на базу транзистора.
*/

uint8_t  currentModule=0; //номер подключенного модуля

uint8_t  rj45_connected[3]={0}; // type of sensor connected via RJ-45 (determined by adc value)
float    hx711Value[3]={0}; // pressure, weight and force sensors values 
uint8_t  valueOnScreen=0;// какое из значений показывается на 7сег диспл.
float    offset[3]={0}; // усановка 0 или тары для весов 


uint16_t  adc[3]={0}; // 3 adc1 channels 
uint32_t  adcTotal[3]={0}; 
uint16_t  adcNSamples=0;

uint32_t volatile timeStart=0; // переменные которые изменяются в прерывании волатильные 
uint32_t volatile timeInterv [10]={0};
_Bool   volatile    time_started=0;
uint8_t volatile  interruptOnSwitch=0;
uint8_t volatile  interruptsCount=0;
uint8_t           intervalOnScreen=0;
uint32_t volatile period=0;
uint32_t volatile periodLast=0;
//
uint32_t volatile period_t=0;
uint32_t volatile periodLast_t=0;
uint32_t volatile old_count = 0;
_Bool volatile flag_time_task = 0;
uint32_t lcd_time_task_count=0;
_Bool volatile data_1_f = 0;
_Bool volatile data_2_f = 0;
uint32_t volatile pin_in = 0;
uint8_t zero = 0xD7;
//
_Bool volatile irqFlag=0;
_Bool volatile  debounceLong = 1;

char bufUsb[20]; // buffer to send to USB
char bufUsbStatus[20]; // buffer to send to USB
float result =0;
float in_pres = 0;
uint8_t diap=0;
float freq=0;
uint32_t cap_count = 0;
uint32_t cap_count_old = 0;
float f = 0;
float f_1 = 0;
float f_all = 0;
uint32_t cap_count_1 = 0;
float cap_old=0;
	uint32_t sendStatusTime=0; 
	uint32_t sendTime=0; 
/////////////////////////////////
	uint8_t shiftBuf[3]={0};	
	char numberStr[10]={0};
	
	_Bool USBconnected=0;
	
//////////////////////
_Bool startSpirograph = 0;
float spirographSum = 0;
uint8_t spirographCounter = 0;
float spirographCounterSec = 0;
float sendPress = 0;
//////////////////////
	
float Rx = 0;
uint16_t dac = 1;
int flag = 0;
uint8_t    input=1;
float callibr = 0.0, callibr300 = 1265000.0;	
_Bool firstConnectPress = 1;
float offsetPress = 0;	
	_Bool ff = 0;
	float aht_20_t = 0;
/* USER CODE END PV */
float induct_result = 0;
///
///
uint16_t c_t = 0;
uint16_t r_t = 0;
uint16_t g_t = 0;
uint16_t b_t = 0;

uint16_t r_p = 0;
uint16_t g_p = 0;
uint16_t b_p = 0;
///	
///	
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void delayUs (uint32_t micros)
{
	micros*=(SystemCoreClock / 1000000) / 9;
	while (micros--);
}

// returns the module ID (1-24)
uint8_t getModuleId(void)  
{
	uint8_t modId=0;
	//к этим пинам идут перемычки на платах, их нужно запаять как единицы в  номере модуля в двоичной системе исч
	if (!HAL_GPIO_ReadPin(id1_GPIO_Port,id1_Pin))  {modId |= 0x1;}
	if (!HAL_GPIO_ReadPin(id2_GPIO_Port,id2_Pin))  {modId |= 0x2;}
	if (!HAL_GPIO_ReadPin(id3_GPIO_Port,id3_Pin))  {modId |= 0x4;}
	if (!HAL_GPIO_ReadPin(id4_GPIO_Port,id4_Pin))  {modId |= 0x8;}
	if (!HAL_GPIO_ReadPin(id5_GPIO_Port,id5_Pin))  {modId |= 0x10;}
	if (modId == 4 && (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) || !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) || !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)))
		return 20;
	return modId; 
}

// программный spi т.к пришлось переносить проект с С6 камня
void sendByteSPI (uint8_t byte)	
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	for(uint8_t i =0; i<8; i++)
	{
		if (byte & 0x80) 
			{ 
				HAL_GPIO_WritePin(mosi_soft_GPIO_Port, mosi_soft_Pin, GPIO_PIN_SET);
			}
		else
		{
			HAL_GPIO_WritePin(mosi_soft_GPIO_Port, mosi_soft_Pin, GPIO_PIN_RESET);
		}
		delayUs(10);
		HAL_GPIO_WritePin(clk_soft_GPIO_Port, clk_soft_Pin, GPIO_PIN_SET);	
		byte<<=1;
		delayUs(10);
		HAL_GPIO_WritePin(clk_soft_GPIO_Port, clk_soft_Pin, GPIO_PIN_RESET);
	}
	
}

// display first 3 digits of float value  on 7-segment  
void displayFloat (float value)
{	
//	uint8_t shiftBuf[3]={0};	
	//char numberStr[10]={0};
	uint8_t pos=0; // position of digit (0-2)
	
	if (signsAllowed)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_RESET); // -
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_RESET);	// .
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET);	// to transistor base
	if (value<0) // включаем минус перед индикаторами
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET);
			value*=-1;
		}
	if (value<1) // включаем точку
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET);
			value*=1000;
		}
	}
	snprintf(numberStr, 4+1, "%f",value );
	
	for (uint8_t i=0; i<4; i++)
	{
		switch (numberStr[i])
		{
			case '.': if(pos!=0){pos--; shiftBuf[pos] |= 0x20;}// add point to previous symbol
						 break; 
			case '0':
				shiftBuf[pos] = zero;
				if (value > 0)
					shiftBuf[pos] = pos == 1 ? 0xD7 : zero;
				break;
			case '1': shiftBuf[pos] = 0x81; break;
			case '2': shiftBuf[pos] = 0xCE; break;
			case '3': shiftBuf[pos] = 0xCB; break;
			case '4': shiftBuf[pos] = 0x99; break;
			case '5': shiftBuf[pos] = 0x5B; break;
			case '6': shiftBuf[pos] = 0x5F; break;
			case '7': shiftBuf[pos] = 0xC1; break;
			case '8': shiftBuf[pos] = 0xDF; break;
			case '9': shiftBuf[pos] = 0xDB; break;
			default: shiftBuf[pos]=0;break;
		}
		pos++;
		if (pos>2) break; // when 3 indicators is filled 
	}
	
		//	HAL_SPI_Transmit(&hspi1, shiftBuf, 3, 1000);  // send to shift reg
	    for (uint8_t i =0; i<3; i++) { sendByteSPI(shiftBuf[i]);}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // on latch pin 
}
void displayFloat_g (float value)
{	
//	uint8_t shiftBuf[3]={0};	
	//char numberStr[10]={0};
	uint8_t pos=0; // position of digit (0-2)
	
	if (signsAllowed)
	{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_RESET); // -
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_RESET);	// .
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_RESET);	// to transistor base
	if (value<0) // включаем минус перед индикаторами
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET);
			value*=-1;
		}
	if (value<1) // включаем точку
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET);
			value*=1000;
		}
	}
	snprintf(numberStr, 4+1, "%f",value );
	
	for (uint8_t i=0; i<4; i++)
	{
		switch (numberStr[i])
		{
			case '.': if(pos!=0){pos--; shiftBuf[pos] |= 0x20;}// add point to previous symbol
						 break; 
			case '0': shiftBuf[pos] = 0xD7; break;
			case '1': shiftBuf[pos] = 0x81; break;
			case '2': shiftBuf[pos] = 0xCE; break;
			case '3': shiftBuf[pos] = 0xCB; break;
			case '4': shiftBuf[pos] = 0x99; break;
			case '5': shiftBuf[pos] = 0x5B; break;
			case '6': shiftBuf[pos] = 0x5F; break;
			case '7': shiftBuf[pos] = 0xC1; break;
			case '8': shiftBuf[pos] = 0xDF; break;
			case '9': shiftBuf[pos] = 0xDB; break;
			default: shiftBuf[pos]=0;break;
		}
		pos++;
		if (pos>4) break; // when 3 indicators is filled 
	}
	
		//	HAL_SPI_Transmit(&hspi1, shiftBuf, 3, 1000);  // send to shift reg
	    for (uint8_t i =0; i<3; i++) { sendByteSPI(shiftBuf[i]);}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // on latch pin 
}
////////////
/////

 void TCS34725_Write(unsigned char subAddr, unsigned char* dataBuffer, unsigned char bytesNumber)
{
    unsigned char sendBuffer[10] = {0, };
    unsigned char byte = 0;

    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;
    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }
    HAL_I2C_Master_Transmit(&hi2c1,TCS34725_ADDRESS<<1, sendBuffer, bytesNumber + 1, 1000);
}
void TCS34725_SetIntegrationTime(uint8_t time)
{
    unsigned char cmd = time;

    TCS34725_Write(TCS34725_ATIME, &cmd, 1);
}
void TCS34725_SetGain(uint8_t gain)
{
    unsigned char cmd = gain;

    TCS34725_Write(TCS34725_CONTROL, &cmd, 1);
}
void TCS34725_Enable(void)
{
    unsigned char cmd = TCS34725_ENABLE_PON;

    TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
    HAL_Delay(3);
    cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
    TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
}

void TCS34725_Read(unsigned char subAddr, unsigned char* dataBuffer, unsigned char bytesNumber)
{
    subAddr |= TCS34725_COMMAND_BIT;

    HAL_I2C_Master_Transmit(&hi2c1,TCS34725_ADDRESS<<1, (unsigned char*)&subAddr, 1, 1000);
    HAL_I2C_Master_Receive(&hi2c1,(TCS34725_ADDRESS<<1)| 0x01, dataBuffer, bytesNumber, 1000);
}
void TCS34725_Setup(void)
{
  
	HAL_Delay(100);
    unsigned char status[1] = {0};
    unsigned char r = 0;
    TCS34725_Read(TCS34725_ID, status, 1); 
    r = 	status[0];	
	  HAL_Delay(100);
	  TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_240MS);
    TCS34725_SetGain(TCS34725_GAIN_1X);
	  HAL_Delay(100);
		TCS34725_Enable();
	  HAL_Delay(100);
}
uint16_t TCS34725_GetChannelData(unsigned char reg)
{
    unsigned char tmp[2] = {0,0};
    uint16_t data = 0;

    TCS34725_Read(reg, tmp, 2);
    data = ((uint16_t)tmp[1] << 8) | tmp[0];

    return data;
}
uint8_t TCS34725_GetRawData()
{
    unsigned char status[1] = {0};
		uint16_t GET_C=0;
    uint16_t GET_R=0;
		uint16_t GET_G=0;
		uint16_t GET_B=0;
    status[0] = TCS34725_STATUS_AVALID;

    TCS34725_Read(TCS34725_STATUS, status, 1);

    if(status[0] & TCS34725_STATUS_AVALID)
    {
        GET_C = TCS34725_GetChannelData(TCS34725_CDATAL);  
        GET_R = TCS34725_GetChannelData(TCS34725_RDATAL);  
        GET_G = TCS34725_GetChannelData(TCS34725_GDATAL);  
        GET_B = TCS34725_GetChannelData(TCS34725_BDATAL);
			  c_t = GET_C;
			  r_t = GET_R;
			  g_t = GET_G;
			  b_t = GET_B;
        return 1;
    }
    return 0;
}
void get_TCS34725()
{
	int t = 0;
	HAL_Delay(1000);
	if(TCS34725_GetRawData()==1)
	{
		result = 1;
	}
	else
	{
		result = 0;
	}
}
uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}
////////////
void hx711clkPulse (void)
{
	  HAL_GPIO_WritePin(GPIOA, clk_Pin, GPIO_PIN_SET);   
    delayUs(10);
    HAL_GPIO_WritePin(GPIOA, clk_Pin, GPIO_PIN_RESET);
    delayUs(5);
}
// gets value from HX711 weight module connected to input (1-3) 
// gain can be set to 128 or 64 on channel A, or to 32 but only on channel B (see datasheet)
uint32_t getValueHX711 (uint8_t input, uint8_t gain)
{
	uint32_t data = 0;
  uint32_t  startTime = HAL_GetTick();
	uint16_t dataPin;	
	switch (input)
	{
		case 1: dataPin = data1_Pin; break;
		case 2: dataPin = data2_Pin; break;
		case 3: dataPin = data3_Pin; break;				
	}	
  while(HAL_GPIO_ReadPin(GPIOA, dataPin))
  {
    delayUs(1);
    if(HAL_GetTick() - startTime > 150) // if timeout 
		{return 0;}
  }
  for(int8_t i=0; i<24 ; i++) // read 24 bit value
  {
		HAL_GPIO_WritePin(GPIOA, clk_Pin, GPIO_PIN_SET);   
    delayUs(10);
    if(HAL_GPIO_ReadPin(GPIOA, dataPin)) {data ++;}	
		data <<= 1;	
		HAL_GPIO_WritePin(GPIOA, clk_Pin, GPIO_PIN_RESET);
		delayUs(5);
  }
  data ^= 0x800000; 
  hx711clkPulse(); // if gain==128 by default 
	if (gain== 32||gain==64) {hx711clkPulse();}	// ????????? ???????? ?????????? ??????????????? ?????????? ?? clk
	if (gain== 64)           {hx711clkPulse();}

  return data;    
}

// temperature sensor 
float getDS18B20 (void)
{
	uint8_t dt[8];
	uint16_t raw_temper;
	float temper;
	
	ds18b20_MeasureTemperCmd(SKIP_ROM, 0);
	ds18b20_ReadStratcpad(SKIP_ROM, dt, 0);
	raw_temper = ((uint16_t)dt[1]<<8)|dt[0];
	temper = ds18b20_Convert(raw_temper);
	if(ds18b20_GetSign(raw_temper))temper*=-1;
	signsAllowed=1;
	return temper;
 }

 // Light sensor GY-49 init and get value
 void luxSensInit (void)
 {	 
	 uint8_t buf1[2]={0x02,0x40}; 
	 HAL_I2C_Master_Transmit(&hi2c1,ADDR,&buf1[1],1,1000);
	 HAL_I2C_Master_Transmit(&hi2c1,ADDR,&buf1[2],1,1000);
 } 
 float getluxSens (void)
 {
	uint8_t reg =0x03; 
	int exponent;
	int mantissa;
	float luminance;
	uint8_t buf[2];

	HAL_I2C_Master_Transmit(&hi2c1,ADDR,&reg,1,1000);	
	HAL_I2C_Master_Receive(&hi2c1,ADDR,buf,2,1000);

	exponent = (buf[0] & 0xF0) >> 4;
	mantissa = ((buf[0] & 0x0F) << 4) | (buf[1] & 0x0F);
	luminance = pow(2, exponent) * (float)mantissa * 0.045; 	 
	 signsAllowed=1;
	 return luminance;
 }

 
 float getTempNTC(uint16_t adcTherm)
 {
	 	float r;
		float steinhart;
	  float const serRes = 100.0, rNtc = 100.0, koef = 3900.0;  //  возможно придется подобрать коэф
	  
		r = (float) ((serRes*adcTherm)/(4096.0-adcTherm));
		steinhart = r / rNtc ; // (R/Ro)
		steinhart = log(steinhart); // ln(R/Ro)
		steinhart /= koef; // 1/B * ln(R/Ro)
		steinhart += 1.0 / (25 + 273.15); // + (1/To)
		steinhart = 1.0 / steinhart; 
		steinhart -= 273.15; 
	 signsAllowed=1;
		return steinhart;	
 }
 

 
void adcSeq (void) // read values from 3 adc channels 
{
	
	for (uint8_t i=0; i<3; i++)
	{
		 HAL_ADC_Start(&hadc1);	
		 HAL_ADC_PollForConversion(&hadc1, 100);
		 adc[i] = HAL_ADC_GetValue(&hadc1);
	}	
}
void adcSum (void) // call this function to sum adc values
{
	adcNSamples++;
	adcSeq();
	for (uint8_t i=0; i<3; i++) {adcTotal[i] += adc[i];}
}
void adcAverage (void) // call this before use adc values 
{
	if (adcNSamples!=0)
	{
		for (uint8_t i=0; i<3; i++)	
		{
			adc[i] = (uint16_t) round (adcTotal[i] / adcNSamples);
			adcTotal[i]=0;
		}
		adcNSamples=0;	
	}
}
float getVoltage(uint16_t adc) { return (3.34/4095)*adc;}


float getAHT20 (void)	
{
	uint8_t AHT10_RX_Data[6];
	uint32_t AHT10_ADC_Raw;
	float humidity;
	uint8_t AHT10_TmpHum_Cmd = 0xAC;

	HAL_I2C_Master_Transmit(&hi2c1, AHT10_Adress, &AHT10_TmpHum_Cmd, 1, 100);
	HAL_Delay(100); // Delay must be > 75 ms
	HAL_I2C_Master_Receive(&hi2c1, AHT10_Adress, (uint8_t*)AHT10_RX_Data, 6, 100);
	AHT10_ADC_Raw = ((uint32_t)AHT10_RX_Data[1] << 12) | ((uint32_t)AHT10_RX_Data[2] << 4) | (AHT10_RX_Data[3] >> 4);
	humidity = (float)(AHT10_ADC_Raw*10.00);
	if(humidity < 200)
	{
		aht_20_t = humidity;
	}
	else
	{
		humidity = aht_20_t;
	}
	HAL_Delay(100);
	signsAllowed=0;
	return humidity;
}

float getUVIndex()
{	
 
	float const  DUVfactor    = 1.61; 	
	float voltage = getVoltage(adc[0]);
	if (voltage<=1.0) return 0;
	voltage -= 1.0;    // subtract for zero point.
  float mWcm2 = voltage * (15.0 / 1.8);
	signsAllowed=0;
	return mWcm2 * DUVfactor; // return index 

	
}

// обработчик прерываний со входов 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(currentModule == ID_MODULE_FORCE || currentModule == ID_MODULE_WEIGHT)
	 {
	  interruptOnSwitch = 0;
	  if((GPIO_Pin == exti1_Pin)&&(!HAL_GPIO_ReadPin(GPIOA, exti1_Pin)))
	    {
		   interruptOnSwitch = 1;
	    }
	   if((GPIO_Pin == exti2_Pin)&&(!HAL_GPIO_ReadPin(GPIOB, exti2_Pin)))
	    {
		   interruptOnSwitch = 2;
	    }
	   if((GPIO_Pin == exti3_Pin)&&(!HAL_GPIO_ReadPin(GPIOB, exti3_Pin)))
	    {
		   interruptOnSwitch = 3;
	    }
		debounceLong =0;
	  HAL_NVIC_DisableIRQ(EXTI0_IRQn); 
	  HAL_NVIC_DisableIRQ(EXTI1_IRQn); 
	  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); 
	  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); 
	  irqFlag=1;
		period = HAL_GetTick() - periodLast;
		periodLast = HAL_GetTick();
		if (interruptsCount<10)
		{
			timeInterv[interruptsCount] = HAL_GetTick() - timeStart; // 
			interruptsCount++;	
		}
	   if(GPIO_Pin == exti4_Pin) 
        {
				  interruptOnSwitch = 4;
				  debounceLong =1;
				} 
   }
	 //////////////////////////
	 
	 if(currentModule == ID_MODULE_CAPACITY)
	 {
	 clearEXTIs();	 
	 if(GPIO_Pin == exti1_Pin)
	  {
		  cap_count++;
		  cap_count_1++;
	  }
   }
}

void enableExtis (void)
{
	  if(currentModule == ID_MODULE_FORCE || currentModule == ID_MODULE_WEIGHT)
		{
			uint16_t debounceTime=20;
	    if(currentModule!=ID_MODULE_TIME || debounceLong) {debounceTime=350;} // if interrupt comes from optical gate then debouncing is not required
	    if(irqFlag && ((HAL_GetTick() - periodLast) > debounceTime) )// 
	    {		
		   irqFlag =0;
       clearEXTIs();
		   HAL_NVIC_EnableIRQ(EXTI0_IRQn); 
		   HAL_NVIC_EnableIRQ(EXTI1_IRQn); 
		   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); 
		   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); 
	    }
		}
		else
		{
	  clearEXTIs();
		HAL_NVIC_EnableIRQ(EXTI0_IRQn); 
		HAL_NVIC_EnableIRQ(EXTI1_IRQn); 
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); 
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		}
		
}
void disableEXTIs (void)
{
	HAL_NVIC_DisableIRQ(EXTI0_IRQn); 
	HAL_NVIC_DisableIRQ(EXTI1_IRQn); 
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn); 
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

void clearEXTIs (void)
{
			__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
		NVIC_ClearPendingIRQ(EXTI0_IRQn);
		NVIC_ClearPendingIRQ(EXTI1_IRQn); 
		NVIC_ClearPendingIRQ(EXTI9_5_IRQn); 
		NVIC_ClearPendingIRQ(EXTI15_10_IRQn); 
}
void timerTask (void)
{
	
	HAL_GPIO_WritePin(clk_GPIO_Port, clk_Pin, GPIO_PIN_SET);
	enableExtis();
	if(!flag_time_task)
	{
		flag_time_task = 1;
		displayFloat((float)0);
		//
		sprintf(bufUsb, USB_STRING_FORMAT, currentModule, 0.0);
		CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));	
		//
	}
	if (interruptOnSwitch==4)
	{
		interruptOnSwitch = 0;
		if(!time_started)
		{
			displayFloat((float)0);
			//
			sprintf(bufUsb, USB_STRING_FORMAT, currentModule, 0.0);
		  CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
			//
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			interruptsCount = 0;
			old_count = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // turn on LED
			time_started = 1;
			periodLast_t = HAL_GetTick();
		}
		else
		{
			displayFloat((float)0);
			//
			sprintf(bufUsb, USB_STRING_FORMAT, currentModule, 0.0);
		  CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
			//
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // turn on LED
			time_started = 0;
			lcd_time_task_count = 0;
		}
		
	}
	if(time_started == 1)
	{
		if(old_count<interruptsCount)
		{
			old_count=interruptsCount;
			displayFloat((float)timeInterv[interruptsCount-1]/1000);
			//
			sprintf(bufUsb, USB_STRING_FORMAT, currentModule, (float)timeInterv[interruptsCount-1]/1000);
		  CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
			//
		}
	}
	if((time_started == 0)&&(interruptsCount)>0)
	{
		clearEXTIs();
		if ((!HAL_GPIO_ReadPin(data1_GPIO_Port, data2_Pin))&&(lcd_time_task_count<interruptsCount)&&(data_2_f==0))
		{
			data_2_f = 1;
			
			displayFloat((float)timeInterv[lcd_time_task_count]/1000);
			lcd_time_task_count++;
			HAL_Delay(1000);
		}
		if(HAL_GPIO_ReadPin(data1_GPIO_Port, data2_Pin))
		{
			data_2_f = 0;
		}
		if ((!HAL_GPIO_ReadPin(data1_GPIO_Port, data1_Pin))&&(lcd_time_task_count>1)&&(data_1_f==0))
		{
			data_1_f = 1;
			lcd_time_task_count--;
			displayFloat((float)timeInterv[lcd_time_task_count-1]/1000);
			HAL_Delay(1000);
		}
		if(HAL_GPIO_ReadPin(data1_GPIO_Port, data1_Pin))
		{
			data_1_f = 0;
		}
	}
	clearEXTIs();
}	
void lcMeterTask (void)
{
	
	period_t = HAL_GetTick();
	if((period_t - periodLast)>=2000)
	{
		if(cap_count>0)
		{
			f = 1443000/((float)cap_count /((period_t - periodLast)/1000.0));
			f = f/450.0;
			f = f/1.20;
		}	
		if(f<400.0)
		{
			f_all = f;
			if (f_all<0.125)
			{
				f_all = f_all-0.005;
			}
		}
		periodLast = period_t;
		cap_count = 0;
	}
	if(cap_count_1>0)
	{
		f_1 = (float)DWT_CYCCNT/72000000.0;
		f_1 = 1.0/f_1;
		f_1 = 1443000/f_1;
		f_1 = f_1/450.0;
		f_1 = f_1/1.25;
		if(f_1>399.0)
		{
			f_all = f_1;
		}
		dwt_init();
		cap_count_1=0;
	}
	 
}

void rj45_connectors_recognise (void)  // узнать какие датчики силы подключены к разьемам
{	
	for(uint8_t i =0; i<5; i++) {adcSum(); HAL_Delay(10);}
	adcAverage();
	
	const uint16_t adc1N = 718;
	const uint16_t adc5N = 1241;
	const uint16_t adc50N = 2047;
	const uint8_t adcError = 20;

	for (uint8_t i =0; i<3; i++)
	{
		rj45_connected[i] = 0;
		if(adc[i]+adcError>adc1N && adc[i]-adcError<adc1N)         {rj45_connected[i]=RJ45_1_N;}
		if(adc[i]+adcError>adc5N && adc[i]-adcError<adc5N)         {rj45_connected[i]=RJ45_5_N;}
		if(adc[i]+adcError>adc50N && adc[i]-adcError<adc50N)       {rj45_connected[i]=RJ45_50_N;}
		if (adc[i]>2000 && currentModule==ID_MODULE_WEIGHT)        {rj45_connected[i]=RJ45_WEIGHT;}
		if (adc[i]>2000 && currentModule==ID_MODULE_DIF_PRESSURE)  {rj45_connected[i]=RJ45_PRESSURE;}
	}
}

uint32_t hx711Average (uint8_t input, uint8_t gain)
{
	float const hx711Samples=20.0;
	unsigned long hx711Sum=0;
	getValueHX711(input,gain); // refresh the gain settings
	for (uint8_t i = 0; i< hx711Samples; i++)
	{
		hx711Sum+= getValueHX711(input,gain);
		HAL_Delay(20);
	}
	hx711Sum /= hx711Samples;
	return hx711Sum;
}



void Hx711Task (void)
{		
		if (input == 1)	{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		else if (input == 2)	{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		else if (input == 3)	{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
		uint8_t ff = 0;
		hx711Value[input - 1] = hx711Average(input,128);
		if (interruptOnSwitch == input)	{
			enableExtis();
			int timeStart = HAL_GetTick();
			interruptOnSwitch = 0;
			while(HAL_GetTick() - timeStart < 2500)	{
				if (HAL_GetTick() - timeStart == 0 || HAL_GetTick() - timeStart == 750 || HAL_GetTick() - timeStart == 1500 || HAL_GetTick() - timeStart == 2250)	{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_12, GPIO_PIN_RESET);
				}
				if (HAL_GetTick() - timeStart == 250 || HAL_GetTick() - timeStart == 1000 || HAL_GetTick() - timeStart == 1750 || HAL_GetTick() - timeStart == 2500)	{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_12, GPIO_PIN_RESET);
				}
				if (HAL_GetTick() - timeStart == 500 || HAL_GetTick() - timeStart == 1250 || HAL_GetTick() - timeStart == 2000)	{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_13, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				}
				enableExtis();
				if (interruptOnSwitch == 4 || interruptOnSwitch == input)	{
					ff = 1;
					if (interruptOnSwitch == input){
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
						HAL_Delay(700);
						callibr300 = hx711Average(input,128);
						
					}
					break ;
				}
				interruptOnSwitch = 0;
			}
			if (ff == 0)	{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				offset[input - 1] = hx711Average(input,128);
				interruptOnSwitch=0;
			}
		}
    hx711Value[input - 1] -= offset[input - 1];
		hx711Value[input - 1] *= 303 / (callibr300 - offset[input - 1]);
		if(hx711Value[input - 1]<0.0)
		{
		hx711Value[input - 1] =  - (hx711Value[input - 1]);	
		}
		hx711Value[input - 1] = hx711Value[input - 1]/102.0;
		sprintf(bufUsb, USB_STRING_FORMAT, (int)(currentModule + (input+1)*100), hx711Value[input - 1]);
		CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));	
		displayFloat(hx711Value[input - 1]);
		//
		if (interruptOnSwitch != input && interruptOnSwitch != 0)	{
			input = interruptOnSwitch;
		}
    interruptOnSwitch=0;
    enableExtis();
		
}


void weightTask(void)	
{
	averageAdc_for_N_msec(10);
		if (input == 1)	{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		else if (input == 2)	{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		else if (input == 3)	{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
		if (adc[input - 1] < 10)	{
			callibr300 = 9436863;
			if (ff == 0)
				offset[input - 1] = 9297608;
		}
		else	{
			callibr300 = 24897586;
			if (ff == 0)
				offset[input - 1] = 25143904;
		}
		hx711Value[input - 1] = hx711Average(input,128);
		if (interruptOnSwitch == input)	{
			ff = 1;
			enableExtis();
			int timeStart = HAL_GetTick();
			interruptOnSwitch = 0;
			while(HAL_GetTick() - timeStart < 2500)	{
				if (HAL_GetTick() - timeStart == 0 || HAL_GetTick() - timeStart == 750 || HAL_GetTick() - timeStart == 1500 || HAL_GetTick() - timeStart == 2250)	{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_12, GPIO_PIN_RESET);
				}
				if (HAL_GetTick() - timeStart == 250 || HAL_GetTick() - timeStart == 1000 || HAL_GetTick() - timeStart == 1750 || HAL_GetTick() - timeStart == 2500)	{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_12, GPIO_PIN_RESET);
				}
				if (HAL_GetTick() - timeStart == 500 || HAL_GetTick() - timeStart == 1250 || HAL_GetTick() - timeStart == 2000)	{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_13, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				}
				enableExtis();
				interruptOnSwitch = 0;
			}
			if (ff == 1)	{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
				offset[input - 1] = hx711Average(input,128);
				interruptOnSwitch=0;
			}
		}
    hx711Value[input - 1] -= offset[input - 1];
		hx711Value[input - 1] *= 303 / (callibr300 - offset[input - 1]);
		if(hx711Value[input - 1]<0.0)
		{
			hx711Value[input - 1] =  - (hx711Value[input - 1]);	
		}
		if (hx711Value[input - 1] < 0.25)	{
			hx711Value[input - 1] = 0;
		}
		sprintf(bufUsb, USB_STRING_FORMAT, (int)(currentModule + (input+1)*100), hx711Value[input - 1]);
		CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
		if (hx711Value[input - 1] > 999)
			hx711Value[input - 1] = 999;
		//signsAllowed = 0;
		displayFloat(hx711Value[input - 1]);
		//
		if (interruptOnSwitch != input && interruptOnSwitch != 0)	{
			input = interruptOnSwitch;
		}
    interruptOnSwitch=0;
    enableExtis();
}


float getCOppm(uint16_t adc)
{
	const float coefficient_A =19.32;
	const float coefficient_B =-0.64;
	float val = getVoltage(adc);
	val = (3.3-val)/val;
	signsAllowed=0;
	return (float)(coefficient_A * pow(val, coefficient_B));
}

float getOxygenPercent(uint16_t adc)
{
	//float const maxVolt = 1.416; // 20 % O2
	float volt = getVoltage(adc);
	signsAllowed=0;
	return (float)(20/1.416)*volt;
}


void dacWrite(uint16_t data)
{	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CS
	for(uint8_t i =0; i<16; i++)
	{
		if (data & 0x8000) 
			{ 
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//SDI
			}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);//SDI
			HAL_Delay(1);
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//SCK	
		data<<=1;
		HAL_Delay(1);
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);//SCK	
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // CS
	delayUs(1);
}

float getResistance(void)
{
  // bit 15: 0 for DAC A, 1 for DAC B. (Always 0 for MCP49x1.)
  // bit 14: buffer VREF?
  // bit 13: gain bit; 0 for 1x gain, 1 for 2x (thus we NOT the variable)
  // bit 12: shutdown bit. 1 for active operation
  // bits 11 through 0: data 
 int flagRelay = 1;
 for (dac = 4; dac < 4095; dac++)
 {
	if (result < 0){
			result = 0;
	}
	if (dac == 4 && flagRelay && Rx > 12)	{
		HAL_GPIO_WritePin(relayPort, relayPin, GPIO_PIN_SET);
		continue ;
	}
	else if (dac == 4) {
   flagRelay = 0;
   HAL_GPIO_WritePin(relayPort, relayPin, GPIO_PIN_RESET);
  }
	
  uint16_t data = 0<<15 | 0<<14 | 1<<13 | 1<<12;
  data|= dac; 
  dacWrite(data);
  averageAdc_for_N_msec(10);
	
	if (adc[0] > 4000 && flagRelay){
		dac -= 3;
		adc[0] = 0;
		continue;
	}
 
   if(adc[0]<700)  {  dac+=61; if(!flagRelay) dac+=30; continue;}
   if(adc[0]<900)  {  dac+=15; continue;}
   if(adc[0]<1100)  {  dac+=15; continue;}
   if(adc[0]<1200) { dac+=10; continue;}
	 if (Rx < 0.03 && Rx > 0 && dac < 3500 && flagRelay != 1)
	 {
		 dac = 3500;
		 continue ;
	 }
   
  averageAdc_for_N_msec(100);
  Rx = (float)adc[0] / (float)dac;
	Rx -= 1.00;
	if (flagRelay)
		Rx *= 10;
	else
		Rx /= 2;
	if (Rx < 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		return (Rx * 1000);
	}
	else	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	}
  return (Rx * 1000);
  }
}

float getVoltageCurrent(void)
{	
	float R=0;
	uint8_t type=0;
	float inVoltage = 0;
	float X=0; // коэфф. делителя 
	signsAllowed = -1;
	switch (currentModule) // текущий модуль 
	{
		case ID_MODULE_CURRENT_10A:    type=DC_CURRENT; R=0.01; break;
		case ID_MODULE_CURRENT_200MA:  type=DC_CURRENT; R=0.5; break;
		case ID_MODULE_CURRENT_2MA:    type=DC_CURRENT; R=50; break;
		case ID_MODULE_CURRENT_1A_AC:  type=AC_CURRENT; R=0.1; break;
		case ID_MODULE_VOLTAGE_200MV: X=2; type=DC_VOLTAGE; break;
		case ID_MODULE_VOLTAGE_30V: X=331;   type=DC_VOLTAGE; break;
		case ID_MODULE_VOLTAGE_30V_AC: X=331; type=AC_VOLTAGE; break;
	}
	if((type==DC_VOLTAGE)&&(getVoltage(adc[0])>0.3)&&(currentModule == ID_MODULE_VOLTAGE_30V ))
	{
		inVoltage = getVoltage(adc[0])*9.447;
	}
	if((type==DC_VOLTAGE)&&(getVoltage(adc[1])>0.01)&&(getVoltage(adc[0])<0.01)&&(currentModule == ID_MODULE_VOLTAGE_30V ))
	{
		inVoltage = getVoltage(adc[1])*42.5;
		signsAllowed = 1;
		inVoltage =  - (inVoltage);
	}
	if((type==DC_CURRENT)&&(getVoltage(adc[0])>0.1)&&(currentModule == ID_MODULE_CURRENT_10A ))
	{
		inVoltage = ((getVoltage(adc[0])-0.06)/34)*100;
	}
	if((type==DC_CURRENT)&&(getVoltage(adc[1])>0.1)&&(currentModule == ID_MODULE_CURRENT_10A ))
	{
		inVoltage = ((getVoltage(adc[1])-0.05)/33)*100;
		signsAllowed = 1;
		inVoltage =  - (inVoltage);
	}
	///////
	
	if((type==DC_CURRENT)&&(getVoltage(adc[0])>0.03)&&(currentModule == ID_MODULE_CURRENT_200MA ))
	{
		inVoltage = getVoltage(adc[0])/ 0.0171608305 ;
		if(inVoltage<56.0)
		{
			inVoltage = inVoltage-1.0;
		}
	}
	
	if((type==DC_CURRENT)&&(getVoltage(adc[1])>0.0005)&&(currentModule == ID_MODULE_CURRENT_200MA ))
	{
		inVoltage = getVoltage(adc[1]) / 0.0162065446;
		if(inVoltage<56.0)
		{
			inVoltage = inVoltage+1.0;
		}
		signsAllowed = 1;
		inVoltage =  - (inVoltage);
	}

	
	if((type==DC_VOLTAGE)&&(getVoltage(adc[0])>0.09)&&(currentModule == ID_MODULE_VOLTAGE_200MV ))
	{
		inVoltage = getVoltage(adc[0]) / 0.00672078133;
		inVoltage = inVoltage - 7.0;
	}
	
	if((type==DC_VOLTAGE)&&(getVoltage(adc[0])<0.07) &&(currentModule == ID_MODULE_VOLTAGE_200MV ))
	{
		inVoltage = getVoltage(adc[1]) / 0.000309938956;
		signsAllowed = 1;
		inVoltage =  - (inVoltage);
	}
	if((type==DC_CURRENT)&&(currentModule == ID_MODULE_CURRENT_2MA ))
	{
		if(getVoltage(adc[0])>2.573)
		{
		inVoltage = (getVoltage(adc[0]) - 2.57) / 0.0415 ;
		inVoltage = inVoltage-0.2;
		}
		else
		{
			inVoltage = (2.573 - getVoltage(adc[0])) / 0.0415 ;
			signsAllowed = 1;
			inVoltage = inVoltage+0.2;
		  inVoltage =  - (inVoltage);
		}
	}
	

	return inVoltage;
}
float get_O2()
{
	float in = 0;
	signsAllowed = -1;
	averageAdc_for_N_msec(500);
	in = (getVoltage(adc[0])*1000)/60.0;
	return in-2.0;
}

 void pres_task()
{
	signsAllowed = -1;
	int inputp = 0;
	if(firstConnectPress){
		HAL_Delay(2500);
	}
	averageAdc_for_N_msec(500);
	if (!HAL_GPIO_ReadPin(GPIOA, data1_Pin))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		if (firstConnectPress){
			averageAdc_for_N_msec(500);
			firstConnectPress = 0;
			offsetPress = (getVoltage(adc[0])-2.5)*133.0;
			if(getVoltage(adc[0])<2.45)
				offsetPress = (2.5-getVoltage(adc[0]))*(-133.0);
			if (offsetPress < 0)
				offsetPress *= -1;
		}
		signsAllowed = 1;
		in_pres = (getVoltage(adc[0])-2.5)*133.0;
		if(getVoltage(adc[0])<2.45)
				in_pres = (2.5-getVoltage(adc[0]))*(-133.0);
		in_pres -= offsetPress;
		in_pres /= 0.28;
		inputp = 1;
 }
	else if (!HAL_GPIO_ReadPin(GPIOA, data2_Pin))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		if (firstConnectPress){
			averageAdc_for_N_msec(500);
			firstConnectPress = 0;
			offsetPress = (getVoltage(adc[1])-2.5)*133.0;
			if(getVoltage(adc[1])<2.45)
				offsetPress = (2.5-getVoltage(adc[1]))*(-133.0);
			if (offsetPress < 0)
				offsetPress *= -1;
		}
		signsAllowed = 1;
		in_pres = (getVoltage(adc[1])-2.5)*133.0;
		if(getVoltage(adc[1])<2.45)
				in_pres = (2.5-getVoltage(adc[1]))*(-133.0);
		in_pres -= offsetPress;
		in_pres /= 0.28;
		inputp = 2;
 }
	else if (!HAL_GPIO_ReadPin(GPIOA, data3_Pin))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		if (firstConnectPress){
			averageAdc_for_N_msec(500);
			firstConnectPress = 0;
			offsetPress = (getVoltage(adc[2])-2.5)*133.0;
			if(getVoltage(adc[2])<2.45)
				offsetPress = (2.5-getVoltage(adc[2]))*(-133.0);
			if (offsetPress < 0)
				offsetPress *= -1;
		}

		signsAllowed = 1;
		in_pres = (getVoltage(adc[2])-2.5)*133.0;
		if(getVoltage(adc[2])<2.45)
				in_pres = (2.5-getVoltage(adc[2]))*(-133.0);
		in_pres -= offsetPress;
		in_pres /= 0.28;
		inputp = 3;
 }
	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		firstConnectPress = 1;
		//displayFloat(0);
	}
	//displayFloat(in_pres);
	sprintf(bufUsb, USB_STRING_FORMAT, (int)(currentModule + (inputp+1)*100), in_pres);
	CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
}
float getNitratSensor()
{
	/* необходимо приготовить два раствора с известными концентрациями и измерить напряжение на выходе модуля  
    затем посчитайте коэффициент наклона прямой зависимости K=((С2 -C1)/(V2-V1)) 
	   где C - концентрация */
	//float K= 0.1234;
	signsAllowed=0;
	averageAdc_for_N_msec(100);
	//return getVoltage(adc[0])*K;
	return getVoltage(adc[0]);
}

void spirograph (void)
{
	signsAllowed = -1;
	float sendPress;
	int inputp = 0;
	zero = 0;
	averageAdc_for_N_msec(100);
	if(firstConnectPress){
		zero = 0xD7;
		displayFloat(0);
		waiting_animation();
		waiting_animation();
		displayFloat(0);
	}
	if (!HAL_GPIO_ReadPin(GPIOA, data1_Pin))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		if (firstConnectPress){
			averageAdc_for_N_msec(100);
			firstConnectPress = 0;
			offsetPress = (getVoltage(adc[0])-2.5);
			if(getVoltage(adc[0])<2.45)
				offsetPress = (2.5-getVoltage(adc[0]));
		}
		signsAllowed = 1;
				in_pres = (2.5-getVoltage(adc[0]));
		in_pres -= offsetPress;
		inputp = 1;
 }
	else if (!HAL_GPIO_ReadPin(GPIOA, data2_Pin))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		if (firstConnectPress){
			averageAdc_for_N_msec(100);
			firstConnectPress = 0;
			offsetPress = (getVoltage(adc[1])-2.5);
			if(getVoltage(adc[1])<2.45)
				offsetPress = (2.5-getVoltage(adc[1]));
		}
		signsAllowed = 1;
				in_pres = (2.5-getVoltage(adc[1]));
		in_pres -= offsetPress;
		inputp = 2;
 }
	else if (!HAL_GPIO_ReadPin(GPIOA, data3_Pin))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		if (firstConnectPress){
			averageAdc_for_N_msec(100);
			firstConnectPress = 0;
			offsetPress = (getVoltage(adc[2])-2.5);
			if(getVoltage(adc[2])<2.45)
				offsetPress = (2.5-getVoltage(adc[2]));
		}

		signsAllowed = 1;
				in_pres = (2.5-getVoltage(adc[2]));
		in_pres -= offsetPress;
		inputp = 3;
 }
	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		firstConnectPress = 1;
		in_pres = 0;
		offsetPress = 0;
	}
	in_pres *= -1;
	in_pres = in_pres < 0.002 ? 0 : in_pres;
	in_pres *= 11.63;
	if (startSpirograph == 0 && in_pres != 0)	{
		startSpirograph = 1;
		spirographCounter = 0;
		spirographCounterSec = 0;
		spirographSum = 0;
	}
	if (startSpirograph && in_pres)	{
		signsAllowed = 0;
		displayFloat(in_pres);
		spirographSum += in_pres;
		spirographCounter++;
		if (spirographCounter == 10){
			spirographCounterSec++;
			spirographCounter = 0;
		}
	}
	if (startSpirograph && !in_pres)	{
		startSpirograph = 0;
		in_pres = spirographSum / 10 * (spirographCounterSec + spirographCounter / 10);
		sendPress = in_pres;
		sprintf(bufUsb, USB_STRING_FORMAT, currentModule, in_pres);
		if (in_pres)
			CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
		signsAllowed = 0;
		displayFloat(in_pres);
		HAL_Delay(500);
	}
}

void averageAdc_for_N_msec (uint16_t msec)
{
	uint32_t time = HAL_GetTick();
	while (HAL_GetTick() - time < msec) {adcSum();}
	adcAverage();
}

void waiting_animation(void)
{
    uint8_t byte = 0x1;
		for (uint8_t i =0; i<7; i++)
		{		
			for (uint8_t j=0; j<3; j++) sendByteSPI(byte);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // toggle latch pin 
			HAL_Delay(100);
			for (uint8_t j=0; j<3; j++) sendByteSPI(0x0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // toggle latch pin 
			HAL_Delay(100);
			byte<<=1;
		}		

}
float GAZ()
{
	averageAdc_for_N_msec(500);
	return ((((getVoltage(adc[0])/1000.0)*1.44) / 0.000732)*1.162)/2.0;
}
void INDUCT()
{
	TIM1->PSC = 2; // ????????? ??????? 1 ???
	//HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	
	for ( diap=0; diap<8; diap++) 
	{
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
		HAL_Delay(100);
		averageAdc_for_N_msec(10);
		if(getVoltage(adc[0])>0.2) 
    {
		TIM1->PSC *=4;
		} 
		else 
    {
    freq = 72000000/(73*(TIM1->PSC));
	 induct_result = 300 / (5.0 / getVoltage(adc[0]));
    induct_result = induct_result / (6.28*freq );	
			induct_result = induct_result*1000;
			if(freq == 123287)
			{
				induct_result = induct_result*2;
			}
			if(freq == 493150)
			{
				induct_result = induct_result*5;
			}
		break;
		}
   }
	 
	 freq = 72000000/(73*(TIM1->PSC));
	
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  port_init(); // init DS18B20 pin 
	uint8_t currentModule_prev =255;
	_Bool init_needed=0;
	HAL_ADCEx_Calibration_Start(&hadc1);
	displayFloat(0);
	enableExtis();
	HAL_Delay(100);
	init_needed = 1;
	//
//if ((&hUsbDeviceFS->dev_state != USBD_STATE_CONFIGURED)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//
	//
  while (1)
  {	 
		
		
     currentModule = getModuleId();
		 if((currentModule!=currentModule_prev)&&(currentModule>0)) 
				{
					firstConnectPress = 1;
					currentModule_prev = currentModule;
					sprintf(bufUsbStatus, "#%d#con#\n", currentModule);
          //
          if(currentModule == ID_MODULE_TIME)
		       {
			       disableEXTIs();   
		       }
					 else
					 {
						 enableExtis();
					 }
					 if(currentModule == ID_MODULE_DIF_PRESSURE || currentModule == ID_MODULE_SPIROMETER)
					 {
						 GPIO_InitTypeDef GPIO_InitStruct = {0};
						 GPIO_InitStruct.Pin = data1_Pin|data2_Pin|data3_Pin;
             GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
             GPIO_InitStruct.Pull = GPIO_PULLUP;
             HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					 }
					 else
					 {
					   GPIO_InitTypeDef GPIO_InitStruct = {0};
						 GPIO_InitStruct.Pin = data1_Pin|data2_Pin|data3_Pin;
             GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
             GPIO_InitStruct.Pull = GPIO_PULLDOWN;
             HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					 }
          //					
				}
			if((currentModule!=currentModule_prev)&&(currentModule<1)) 
				{
					currentModule_prev = currentModule;
 					sprintf(bufUsbStatus, "#%d#dis#\n", currentModule);
				}	
				if((HAL_GetTick() - sendStatusTime)>3000) // send which module is connected every 3s
				{
					sendStatusTime = HAL_GetTick();								
					while(CDC_Transmit_FS((uint8_t*)bufUsbStatus,strlen(bufUsbStatus))==USBD_BUSY){ // wait if usb is busy
						if((HAL_GetTick() - sendStatusTime)>1500) break ; // timeout
					}
				}
				if((currentModule==currentModule_prev)&&(currentModule>0))
				{
					zero = 0xD7;
					switch (currentModule)	{
						case ID_MODULE_CO_GAS:
							result = GAZ();
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							signsAllowed = -1;
							if(result<1000)
								displayFloat(result);
							else
								displayFloat(result/1000.0);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_ATM_PRESSURE:
						{
							static uint32_t tmp=0;
							static uint32_t tmp_old=0;
							if (init_needed)
							{
								read_calliberation_data();
								init_needed = 0;
							}
							tmp = BMP180_GetPress();
							if(tmp!= tmp_old)
							{
								tmp_old = tmp;
								signsAllowed = -1;
								displayFloat((float)tmp / 100.0);
							}
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, (float)tmp / 100.0);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						}
						case ID_MODULE_VOLTAGE_30V:
							averageAdc_for_N_msec(500);
							result = getVoltageCurrent();
							displayFloat(result);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_CURRENT_10A:
							averageAdc_for_N_msec(500);
							result = getVoltageCurrent();
							displayFloat(result);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_VOLTAGE_200MV:
							averageAdc_for_N_msec(500);
							result = getVoltageCurrent();
							displayFloat(result);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_CURRENT_200MA:
							averageAdc_for_N_msec(500);
							result = getVoltageCurrent();
							displayFloat(result);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_CURRENT_2MA:
							averageAdc_for_N_msec(500);
							result = getVoltageCurrent();
							displayFloat(result);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_CAPACITY:
							lcMeterTask(); 
							if(cap_old!= f_all)
							{	
								cap_old = f_all;	
								result = cap_old;
								//result = result / 28.07861;
								//result = result / 5.4380;
								if(result>0.0099)
								{
									displayFloat(result);
									//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
									//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
								}	
								else
								{
									displayFloat(result*1000.0);
									//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
									//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
								}							 
								sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result*1000.0);
								CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							}
							break ;
						case ID_MODULE_RESISTANCE:
							result = getResistance(); 
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							if (Rx < 1)
								displayFloat(result);
							else
								displayFloat(result / 1000);
							break ;
						case ID_MODULE_TEMP:
							if(init_needed) ds18b20_init(SKIP_ROM);
							result = getDS18B20(); 
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_TIME:
							disableEXTIs();   
							timerTask();
							break ;
						case ID_MODULE_FORCE:
							Hx711Task();
							break ;
						case ID_MODULE_DIF_PRESSURE:
							pres_task();
							break ;
						case ID_MODULE_SPIROMETER:
							spirograph();
							break ;
						case ID_MODULE_WEIGHT:
							weightTask();
							break ;
						case ID_MODULE_INDUCTANCE:
							INDUCT();
							result = induct_result;
							break ;
						case ID_MODULE_RADIATION:
							HAL_Delay(1000);
							result = getRadSens()/3.6;
							displayFloat(result);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_LIGHT:
							if(init_needed)
							{
								TCS34725_Setup();
								init_needed = 0;
							}
							//HAL_Delay(1500);
							//result = getluxSens();
							get_TCS34725();
							result = (float)calculateLux(r_t,g_t,b_t);
							/*r_p = (float)(r_t+g_t+b_t);
							r_p = (float) r_t / ((float)r_p / 100.0);
							g_p = (float)(r_t+g_t+b_t);
							g_p = (float) g_t / ((float)g_p / 100.0);
							b_p = (float)(r_t+g_t+b_t);
							b_p = (float) b_t / ((float)b_p / 100.0);*/
							displayFloat(result);
							// RGB info output///////////////////////////////////////////////
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							HAL_Delay(100);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule+100,(float) r_p);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							HAL_Delay(100);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule+200,(float) g_p);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							HAL_Delay(100);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule+300,(float) b_p);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_HUMIDITY:
							result=getAHT20();  
							HAL_Delay(500);
							displayFloat(result);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							displayFloat(result);
							break ;
						case ID_MODULE_ULTRAV:
							averageAdc_for_N_msec(100);
							result = getUVIndex();
							displayFloat(result);
							sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result);
							CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
							break ;
						case ID_MODULE_NITRATES:
							result = getNitratSensor();
							break ;
					}
	      }
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
				
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

