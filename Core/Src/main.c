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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_STRING_FORMAT "#%d#%.4f#\n" // формат строки отправляемой в ком-порт 
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
	ID_MODULE_ULTRAV	
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
_Bool volatile irqFlag=0;
_Bool volatile  debounceLong = 1;

char bufUsb[20]; // buffer to send to USB
char bufUsbStatus[20]; // buffer to send to USB
float result =0;
unsigned long volatile impulseCount=0;
_Bool lcMeterStarted=0;
_Bool volatile timerStopped=0;

	uint32_t sendStatusTime=0; 
	uint32_t sendTime=0; 
/////////////////////////////////
	uint8_t shiftBuf[3]={0};	
	char numberStr[10]={0};
	
	_Bool USBconnected=0;
	
/* USER CODE END PV */

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
		if (pos>2) break; // when 3 indicators is filled 
	}
	
		//	HAL_SPI_Transmit(&hspi1, shiftBuf, 3, 1000);  // send to shift reg
	    for (uint8_t i =0; i<3; i++) { sendByteSPI(shiftBuf[i]);}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);  // on latch pin 
}


// HX711 clock pulse 
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
  while(HAL_GPIO_ReadPin(GPIOA, dataPin) == GPIO_PIN_SET)
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
	if (gain== 32||gain==64) {hx711clkPulse();}	// настройка усиления происходит дополнительными импульсами на clk
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
	luminance = pow(2, exponent) * mantissa * 0.045; 	 
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
float getVoltage(uint16_t adc) { return (3.3/4095)*adc;}


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
	humidity = (float)(AHT10_ADC_Raw*100.00/1048576.00);
	HAL_Delay(100);
	signsAllowed=0;
	return humidity;
}

float getUVIndex (uint16_t adcUV)
{	
  float const  DUVfactor    = 1.61; 	
	float voltage = getVoltage(adcUV);
	if (voltage<=1.0) return 0;
	voltage -= 1.0;    // subtract for zero point.
  float mWcm2 = voltage * (15.0 / 1.8);
	signsAllowed=0;
	return mWcm2 * DUVfactor; // return index 
}

// обработчик прерываний со входов 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==GPIO_PIN_8){ impulseCount++; return;}  // используется для измерения частоты, если есть прерывание - выходим из функции 
	interruptOnSwitch = 0;
	if(GPIO_Pin == exti1_Pin) interruptOnSwitch = 1; 
	if(GPIO_Pin == exti2_Pin) interruptOnSwitch = 2;
	if(GPIO_Pin == exti3_Pin) interruptOnSwitch = 3;

		debounceLong =0;
  // disable interrupts to prevent false triggering 
	// enableExtis() must be placed in endless loop 
	// to enable interrupts after some time 
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
		
	if(GPIO_Pin == exti4_Pin) {interruptOnSwitch = 4;debounceLong =1;} 

}

void enableExtis (void)
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
	enableExtis();
	
	if (interruptOnSwitch==4) // если нажали пуск
	{
		interruptOnSwitch = 0;
		
		if(!time_started)
		{
			time_started=1;
			interruptsCount = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // turn on LED
			timeStart = HAL_GetTick();// начинаем отсчет 
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // сигнал на запуск солениоида 			
			sprintf(bufUsb, USB_STRING_FORMAT, ID_MODULE_TIME, 0.0000);
		  CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
		}
		else 
		{
			time_started=0;
			intervalOnScreen=0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);//off LED
			sprintf(bufUsb, USB_STRING_FORMAT, ID_MODULE_TIME, -1.0000); // пауза 
		  CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
		}		
	}
	
	
	if (HAL_GetTick() - timeStart >= 100) {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);} // отпускаем соленоид через 100мс
	
	if(interruptOnSwitch && time_started)
	{
	  sprintf(bufUsb, USB_STRING_FORMAT, ID_MODULE_TIME, (float) period/1000); // в порт отправляется время с последнего прерывания 
		//CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
					// usb is transmitting one by one here, so we need to check it busy status
			sendTime = HAL_GetTick();				
					while(CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb))!=USBD_OK){ // wait if usb is busy 
						if((HAL_GetTick() - sendTime)>3000) break; // timeout
					}
	}
	
	if (interruptOnSwitch && interruptsCount<10) // на дисплее отображается 10 интервалов от старта 
	{
		
		displayFloat((float)timeInterv[interruptsCount]/1000);
	}
	
	interruptOnSwitch=0;		
	
  clearEXTIs();
	
	/* для просмотра значений интервалов (сохраняются 10 интервалов со старта) */
	if(!time_started)
	{		
		_Bool button_pressed=0;
		signsAllowed=1;
		//пролистывание кнопками 
		if (HAL_GPIO_ReadPin(data1_GPIO_Port, data1_Pin) && intervalOnScreen!=0) { intervalOnScreen--;button_pressed=1;}
		if (HAL_GPIO_ReadPin(data2_GPIO_Port, data2_Pin)){ intervalOnScreen++;button_pressed=1;}
		//
		if(intervalOnScreen>interruptsCount) {intervalOnScreen=interruptsCount;}
		if(button_pressed)
		{
			displayFloat((float) intervalOnScreen/100);  // покажем номер интервала 
			HAL_Delay(300);
		  displayFloat((float)timeInterv[intervalOnScreen]/1000); 
		}
	}
}
	
void lcMeterTask (void)
{
	if (!lcMeterStarted)
	{
		lcMeterStarted=1;
		impulseCount=0;
		HAL_TIM_Base_Start(&htim1); // start 1 sec. timer 
		NVIC_EnableIRQ(EXTI9_5_IRQn);  // enable interrupts from lc generator pin
	}
	if (timerStopped)
	{
		
		// freq == impulse count 
		//calculate L or C
		// L=100uH C=1000pF
		// C= 1 / freq^2*4*(pi^2)*L
		float val=0;
		if (currentModule==ID_MODULE_CAPACITY)
		{
			val = 1*pow(10,9)/(pow(impulseCount,2)*4*9.869604*0.0001); // емкость в нФ
			sprintf(bufUsb, USB_STRING_FORMAT, ID_MODULE_CAPACITY, val); 
		}
		else // if induct.
		{
			val = 1*pow(10,6)/(pow(impulseCount,2)*4*9.869604*1*pow(10,-9)); // in mH
			sprintf(bufUsb, USB_STRING_FORMAT, ID_MODULE_INDUCTANCE, val);
		}
		
		
		if (val>999)    // меняем режим отображения и включаем лампочки мкф или нф (H or mH)
		{
			val/=1000; // convert to uF or H
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
		}
		signsAllowed=0;
		displayFloat(val);		
		CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
		
		lcMeterStarted=0; // запускаем таймер и прерывания заново 
		timerStopped=0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if(htim == &htim1)
		{
			NVIC_DisableIRQ(EXTI9_5_IRQn); // stop interrupts 
			timerStopped=1;
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
	uint8_t const hx711Samples=10;
	unsigned long hx711Sum=0;
	getValueHX711(input,gain); // refresh the gain settings
	for (uint8_t i = 0; i< hx711Samples; i++)
	{
		hx711Sum+= getValueHX711(input,gain);
		HAL_Delay(50);
	}
	hx711Sum /= hx711Samples;
	return hx711Sum;
}



void Hx711Task (void)
{		
	
	
//     uint8_t		input=1;
//		hx711Value[input] = hx711Average(input,128);
//		
//		if (interruptOnSwitch == input) {offset[input] = hx711Value[input];interruptOnSwitch=0;} // установка нуля
//		
//		hx711Value[input] -= offset[input];
//		hx711Value[input]*=((28.28 - 101.12)/(881000 - 917500));
//		interruptOnSwitch=0;
//		enableExtis();
	
//	uint8_t const gainLevel_1N = HX711_GAIN_DEFAULT; // возможно придется подобрать усиление. по умолч. 128 
//	uint8_t const gainLevel_5N = HX711_GAIN_DEFAULT;
//	uint8_t const gainLevel_50N = HX711_GAIN_DEFAULT;
//	uint8_t const gainLevel_pressure = HX711_GAIN_DEFAULT;
//	uint8_t const gainLevel_weight = HX711_GAIN_DEFAULT;
//	float const scale1N =   ((28.28 - 101.12)/(881000 - 917500));
//	float const scale5N =  5/1000000;
//	float const scale50N = 50/800000;
//	float const scalePressure = 200/340000; 
//	float const scaleWeight=1000/300000;

//	enableExtis();
//	
//  rj45_connectors_recognise();
//	for (uint8_t input =0; input<3; input++)
//	{	
////		switch (rj45_connected[input])
////		{
////			case RJ45_1_N:      hx711Value[input] = hx711Average(input,gainLevel_1N)*scale1N;             break;
////			case RJ45_5_N:      hx711Value[input] = hx711Average(input,gainLevel_5N)*scale5N;             break;
////			case RJ45_50_N:     hx711Value[input] = hx711Average(input,gainLevel_50N)*scale50N;           break;
////			case RJ45_PRESSURE: hx711Value[input] = hx711Average(input,gainLevel_pressure)*scalePressure; break;
////			case RJ45_WEIGHT:   hx711Value[input] = hx711Average(input,gainLevel_weight)*scaleWeight;     break;
////			default: hx711Value[input]=0;
////		}
//	
//		input=1;
//		hx711Value[input] = hx711Average(input,128);
//		
//		if (interruptOnSwitch == input+1) {offset[input] = hx711Value[input];interruptOnSwitch=0;} // установка нуля
//		
//		hx711Value[input] -= offset[input];
//		
//		
//		if(rj45_connected[input]!=0)
//		{
//		  //для входа 1 номер 108, для входа 2 - 208 и тд
//			sprintf(bufUsb, USB_STRING_FORMAT,(int)(currentModule + (input+1)*100), hx711Value[input]);
//		  
//			// usb is transmitting one by one here, so we need to check it busy status
//			sendTime = HAL_GetTick();				
////					while(CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb))!=USBD_OK){ // wait if usb is busy 
////						if((HAL_GetTick() - sendTime)>3000) break; // timeout
////					}
//		}
//	}
//	
//	if (interruptOnSwitch == 4) {valueOnScreen++;interruptOnSwitch=0;}
//	if(valueOnScreen>2) {valueOnScreen=0;}
//	signsAllowed=1;
//	displayFloat(hx711Value[valueOnScreen]);
//	
//	// show which value is on the display
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
//	switch (valueOnScreen)
//  {
//  	case 0:HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
//  		break;
//  	case 1:HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
//  		break;
//  	default:HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
//  		break;
//  }
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

float getResistance(void)
{
	float R =0;
	float Rx=0;
	for(uint8_t i =0; i<4; i++)
	{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);//100k
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);//10k
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);//1k
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);//100R

		switch(i)
		{
			case 0: HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET); R = 100;  break;
			case 1: HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET); R = 10;   break;
			case 2: HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET); R = 1;    break;
			default: HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET); R = 0.1; break;
		}
		averageAdc_for_N_msec(500);
		if(adc[0]<400 && i<3) continue;
		
		Rx = (R*getVoltage(adc[0])/(3.3-getVoltage(adc[0])));  // kOhm
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);//LED1
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);//LED2
		if (Rx<1.0) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);  // kOhm
		else HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET); // Ohm
		Rx*=1000; // ohm
		
		signsAllowed=0;
		break;
	}
	return Rx;
}

float getVoltageCurrent(void)
{	
	float R=0;
	uint8_t type=0;
	float inVoltage = 0;
	float X=0; // коэфф. делителя 
	
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
	
	inVoltage = getVoltage(adc[0])/33;
	if((type==DC_CURRENT||type==DC_CURRENT)&& adc[0]< adc[1]) {inVoltage = getVoltage(adc[1])/-33;} // если отрицательное напряжение 
	if(X>0) {inVoltage*=X;}
	signsAllowed=1;
	switch(type)
	{	
		case AC_CURRENT: inVoltage= (inVoltage/R)*1.57; break;
		case AC_VOLTAGE: inVoltage= inVoltage*1.57; break;		
		case DC_CURRENT: inVoltage= inVoltage/R; break;
		default:         inVoltage= inVoltage; break;
	}
	return inVoltage;
}

float getNitratSensor(uint16_t adc)
{
	/* необходимо приготовить два раствора с известными концентрациями и измерить напряжение на выходе модуля  
    затем посчитайте коэффициент наклона прямой зависимости K=((С2 -C1)/(V2-V1)) 
	   где C - концентрация */
	float K= 0.1234;
	signsAllowed=0;
	return getVoltage(adc)*K;
}

void spirograph (void)
{
	//// ////////////
	////////////
	//
	////
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
	
//if ((&hUsbDeviceFS->dev_state != USBD_STATE_CONFIGURED)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//		Hx711Task(); 
		
		averageAdc_for_N_msec(1000);
		
////////////////////////////////////////////////////////////////DELETE//////////////////////////////////////
//		if(interruptOnSwitch)
//		currentModule = 0;	
//		if(interruptOnSwitch == 4)
//		currentModule = ID_MODULE_VOLTAGE_30V_AC;
//	
//		interruptOnSwitch=0;
//		enableExtis();
///////////////////////////////////////////////////////////////DELETE///////////////////////////////////////////
//		

//	currentModule = getModuleId();		
		
//		if (currentModule!=currentModule_prev) 
//			{
//				
//				init_needed=1;
//				if(currentModule!=0) 
//				{
//					sprintf(bufUsbStatus, "#%d#con#\n", currentModule); 					
//				}
//				else {sprintf(bufUsbStatus, "#%d#dis#\n", currentModule_prev);  }		
//				
//				currentModule_prev = currentModule;
//			}
//		else {init_needed=0;}
//		
//		if((HAL_GetTick() - sendStatusTime)>3000) // send which module is connected every 3s
//				{
//					sendStatusTime = HAL_GetTick();				
//				
//					while(CDC_Transmit_FS((uint8_t*)bufUsbStatus,strlen(bufUsbStatus))==USBD_BUSY){ // wait if usb is busy 
//						if((HAL_GetTick() - sendStatusTime)>3000) break; // timeout
//					}
//				}
		

//		result = -255; // some unreachable value 
//		switch(currentModule)
//		{	
//			case ID_MODULE_ATM_PRESSURE: 
//				if(init_needed) {read_calliberation_data();}  
//				result = BMP180_GetPress();
//				HAL_Delay(2000);
//				break;
//			case ID_MODULE_HUMIDITY: 
//				result=getAHT20();  
//				HAL_Delay(2000);			
//				break;
//			case ID_MODULE_TIME: 
//				timerTask(); 
//				break;
//			case ID_MODULE_DIF_PRESSURE: 
//				Hx711Task(); 
//				break;
//			case ID_MODULE_INDUCTANCE: 
//				lcMeterTask();  
//				break;
//			case ID_MODULE_RADIATION: 
//				result = getRadSens();
//				break;
//			case ID_MODULE_WEIGHT: 
//				Hx711Task(); 
//				break;
//			case ID_MODULE_LIGHT: 
//				if(init_needed) luxSensInit();
//				result = getluxSens();	
//				break;
//			case ID_MODULE_FORCE: 
//				Hx711Task();  
//				break;
//			case ID_MODULE_TEMP: 
//				if(init_needed) ds18b20_init(SKIP_ROM);
//				result = getDS18B20();
//				break;
//			case ID_MODULE_RESISTANCE: 
//				result = getResistance();  
//				break;
//			case ID_MODULE_CAPACITY: 
//				lcMeterTask();  
//				break;
//			case ID_MODULE_CURRENT_2MA: 
//				averageAdc_for_N_msec(500);
//				result = getVoltageCurrent();
//				break;
//			case ID_MODULE_CURRENT_200MA:
//				averageAdc_for_N_msec(500);
//				result = getVoltageCurrent();
//				break;
//			case ID_MODULE_CURRENT_10A:
//								averageAdc_for_N_msec(500);
//				result = getVoltageCurrent();
//				break;
//			case ID_MODULE_CURRENT_1A_AC:
//								averageAdc_for_N_msec(500);
//				result = getVoltageCurrent();
//				break;
//			case ID_MODULE_VOLTAGE_200MV:
//								averageAdc_for_N_msec(500);
//				result = getVoltageCurrent();
//				break;
//			case ID_MODULE_VOLTAGE_30V: 
//				averageAdc_for_N_msec(500);
//				result = getVoltageCurrent();
//				break;
//			case ID_MODULE_VOLTAGE_30V_AC:
//				averageAdc_for_N_msec(500);
//				result = getVoltageCurrent();
//				break;
//			case ID_MODULE_SPIROMETER: 
//				spirograph();  
//				break;
//			case ID_MODULE_OXYGEN:
//				averageAdc_for_N_msec(2500);
//				result = getOxygenPercent(adc[0]);
//				break;
//			case ID_MODULE_NITRATES:
//				averageAdc_for_N_msec(1500);
//				result = getNitratSensor(adc[0]);
//				break;
//			case ID_MODULE_CO_GAS: 
//				averageAdc_for_N_msec(2500);
//				result = getCOppm(adc[0]);
//				break;
//			case ID_MODULE_TEMP_FAST:
//				HAL_ADC_Start(&hadc1);
//				HAL_ADC_PollForConversion(&hadc1, 100);
//				adc[0] = HAL_ADC_GetValue(&hadc1);
//				HAL_ADC_Stop(&hadc1);
//				result = getTempNTC(adc[0]);
//				break;
//			case ID_MODULE_ULTRAV:
//				averageAdc_for_N_msec(1500);
//				result = getUVIndex(adc[0]);
//				break;
//			default:  waiting_animation();  break;
//		}
//		
//		if(result!=-255) 
//		{
//			displayFloat(result);
//			sprintf(bufUsb, USB_STRING_FORMAT, currentModule, result); 
//		//	CDC_Transmit_FS((uint8_t*)bufUsb,strlen(bufUsb));
//		}
		
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

