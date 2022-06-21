#include "bmp180.h"
#include "i2c.h"
#include "math.h"

#define BMP180_ADDRESS 0xEE<<1

int16_t AC1;
int16_t AC2;
int16_t AC3;
uint16_t AC4;
uint16_t AC5;
uint16_t AC6;
int16_t B1;
int16_t B2;
int16_t MB;
int16_t MC;
int16_t MD;

long UP,X1,X2,B5,B6,X3,B3,UT;
unsigned long B4,B7;
float Press;


void read_calliberation_data (void)
{
	uint8_t Callib_Data[22] = {0};
	uint16_t Callib_Start = 0xAA;
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, Callib_Start, 1, Callib_Data,22, HAL_MAX_DELAY);

	AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = ((Callib_Data[20] << 8) | Callib_Data[21]);
}

uint32_t Get_UPress (int oss)   // oversampling setting 0,1,2,3
{
	uint8_t datatowrite = 0x34+(oss<<6);
	uint8_t Press_RAW[3] = {0};
	HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	switch (oss)
	{
		case (0):
			HAL_Delay (5);
			break;
		case (1):
			HAL_Delay (8);
			break;
		case (2):
			HAL_Delay (14);
			break;
		case (3):
			HAL_Delay (26);
			break;
	}
	HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, 0xF6, 1, Press_RAW, 3, 1000);
	return (((Press_RAW[0]<<16)+(Press_RAW[1]<<8)+Press_RAW[2]) >> (8-oss));
}


float BMP180_GetPress (void)
{

	int oss =3;
	UP = Get_UPress(oss);
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	B6 = B5-4000;
	X1 = (B2 * (B6*B6/(pow(2,12))))/(pow(2,11));
	X2 = AC2*B6/(pow(2,11));
	X3 = X1+X2;
	B3 = (((AC1*4+X3)<<oss)+2)/4;
	X1 = AC3*B6/pow(2,13);
	X2 = (B1 * (B6*B6/(pow(2,12))))/(pow(2,16));
	X3 = ((X1+X2)+2)/pow(2,2);
	B4 = AC4*(unsigned long)(X3+32768)/(pow(2,15));
	B7 = ((unsigned long)UP-B3)*(50000>>oss);
	if (B7<0x80000000) Press = (B7*2)/B4;
	else Press = (B7/B4)*2;
	X1 = (Press/(pow(2,8)))*(Press/(pow(2,8)));
	X1 = (X1*3038)/(pow(2,16));
	X2 = (-7357*Press)/(pow(2,16));
	Press = Press + (X1+X2+3791)/(pow(2,4));

	return Press;
}

