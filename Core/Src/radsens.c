#include "radsens.h"
#include "i2c.h"


 
float getRadSens (void)
 {
	 uint8_t data[RS_REG_COUNT] = {0};
	 uint8_t regAddr =0;
	 
	 for (uint8_t i = 0; i< RS_REG_COUNT; i++)
	 {
			HAL_I2C_Mem_Read(&hi2c1, RS_DEFAULT_I2C_ADDRESS, regAddr, I2C_MEMADD_SIZE_8BIT, &data[i], 1, 1000);
		  regAddr++;
	 }
	 
	 //float rad = (((uint32_t)data[3] << 16) | ((uint16_t)data[4] << 8) | data[5]) / 10.0;
	 float rad = (((uint32_t)data[6] << 16) | ((uint16_t)data[7] << 8) | data[8]) / 10.0;
        return rad;
 }
