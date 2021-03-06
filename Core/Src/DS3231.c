/******************************************************************************************************************
@File:  	DS3231 RTC Module
@Author:  Khue Nguyen
@Website: khuenguyencreator.com
@Youtube: https://www.youtube.com/channel/UCt8cFnPOaHrQXWmVkk-lfvg
Huong dan su dung:
- Su dung thu vien HAL
- Khoi tao I2C cho Ds3231
- Khoi tao bien luu DS3231
DS3231_Name DS3231;
- Khoi tao DS3231 do:
DS3231_Init(&DS3231, &hi2c1);
- Su dung cac ham phai truyen dia chi cua DS3231 do: 
DS3231_GetTime(&DS3231);
DS3231_GetDate(&DS3231);
******************************************************************************************************************/

#include "DS3231.h"
//*****************************************Low Level Layer ***********************************************//
// Thay doi thanh IT hoac DMA neu can thiet

static void I2C_WriteTime(DS3231_Name* DS3231)
{
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, DS3231->TxTimeBuff, 3, 1000);
}

static void I2C_ReadTime(DS3231_Name* DS3231)
{
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, DS3231->RxTimeBuff, 3, 1000);
}

static void I2C_WriteDate(DS3231_Name* DS3231)
{
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 3, I2C_MEMADD_SIZE_8BIT, DS3231->TxDateBuff, 4, 1000);
}

static void I2C_ReadDate(DS3231_Name* DS3231)
{
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 3, I2C_MEMADD_SIZE_8BIT, DS3231->RxDateBuff, 4, 1000);
}

static void I2C_ReadTemp(DS3231_Name* DS3231)
{
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0x11, I2C_MEMADD_SIZE_8BIT, DS3231->TempBuff, 2, 1000);
}

static void I2C_WriteAlarmTime(DS3231_Name* DS3231)
{
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 7, I2C_MEMADD_SIZE_8BIT, DS3231->TxTimeBuff, 3, 1000);
}

static void I2C_WriteAlarmDate(DS3231_Name* DS3231)
{
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 9, I2C_MEMADD_SIZE_8BIT, DS3231->TxDateBuff, 2, 1000);
}

void force_temp_conv (DS3231_Name* DS3231)
{
	uint8_t status=0;
	uint8_t control=0;
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0x0F, 1, &status, 1, 100);  // read status register
	if (!(status&0x04))  // if the BSY bit is not set
	{
		HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0x0E, 1, &control, 1, 100);  // read control register
		HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 0x0E, 1, (uint8_t *)(control|(0x20)), 1, 100);  // write modified control register with CONV bit as'1'
	}
}
static uint8_t BCD2DEC(uint8_t data)
{
	return (data>>4)*10+ (data&0x0f);
}

static uint8_t DEC2BCD(uint8_t data)
{
	return (data/10)<<4|(data%10);
}


//****************************************HIGH Level Layer *************************************************//
void DS3231_SetInterruptMode(DS3231_Name* DS3231, DS3231_State enable)
{
	uint8_t control = 0;
	uint8_t val = 0x04;
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0x0E, 1, &control, 1, 100);  // read control register
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 0x0E, 1, &val, 1, 100);
}
void DS3231_EnableAlarm1(DS3231_Name* DS3231, DS3231_State enable)
{
	uint8_t control = 0;
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0x0E, 1, &control, 1, 100);  // read control register
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 0x0E, 1, (uint8_t *)((control & (0xfe))|(enable & 0x01)), 1, 100);
}
void DS3231_ClearAlarm1Flag(DS3231_Name* DS3231)
{
	uint8_t status = 0;
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0x0F, 1, &status, 1, 100);  // read control register
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 0x0F, 1, (uint8_t *)(status & (0xfe)), 1, 100);
}
void DS3231_EnableAlarm2(DS3231_Name* DS3231, DS3231_State enable)
{
	uint8_t control = 0;
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0x0E, 1, &control, 1, 100);  // read control register
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 0x0E, 1, (uint8_t *)((control & (0xfd))|((enable & 0x01)<<1)), 1, 100);
}
void DS3231_ClearAlarm2Flag(DS3231_Name* DS3231)
{
	uint8_t status = 0;
	HAL_I2C_Mem_Read(DS3231->I2C, DS3231_ADDRESS, 0x0F, 1, &status, 1, 100);  // read control register
	HAL_I2C_Mem_Write(DS3231->I2C, DS3231_ADDRESS, 0x0F, 1, (uint8_t *)(status & (0xfd)), 1, 100);
}
void DS3231_Init(DS3231_Name* DS3231, I2C_HandleTypeDef* I2C_In)
{
	DS3231->I2C = I2C_In;
	DS3231_EnableAlarm1(DS3231, DS3231_Disable);
	DS3231_EnableAlarm2(DS3231, DS3231_Disable);
	DS3231_ClearAlarm1Flag(DS3231);
	DS3231_ClearAlarm2Flag(DS3231);
	DS3231_SetInterruptMode(DS3231, DS3231_Enable);
}

void DS3231_SetTime(DS3231_Name* DS3231, uint8_t Hour, uint8_t Min, uint8_t Sec)
{
	DS3231->TxTimeBuff[0] = DEC2BCD(Sec);
	DS3231->TxTimeBuff[1] = DEC2BCD(Min);
	DS3231->TxTimeBuff[2] = DEC2BCD(Hour);
	I2C_WriteTime(DS3231);
}	
void DS3231_GetTime(DS3231_Name* DS3231)
{
	I2C_ReadTime(DS3231);
	DS3231->Sec = BCD2DEC(DS3231->RxTimeBuff[0]);
	DS3231->Min = BCD2DEC(DS3231->RxTimeBuff[1]);
	DS3231->Hours = BCD2DEC(DS3231->RxTimeBuff[2]);
	
}
void DS3231_SetDate(DS3231_Name* DS3231, uint8_t Day, uint8_t Date, uint8_t Month, uint8_t Year)
{
	DS3231->TxDateBuff[0] = DEC2BCD(Day);
	DS3231->TxDateBuff[1] = DEC2BCD(Date);
	DS3231->TxDateBuff[2] = DEC2BCD(Month);
	DS3231->TxDateBuff[3] = DEC2BCD(Year);
	I2C_WriteDate(DS3231);
}	
void DS3231_GetDate(DS3231_Name* DS3231)
{
	I2C_ReadDate(DS3231);
	DS3231->Day = BCD2DEC(DS3231->RxDateBuff[0]);
	DS3231->Date = BCD2DEC(DS3231->RxDateBuff[1]);
	DS3231->Month = BCD2DEC(DS3231->RxDateBuff[2]);
	DS3231->Year = BCD2DEC(DS3231->RxDateBuff[3]);
}
float DS3231_GetTemp(DS3231_Name* DS3231)
{
	force_temp_conv(DS3231);
	I2C_ReadTemp(DS3231);
	return((DS3231->TempBuff[0])+(DS3231->TempBuff[1]>>6)/4.0);
}
void DS3231_SetAlarmTime(DS3231_Name* DS3231, uint8_t Hour, uint8_t Min, uint8_t Sec)
{
	DS3231->TxTimeBuff[0] = DEC2BCD(Sec);
	DS3231->TxTimeBuff[1] = DEC2BCD(Min);
	DS3231->TxTimeBuff[2] = DEC2BCD(Hour);
	I2C_WriteAlarmTime(DS3231);
}
void DS3231_SetAlarmDate(DS3231_Name* DS3231, uint8_t Day, uint8_t Date)
{
	DS3231->TxDateBuff[0] = DEC2BCD(Day);
	DS3231->TxDateBuff[1] = DEC2BCD(Date);
	I2C_WriteAlarmDate(DS3231);
}
