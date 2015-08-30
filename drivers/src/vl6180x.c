/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file eeprom.c
 * Driver for the 24AA64F eeprom.
 *
 */
#define DEBUG_MODULE "vl6180x"

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "console.h"
#include "config.h"
#include "param.h"

#include "vl6180x.h"
#include "debug.h"
#include "eprintf.h"
#include "i2cdev.h"
#include "stdint.h"
#include "string.h"
#include "log.h"

#define VL6180X_I2C_ADDR     0x29
static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit;

bool vl6180xInit(I2C_Dev *i2cPort)//
{
	if (isInit)
	return true;

	I2Cx = i2cPort;
	devAddr = VL6180X_I2C_ADDR;

	uint8_t data;
	bool status;
	status = i2cdevRead16(I2Cx, devAddr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 2, &data);
	if (status == 1) //0x01
	{
	DEBUG_PRINT("vl6180x Initialized.\n");

	i2cdevWrite16(I2Cx, devAddr, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 1, 0);
	}

  isInit = true;

  return true;
}

bool vl6180xTest(void)
{
  bool status;

  status = vl6180xTestConnection();
  if (status)
  {
    DEBUG_PRINT("vl6180x connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("vl6180x connection [FAIL].\n");
  }

  return status;
}

bool vl6180xTestConnection(void)
{
  uint8_t tmp;
  bool status;

  if (!isInit)
    return false;

  status = i2cdevRead16(I2Cx, devAddr, 0, 1, &tmp);

  return status;
}

//bool vl6180xReadBuffer(uint8_t* buffer, uint16_t readAddr, uint16_t len)
//{
//  bool status;
//
//  if ((uint32_t)readAddr + len > VL6180X_SIZE)
//  {
//     return false;
//  }
//
//  status = i2cdevRead16(I2Cx, devAddr, readAddr, len, buffer);
//
//  return status;
//}


uint8_t VL6180x_getRegister(uint16_t registerAddr)
{
  uint8_t data;
//  bool status;
  // Read adata byte from a 16bit internal register of the VL6180
  i2cdevRead16(I2Cx, devAddr, registerAddr, 2, &data); //I2Cx and devAddr set during init.

  return data;
}

uint16_t VL6180x_getRegister16bit(uint16_t registerAddr)
{
  uint8_t data;
  // Read two data byte from a 16bit internal register of the VL6180
  i2cdevRead16(I2Cx, devAddr, registerAddr, 2, &data); //I2Cx and devAddr set during init.

  return data;
}

void VL6180x_setRegister(uint16_t registerAddr, uint8_t data)
{
  // Write a data byte to a 16bit internal register of the VL6180
	i2cdevWrite16(I2Cx, devAddr, registerAddr, 1, &data);  //I2Cx and devAddr set during init.
}

void VL6180x_setRegister16bit(uint16_t registerAddr, uint8_t data)
{
  // Write two data bytes to a 16bit internal register of the VL6180
  i2cdevWrite16(I2Cx, devAddr, registerAddr, 2, &data); // I2Cx, devAddr, registerAddr, 2, &data I2Cx and devAddr set during init.
}


//bool vl6180xWriteBuffer(uint8_t* buffer, uint16_t writeAddr, uint16_t len)
//{
//  bool status = true;
//  uint16_t index;
//
//  if ((uint32_t)writeAddr + len > VL6180X_SIZE)
//  {
//     return false;
//  }
//
//  for (index = 0; index < len && status; index++)
//  {
//    status = i2cdevWrite16(I2Cx, devAddr, writeAddr + index, 1, &buffer[index]);
//    vTaskDelay(M2T(6));
//  }
//
//  return status;
//}
//
//bool vl6180xWritePage(uint8_t* buffer, uint16_t writeAddr)
//{
//
//  return false;
//}


uint8_t vl6180xgetDistance()
{
	bool status;
//	uint8_t data;
////	DEBUG_PRINT ("VL6180X Distance. /n");
//
//
//while (isInit == true)
//{
VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x03); //Start continuous mode -single shot = 0x01
 //i2cdevWrite16(I2Cx, devAddr, VL6180X_SYSRANGE_START, 1, 0x03);
 vTaskDelay(M2T(500));
//
// if (VL6180X_RESULT_INTERRUPT_STATUS_GPIO == 1)
// {
  status = VL6180x_getRegister(VL6180X_RESULT_RANGE_VAL);
  //status = i2cdevRead16(I2Cx, devAddr, VL6180X_RESULT_RANGE_VAL, 2, &data);

  return status;

  //VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
 	 }
//   }
//}

void VL6180xDefaultSettings(void){
  //Recommended settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
	  VL6180x_setRegister(0x0207, 0x01);
	  VL6180x_setRegister(0x0208, 0x01);
	  VL6180x_setRegister(0x0096, 0x00);
	  VL6180x_setRegister(0x0097, 0xfd);
	  VL6180x_setRegister(0x00e3, 0x00);
	  VL6180x_setRegister(0x00e4, 0x04);
	  VL6180x_setRegister(0x00e5, 0x02);
	  VL6180x_setRegister(0x00e6, 0x01);
	  VL6180x_setRegister(0x00e7, 0x03);
	  VL6180x_setRegister(0x00f5, 0x02);
	  VL6180x_setRegister(0x00d9, 0x05);
	  VL6180x_setRegister(0x00db, 0xce);
	  VL6180x_setRegister(0x00dc, 0x03);
	  VL6180x_setRegister(0x00dd, 0xf8);
	  VL6180x_setRegister(0x009f, 0x00);
	  VL6180x_setRegister(0x00a3, 0x3c);
	  VL6180x_setRegister(0x00b7, 0x00);
	  VL6180x_setRegister(0x00bb, 0x3c);
	  VL6180x_setRegister(0x00b2, 0x09);
	  VL6180x_setRegister(0x00ca, 0x09);
	  VL6180x_setRegister(0x0198, 0x01);
	  VL6180x_setRegister(0x01b0, 0x17);
	  VL6180x_setRegister(0x01ad, 0x00);
	  VL6180x_setRegister(0x00ff, 0x05);
	  VL6180x_setRegister(0x0100, 0x05);
	  VL6180x_setRegister(0x0199, 0x05);
	  VL6180x_setRegister(0x01a6, 0x1b);
	  VL6180x_setRegister(0x01ac, 0x3e);
	  VL6180x_setRegister(0x01a7, 0x1f);
	  VL6180x_setRegister(0x0030, 0x00);
	  VL6180x_setRegister(0x0016, 0x00);
  //Enable Interrupts on Conversion Complete (any source)

//  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, (4 << 3)|(4) ); // Set GPIO1 high when sample complete
//
//
//  VL6180x_setRegister(VL6180X_SYSTEM_MODE_GPIO1, 0x10); // Set GPIO1 high when sample complete
//  VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30); //Set Avg sample period
//  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, 0x46); // Set the ALS gain
//  VL6180x_setRegister(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF); // Set auto calibration period (Max = 255)/(OFF = 0)
//  VL6180x_setRegister(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x63); // Set ALS integration time to 100ms
//  VL6180x_setRegister(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01); // perform a single temperature calibration
  //Optional settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
//  VL6180x_setRegister(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09); // Set default ranging inter-measurement period to 100ms
//  VL6180x_setRegister(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A); //default 0x31 500ms Set default ALS inter-measurement period to 100ms
//  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’
  //Additional settings defaults from community
//  VL6180x_setRegister(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);
//  VL6180x_setRegister(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);
//  VL6180x_setRegister16bit(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B );
//  VL6180x_setRegister16bit(VL6180X_SYSALS_INTEGRATION_PERIOD, 0x64);
//
//  VL6180x_setRegister(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD,0x30);
//  VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN,0x40);
//  VL6180x_setRegister(VL6180X_FIRMWARE_RESULT_SCALER,0x01);
}

//void getAmbientLight()
//{
  //First load in Gain we are using, do it everytime incase someone changes it on us.
  //Note: Upper nibble shoudl be set to 0x4 i.e. for ALS gain of 1.0 write 0x46
  //VL6180x_setRegister(VL6180X_SYSALS_ANALOGUE_GAIN, (0x40 | 20)); // Set the ALS gain

  //Start ALS Measurement
//  VL6180x_setRegister(VL6180X_SYSRANGE_START, 0x03);
//
//  vTaskDelay(M2T(500)); //give it time...
//
//  unsigned int alsIntegrationPeriodRaw = VL6180x_getRegister16bit(VL6180X_RESULT_RANGE_VAL);
//
//  VL6180x_setRegister(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  //Retrieve the Raw ALS value from the sensor
  //unsigned int alsRaw = VL6180x_getRegister16bit(VL6180X_RESULT_ALS_VAL);

  //Get Integration Period for calculation, we do this everytime incase someone changes it on us.
  //unsigned int alsIntegrationPeriodRaw = VL6180x_getRegister16bit(VL6180X_RESULT_RANGE_VAL);

  //float alsIntegrationPeriod = 100.0 / alsIntegrationPeriodRaw ;

  //Calculate actual LUX from Appnotes

  //float alsGain = 20.0;  //0.0

//  switch (VL6180X_ALS_GAIN){
//    case GAIN_20: alsGain = 20.0; break;
//    case GAIN_10: alsGain = 10.32; break;
//    case GAIN_5: alsGain = 5.21; break;
//    case GAIN_2_5: alsGain = 2.60; break;
//    case GAIN_1_67: alsGain = 1.72; break;
//    case GAIN_1_25: alsGain = 1.28; break;
//    case GAIN_1: alsGain = 1.01; break;
//    case GAIN_40: alsGain = 40.0; break;
//  }

//Calculate LUX from formula in AppNotes

 // float alsCalculated = (float)0.32 * ((float)alsRaw / alsGain) * alsIntegrationPeriod;

//  return alsIntegrationPeriodRaw;
//}

//void getIdentification(struct VL6180xIdentification *temp){
//  temp->idModel =  VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_ID);
//  temp->idModelRevMajor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_REV_MAJOR);
//  temp->idModelRevMinor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODEL_REV_MINOR);
//  temp->idModuleRevMajor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODULE_REV_MAJOR);
//  temp->idModuleRevMinor = VL6180x_getRegister(VL6180X_IDENTIFICATION_MODULE_REV_MINOR);
//
//  temp->idDate = VL6180x_getRegister16bit(VL6180X_IDENTIFICATION_DATE);
//  temp->idTime = VL6180x_getRegister16bit(VL6180X_IDENTIFICATION_TIME);
//}

//uint8_t changeAddress(uint8_t old_address, uint8_t new_address){
//  //NOTICE:  IT APPEARS THAT CHANGING THE ADDRESS IS NOT STORED IN NON-VOLATILE MEMORY
//  // POWER CYCLING THE DEVICE REVERTS ADDRESS BACK TO 0X29
//
//  if( old_address == new_address) return old_address;
//  if( new_address > 127) return old_address;
//
//   VL6180x_setRegister(VL6180X_I2C_SLAVE_DEVICE_ADDRESS, new_address);
//
//   return VL6180x_getRegister(VL6180X_I2C_SLAVE_DEVICE_ADDRESS);
//}
