/*
 * MAX31865.c
 *
 *  Created on: 12.06.2016
 *      Author: washed
 */

#include "stm32f4xx_hal.h"
#include <MAX31865.h>
#include <math.h>


uint32_t MAX31865_DEVICES_RTD_DATA[MAX31865_MAX_DEVICES];
volatile uint8_t MAX31865_DEVICES_SAMPLE_READY[MAX31865_MAX_DEVICES] = { 0, 0, 0, 0 };
volatile uint32_t MAX31865_DEVICES_TIME_SINCE_LAST_READ[MAX31865_MAX_DEVICES] = { 0, 0, 0, 0 };
int32_t MAX31865_DEVICES_TEMP[MAX31865_MAX_DEVICES];

static inline void assertCS( uint32_t device_num );
static inline void deassertCS( uint32_t device_num );

const uint32_t MAX31865_DEVICES_CS_BANK_PIN[MAX31865_MAX_DEVICES][2] =
{
		{MAX31865_0_CS_BANK, MAX31865_0_CS_PIN},
		{MAX31865_1_CS_BANK, MAX31865_1_CS_PIN},
		{MAX31865_2_CS_BANK, MAX31865_2_CS_PIN},
		{MAX31865_3_CS_BANK, MAX31865_3_CS_PIN}
};

const uint32_t MAX31865_DEVICES_DR_BANK_PIN[MAX31865_MAX_DEVICES][2] =
{
		{MAX31865_0_DR_BANK, MAX31865_0_DR_PIN},
		{MAX31865_1_DR_BANK, MAX31865_1_DR_PIN},
		{MAX31865_2_DR_BANK, MAX31865_2_DR_PIN},
		{MAX31865_3_DR_BANK, MAX31865_3_DR_PIN}
};

void handleMAX31865Devices()
{
	for ( uint32_t device_num = 0; device_num < MAX31865_CON_DEVICES; device_num++ )
	{
		if ( MAX31865_DEVICES_SAMPLE_READY[device_num] )
		{
			MAX31865_DEVICES_TIME_SINCE_LAST_READ[device_num] = 0;
			getRTDData_MAX31865(device_num);
			//TODO: Do any necessary post processing steps here
			if ( MAX31865_USE_CALLENDAR_VANDUSEN )
			{

			}
			else
			{
				MAX31865_DEVICES_TEMP[device_num] = lrintf((float)(((float)MAX31865_DEVICES_RTD_DATA[device_num] / 32.0) - 256.0)*(float)TEMP_INT_FACTOR);

			}
			/*
			switch ( device_num )
			{
			case 0:
				addTemperatureSample( &temp_control0, MAX31865_DEVICES_TEMP[0] );
				break;
			}
			*/
		}
	}
}

void checkMAX31865WDG()
{
	for ( uint32_t device_num = 0; device_num < MAX31865_CON_DEVICES; device_num++ )
	{
		if ( MAX31865_DEVICES_TIME_SINCE_LAST_READ[device_num] >= MAX31865_WDG_PERIOD )
			MAX31865_DEVICES_SAMPLE_READY[device_num] = 1;
	}
}

void getRTDData_MAX31865( uint32_t device_num )
{
	const uint8_t tx_data[3] = { MAX31865_RTDMSB_REG_RD_ADDR, 0xFF, 0xFF };
	uint8_t rx_data[3];

	assertCS( device_num );
	HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 3, 100 );
	deassertCS( device_num );
	MAX31865_DEVICES_SAMPLE_READY[device_num] = 0;
	MAX31865_DEVICES_RTD_DATA[device_num] = rx_data[1] << 7;
	MAX31865_DEVICES_RTD_DATA[device_num] |= rx_data[2] >> 1;
}
void setCfgReg_MAX31865( uint32_t device_num, uint8_t config_flags )
{
	uint8_t tx_data[2] =	{ MAX31865_CFG_REG_WR_ADDR,	config_flags };
	uint8_t rx_data[2];
	assertCS( device_num );
	HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 2, 50 );
	deassertCS( device_num );
}

void initSPIIdleClock()
{
	uint8_t tx_data[2] = { 0xFF, 0xFF };
	uint8_t rx_data[2];
	assertCS( 0 );
	HAL_SPI_TransmitReceive( MAX31865_SPI_INSTANCE_PT, (uint8_t*)&tx_data, (uint8_t*)&rx_data, 2, 10 );
	deassertCS( 0 );
}

static inline void assertCS( uint32_t device_num )
{
	((GPIO_TypeDef *)(MAX31865_DEVICES_CS_BANK_PIN[device_num][0]))->BSRR = (MAX31865_DEVICES_CS_BANK_PIN[device_num][1] << 16UL);
}

static inline void deassertCS( uint32_t device_num )
{
	((GPIO_TypeDef *)(MAX31865_DEVICES_CS_BANK_PIN[device_num][0]))->BSRR = MAX31865_DEVICES_CS_BANK_PIN[device_num][1];
}

// DR Pin callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if ( GPIO_Pin == MAX31865_0_DR_PIN )
	{
		MAX31865_DEVICES_SAMPLE_READY[0] = 1;
	}
	if ( GPIO_Pin == MAX31865_1_DR_PIN )
	{
		MAX31865_DEVICES_SAMPLE_READY[1] = 1;
	}
	if ( GPIO_Pin == MAX31865_2_DR_PIN )
	{
		MAX31865_DEVICES_SAMPLE_READY[2] = 1;
	}
	if ( GPIO_Pin == MAX31865_3_DR_PIN )
	{
		MAX31865_DEVICES_SAMPLE_READY[3] = 1;
	}
}

void tickMAX31865WDGTimer( uint32_t ticks )
{
	for ( uint32_t device_num = 0; device_num < MAX31865_CON_DEVICES; device_num++ )
	{
		MAX31865_DEVICES_TIME_SINCE_LAST_READ[device_num] += ticks;
	}
}
