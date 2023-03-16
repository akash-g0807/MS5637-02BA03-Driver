#include "MS5637_02BA03.h"
#include <stdio.h>

/**
 * EEPROM Coefficients
*/
uint16_t coefficients[8] = {0,0,0,0,0,0,0,0};

/**
 * I2C Read and Write Commands
*/
uint8_t MS5637_ReadRegisters(void* i2c, const uint8_t addr, const uint8_t reg, uint8_t *data_buff, const uint8_t num_bytes){
    i2c_write_blocking(i2c, addr, &reg, 1, true);  
    uint8_t num_bytes_read = i2c_read_blocking(i2c, addr, data_buff, num_bytes, false);
    return num_bytes_read;
}

uint8_t MS5637_WriteCommand(void* i2c, const uint8_t addr, const uint8_t command){
    
    uint8_t bytes_wrote = i2c_write_blocking(i2c, addr, &command, 1, false);
    return bytes_wrote;

}

MS5637_reset_status MS5637_Initialise(MS5637 *dev, void *i2c){

    dev->i2c = i2c;

    dev->pressure = 0.0f;
    dev->temperature = 0.0f;

    dev-> read_MS5637_Data = MS5637_ReadRegisters;
    dev->write_MS5637_Command = MS5637_WriteCommand;

    /* Reset Command */
    uint8_t reset_command = 0x1E;
    uint8_t wrote = dev->write_MS5637_Command(dev->i2c,MS5637_ADDRESS, RESET);
    
    /* Checking if the write command has failed */
    if(wrote == 1){
        return MS5637_INIT_SUCCESS;;
    }
    return MS5637_INIT_FAIL;

}

/* Reading the coefficients from the EEPROM */
uint32_t read_eeprom_coefficients(MS5637 *dev, uint8_t prom_command){
    uint8_t data[2] = {0,0};
    uint8_t num_bytes_read = dev->read_MS5637_Data(dev->i2c, MS5637_ADDRESS, prom_command, data, 2);
    return (data[0] << 8) | data[1];
}


void read_eeprom(MS5637 *dev){
    int coefficients_index = 0;

    uint8_t prom_addr_list[] = {MS5637_PROM_ADDR_0,
		    MS5637_PROM_ADDR_1,
		    MS5637_PROM_ADDR_2,
		    MS5637_PROM_ADDR_3,
		    MS5637_PROM_ADDR_4,
		    MS5637_PROM_ADDR_5,
		    MS5637_PROM_ADDR_6,};

    for(int i = 0; i < 7;i++)
    {
        coefficients[coefficients_index] = read_eeprom_coefficients(dev, prom_addr_list[i]);
        coefficients_index = coefficients_index + 1;
    }
}

/**
 * READING RAW ADC DATA
 * Based on the calculated OSR conversion time and the the command for the OSR
 */
uint32_t conversion_read_adc(MS5637 *dev, uint8_t command, float waiting_time, uint8_t adc_address){
    uint8_t adc_data[3];
    uint8_t wrote = dev->write_MS5637_Command(dev->i2c, MS5637_ADDRESS, command);
    sleep_ms(waiting_time);

    uint8_t num_bytes_read = dev->read_MS5637_Data(dev->i2c, MS5637_ADDRESS, adc_address, adc_data, 3);
    uint32_t adc_return = adc_data[0] << 16 | adc_data[1] << 8 | adc_data[2];
    return adc_return;

}

void MS5637_ReadTemperature_and_Pressure(MS5637 *dev, uint8_t resolution){
    int64_t OFF, SENS, P, T2, OFF2, SENS2;
    int32_t dT, TEMP;

    /* Getting the temperature command and conversion time based on OSR */
    uint8_t temperature_command;
    uint8_t temperature_waiting_time;
    
    /* Getting the pressure command and conversion time based on OSR */
    uint8_t pressure_command;
    uint8_t pressure_waiting_time;

    temperature_command = resolution * 2;
    temperature_command |= MS5637_START_TEMPERATURE_ADC_CONVERSION;
    uint8_t time[] ={MS5637_CONVERSION_TIME_OSR_256,
		MS5637_CONVERSION_TIME_OSR_512,
		MS5637_CONVERSION_TIME_OSR_1024,
		MS5637_CONVERSION_TIME_OSR_2048,
		MS5637_CONVERSION_TIME_OSR_4096,
		MS5637_CONVERSION_TIME_OSR_8192}; 

    int index_time_temperature = (temperature_command & MS5637_CONVERSION_OSR_MASK)/2;
    temperature_waiting_time = time[index_time_temperature];

    pressure_command = resolution * 2;
    pressure_command |= MS5637_START_PRESSURE_ADC_CONVERSION;

    int index_time_pressure = (pressure_command & MS5637_CONVERSION_OSR_MASK)/2;
    pressure_waiting_time = time[index_time_pressure];

    read_eeprom(dev);

    uint32_t adc_temperature = conversion_read_adc(dev, temperature_command, temperature_waiting_time, ADC_READ);
    uint32_t adc_pressure = conversion_read_adc(dev, pressure_command, pressure_waiting_time, ADC_READ);

    dT = (int32_t)adc_temperature - ((int32_t)coefficients[MS5637_REFERENCE_TEMPERATURE_INDEX] <<8 );

    TEMP = 2000 + ((int64_t)dT * (int64_t)coefficients[MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;

	
	/* Second order temperature compensation */
	if( TEMP < 2000 )
	{
		T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
		OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		
		if( TEMP < -1500 )
		{
			OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
		}
	}
	else
	{
		T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
		OFF2 = 0 ;
		SENS2 = 0 ;
	}
	
	/* OFF = OFF_T1 + TCO * dT */
	OFF = ( (int64_t)(coefficients[MS5637_PRESSURE_OFFSET_INDEX]) << 17 ) + ( ( (int64_t)(coefficients[MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 6 ) ;
	OFF -= OFF2 ;
	
	/* Sensitivity at actual temperature = SENS_T1 + TCS * dT */
	SENS = ( (int64_t)coefficients[MS5637_PRESSURE_SENSITIVITY_INDEX] << 16 ) + ( ((int64_t)coefficients[MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 7 ) ;
	SENS -= SENS2 ;
	
	/* Temperature compensated pressure = D1 * SENS - OFF */
	P = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15 ;
	
	float temperature = ( (float)TEMP - T2 ) / 100;
	float pressure = (float)P / 100;

    /* Updating the temperature and pressure values in the sensor struct */
    dev->pressure = (float)P / 100;
    dev->temperature = ( (float)TEMP - T2 ) / 100;
}