/**
 * MS5637-02BA03 Drivers for the RP2040
 * Author: York Aerospace and Rocketry
*/
#ifndef MS5637_H
#define MS5637_H

/**
 * Including relevant libraries
*/
#include <stdint.h>
#include <stdio.h>

/**
 * MS5637-02BA03 i2c address
*/
#define MS5637_ADDRESS  0x76 

#define RESET 0x1E
#define MS5637_START_PRESSURE_ADC_CONVERSION 0x40
#define MS5637_START_TEMPERATURE_ADC_CONVERSION 0x50
#define MS5637_CONVERSION_OSR_MASK 0x0F
#define ADC_READ 0x00

#define MS5637_PROM_ADDR_0 0xA0
#define MS5637_PROM_ADDR_1 0xA2
#define MS5637_PROM_ADDR_2 0xA4
#define MS5637_PROM_ADDR_3 0xA6
#define MS5637_PROM_ADDR_4 0xA8
#define MS5637_PROM_ADDR_5 0xAA
#define MS5637_PROM_ADDR_6 0xAC

#define MS5637_CONVERSION_TIME_OSR_256 1
#define MS5637_CONVERSION_TIME_OSR_512 2
#define MS5637_CONVERSION_TIME_OSR_1024 3
#define MS5637_CONVERSION_TIME_OSR_2048 5
#define MS5637_CONVERSION_TIME_OSR_4096 9
#define MS5637_CONVERSION_TIME_OSR_8192 17
	 
#define RESOLUTION_OSR_256 0
#define RESOLUTION_OSR_512 1 
#define RESOLUTION_OSR_1024 2
#define RESOLUTION_OSR_2048 3
#define RESOLUTION_OSR_4096 4
#define RESOLUTION_OSR_8192 5
	
#define MS5637_CRC_INDEX 0
#define MS5637_PRESSURE_SENSITIVITY_INDEX 1 
#define MS5637_PRESSURE_OFFSET_INDEX 2
#define MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX 3
#define MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX 4
#define MS5637_REFERENCE_TEMPERATURE_INDEX 5
#define MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX 6

/**
 * MS5637 STATUS ENUM
*/
typedef enum {
    MS5637_INIT_FAIL,
    MS5637_INIT_SUCCESS
} MS5637_reset_status;

/**
 * FUNCTION POINTERS
*/
typedef uint8_t (*read_Data)(void*, const uint8_t, const uint8_t, uint8_t*, const uint8_t);
typedef uint8_t (*write_Command)(void*, const uint8_t, const uint8_t);

/**
 * SENSOR STRUCT
*/
typedef struct ms5637{

    /* i2c port */
    void *i2c;

    /* Temperature (in degrees C) and Pressure Data (in mili bars) */
    float pressure;

    float temperature;

    read_Data read_MS5637_Data;

    write_Command write_MS5637_Command;

} MS5637;

/**
 * INITIALISATION
 */
MS5637_reset_status MS5637_Initialise(MS5637 *dev, void *i2c);

/**
 * DATA READING
 * Data reading depends on the Over Sampling Rate
 * Usage:
 *      Resolution 0 --> OSR=256 
 *      Resolution 1 --> OSR=512 
 *      Resolution 2 --> OSR=1024 
 *      Resolution 3 --> OSR=2048
 *      Resolution 4 --> OSR=4096
 *      Resolution 5 --> OSR=8192
 *      
 */
void MS5637_ReadTemperature_and_Pressure(MS5637 *dev, uint8_t resolution);

/**
 * LOW LEVEL FUNCTIONS
 */
uint8_t MS5637_ReadRegisters(void* i2c, const uint8_t addr, const uint8_t reg, uint8_t *data_buff, const uint8_t num_bytes);
uint8_t MS5637_WriteCommand(void* i2c, const uint8_t addr, const uint8_t command);

#endif

