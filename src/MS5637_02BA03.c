#include "MS5637_02BA03.h"
#include <stdio.h>

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

    uint8_t reset_command = 0x1E;
    uint8_t wrote = MS5637_WriteCommand(dev->i2c,MS5637_ADDRESS, RESET);
    
    if(wrote == 1){
        return MS5637_INIT_SUCCESS;;
    }
    return MS5637_INIT_FAIL;

}

uint32_t read_eeprom_coefficients(MS5637 *dev, uint8_t prom_command){
    uint8_t data[2] = {0,0};
    uint8_t num_bytes_read = MS5637_ReadRegisters(dev->i2c, MS5637_ADDRESS, prom_command, data, 2);
    printf("%d\n", num_bytes_read);
    return (data[0] << 8) | data[1];
}


int main(){
    stdio_init_all();
    
 
    const uint8_t sda_pin = 20;
    const uint8_t scl_pin = 21; 

    i2c_init(i2c0, 400 * 1000);

    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
     
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    MS5637 sensor1;

    int status = MS5637_Initialise(&sensor1, i2c0);

    while(status){
        uint32_t prom_data = read_eeprom_coefficients(&sensor1, MS5637_PROM_ADDR_1);
        printf("EEPROM1: %d\n", prom_data);
    }

    return 0;
}