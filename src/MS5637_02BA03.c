#include "MS5637_02BA03.h"
#include <stdio.h>

uint8_t MS5637_ReadRegisters(void* i2c, const uint8_t addr, const uint8_t reg, uint8_t *data_buff, const uint8_t num_bytes){
    i2c_write_blocking(i2c, addr, &reg, 1, true);  
    uint8_t num_bytes_read = i2c_read_blocking(i2c, addr, data_buff, num_bytes, false);
    return num_bytes_read;
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

    while(true){
        printf("Hello World");
    }

    return 0;
}