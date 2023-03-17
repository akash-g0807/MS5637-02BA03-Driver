#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "MS5637_02BA03.h"


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

        MS5637_ReadTemperature_and_Pressure(&sensor1, 5);
        printf("%f,0,\n", sensor1.temperature);

    }

    return 0;
}