/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "sdkconfig.h"
#include "esp_log.h"

void app_main(void)
{   
    //configurate the maximun resolution value of the adc 2^12 = 4096
    adc1_config_width(ADC_WIDTH_BIT_12); 
    //configure the attenuation
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_0);
    while(1){
        //get the adc value, assigned to variable val
        int val = adc1_get_raw(ADC1_CHANNEL_7);
        //mapping adc value to voltage
        float voltage = (val*(3.3))/4096;
        //print val,voltage to serial port 
        printf("adc1_value: %d\n",val);
        printf("voltage_value: %f\n",voltage);
        //delay function
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
