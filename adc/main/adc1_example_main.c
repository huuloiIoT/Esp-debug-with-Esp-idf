//Note :------------------------------------
//ADC2 dùng cho driver của wifi. vì thế chỉ được dùng khi không bật wifi
//ADC driver API hỗ trợ ADC1 (8 channels, được gấn với GPIOs 32 - 39)
//ADC driver API hỗ trợ ADC2 (10 channels, được gấn với GPIOs 0, 2, 4, 12 - 15 and 25 - 27)

// *********************************CẤU HÌNH CHÂN ĐỌC ADC*********************************
//2 API chính || adc1_config_width(ADC_WIDTH_BIT_t) || adc1_config_channel_atten(ADC_ATTEN_DB_t)
//ADC_WIDTH_BIT_12: cài đặt độ phân giải 12 bit || dải bit có thể set từ 9~10 bit
//ADC_ATTEN_DB_0: cài đặt độ suy giảm là 0 đo được 800mV || ADC_ATTEN_DB_2_5 đo được 1100mV || ADC_ATTEN_DB_6 đo được 1350mV || ADC_ATTEN_DB_11 đo được 2600mV 

// *********************************ĐỌC ADC*********************************
//đọc giá trị chuyển đổi adc bằng câu lệnh adc1_get_raw(adc1_channel_t) || adc2_get_raw(adc2_channel_t)
//adc1_channel_2 là GPIO38 || adc1_channel_3 là GPIO39 || adc2_channel_2 là GPIO2 || adc2_channel_3 là GPIO15

// *********************************CHỐNG NHIỄU*********************************
//ESP32 rất nhạy với nhiễu dẫn đến sự khác biệt lớn giữa các giá trị đọc về. Để giảm nhiễu chúng ta có thể mắc thêm một tụ 1mF hoặc lấy mẫu nhiều lần
//adc_vref_to_gpio () được sử dụng để định tuyến điện áp tham chiếu bên trong đến chân GPIO Sẽ rất hữu ích khi hiệu chỉnh việc đọc ADC

// *********************************HIỆU CHUẨN*********************************
//Theo thiết kế, điện áp tham chiếu ADC là 1100 mV, tuy nhiên điện áp tham chiếu thực có thể nằm trong khoảng từ 1000 mV đến 1200 mV giữa các ESP32 khác nhau.
//Có 3 cách hiệu chuẩn
//  1.Hai giá trị Điểm đại diện: ghi vào eFuse BLOCK3 cho mỗi số đọc của ADC ở 150 mV và 850 mV.
//  2.eFuse Vref đại diện cho điện áp tham chiếu ADC thực. Giá trị này được đo và ghi vào eFuse BLOCK0 trong quá trình hiệu chuẩn tại nhà máy.
//  3.Vref mặc định là một ước tính của điện áp tham chiếu ADC do người dùng cung cấp như một tham số trong quá trình mô tả đặc tính. Nếu không có giá trị Two Point hoặc eFuse Vref, Vref mặc định sẽ được sử dụng.
//      3.1.xem eFuse Vref có xuất hiện hay không bằng cách chạy công cụ espefuse.py với tham số adc_info
//      3.2.Board hiện tại đang dùng eFuse Vref


#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

static esp_adc_cal_characteristics_t *adc_chars;
float adc_reading;


void dac_out_task(void *pvParameters)
{
  //static uint8_t i = 0;
  while (1) {                                                                           // DAC chỉ có 2 kênh
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);
    dac_output_voltage(DAC_CHANNEL_1,200);                                              // kênh DAC 8 bit       || DAC_CHANNEL_1 = GPIO25 : kênh 1  || giá trị điện áp = VDD/255*200
    dac_output_voltage(DAC_CHANNEL_2,100);                                              // kênh DAC 8 bit       || DAC_CHANNEL_2 = GPIO26 : kênh 2  || giá trị điện áp = VDD/255*100
    for (int i = 0; i < 64; i++) {                                                      // the sapmple value is 64 lấy mẫu 64 lần                                         
        adc_reading += adc1_get_raw(ADC1_CHANNEL_4);                                    // ghi giá trị đọc được từ bộ adc1 vào biến adc_reading
    }
    adc_reading = adc_reading / 64;                                                     // chia giấ trị để có giá trị trung bình
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);   
    printf("Raw: %f\tVoltage: %dmV\n", adc_reading, voltage);           
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


void app_main()
{
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,     1100     ,  adc_chars);
    //                                                                      DEFAULT_VREF || adc_chars                      
    nvs_flash_init();
    adc1_config_width(ADC_WIDTH_12Bit);                                                   // chỉnh độ phân giải kênh 1 || 12 bit = 2^12 = 4096
    adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_11db);                             // adc1 channel 4 là gpio 32 xem api manual
    xTaskCreatePinnedToCore(&dac_out_task, "dac_out_task", 2048, NULL, 5,NULL, 0);        // API RTOS
}

