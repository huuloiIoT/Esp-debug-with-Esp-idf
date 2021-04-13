/*
    1.ESP32 có hai đơn vị MCPWM có thể được sử dụng để điều khiển các loại động cơ khác nhau. Mỗi đơn vị có ba cặp đầu ra PWM.
    1.1.Những module phụ trợ cho việc điều khiển motor Fault Handler, signal Capture, Carrier and Interrupts.
    2.Phạm vi cấu hình phụ thuộc vào loại động cơ, cụ thể là số lượng đầu ra và đầu vào được yêu cầu và chuỗi tín hiệu sẽ là gì để điều khiển động cơ
    2.1 ************************************* CẤU HÌNH CHÂN BĂM XUNG********************************************
    2.1.1   các bước thực hiện
            2.1.1.1. Chọn một MPWn unit dùng để điều khiển motor. Có 2 unit sẵn có trên ESP32 và được liệt kê(enum) trong định nghĩa biến mcpwm_unit_t || MCPWM_UNIT_0 : MCPWM unit0 được chọn || MCPWM_UNIT_1 : MCPWM unit1 được chọn
            2.1.1.2. Cấu hình 2 GPIOs ở chế độ output trong unit được lựa chọn bằng lệnh calling mcpwm_gpio_init().
                        esp_err_t mcpwm_gpio_init(mcpwm_unit_t  mcpwm_num, mcpwm_io_signals_t   io_signal, int gpio_num)
                        mcpwm_num: set MCPWM unit(0-1)  xem MCPWM Overview
                        io_signal: set MCPWM signals, mỗi đơn vị MCPWM có 6 output(MCPWMXA, MCPWMXB) và 9 input(SYNC_X, FAULT_X, CAP_X) ‘X’ is timer_num(0-2) xem MCPWM Block Diagram
                        gpio_num: đặt giá trị này là 18 nếu muốn dùng chân 18   ||  gpio_num = 18
                        VD: mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);         ||  MCPWM_UNIT_0 chọn unit0     ||  MCPWM0A
                     Giải thích:   
                     Tất cả các chế độ tín hiệu được liệt kê ở mcpwm_io_signals_t.  ||  MCPWM0A:PWM0A output pin || MCPWM0B:PWM0B output pin || MCPWM_SYNC_0:SYNC0 input pin || MCPWM_FAULT_0:FAULT0 input pin || MCPWM_CAP_0:CAP0 input pin
                     Để cài đặt nhiều chân cùng 1 lúc ta dùng hàm mcpwm_set_pin() cùng với mcpwm_pin_config_t || esp_err_t mcpwm_set_pin(mcpwm_unit_t mcpwm_num, const mcpwm_pin_config_t *mcpwm_pin) || mcpwm_num: set MCPWM unit(0-1) || mcpwm_pin: MCPWM pin structure
            2.1.1.3. Chọn timer. Có ba bộ timer có sẵn trong unit. và được liệt kê ở mcpwm_timer_t. || MCPWM_TIMER_0: chọn điều khiển bằng timer0 || MCPWM_TIMER_1: chọn điều khiển bằng timer1
            2.1.1.4. Cài đặt tần số timer và duty cycle bằng cấu trúc mcpwm_config_t.
                        struct mcpwm_config_t
                        uint32_t frequency : Set frequency of MCPWM in Hz
                        float cmpr_a : cài đặt duty cycle cho operator a(MCPWMXA), VD để có 62.3% duty cycle, duty_a = 62.3
                        float cmpr_b : cài đặt duty cycle cho operator b(MCPWMXB), VD để có 48% duty cycle, duty_b = 48.0
                        mcpwm_duty_type_t duty_mode : cài đặt kiểu duty cycle || MCPWM_DUTY_MODE_0 = 0 duty cycle tỉ lệ với thời gian xuất hiện mức cao sử dụng cho chế độ không đối xứng MCPWM || MCPWM_DUTY_MODE_1 duty cycle tỉ lệ với mức thấp, chế độ không đối xứng || MCPWM_HAL_GENERATOR_MODE_FORCE_LOW || MCPWM_HAL_GENERATOR_MODE_FORCE_HIGH
                        mcpwm_counter_type_t counter_mode : cài đặt kiểu MCPWM counter || MCPWM_UP_COUNTER = 1 đếm lên không đối xứng MCPWM || MCPWM_DOWN_COUNTER đếm xuống không đối xứng MCPWM || MCPWM_UP_DOWN_COUNTER đếm lên xuống đối xứng MCPWM, tần số là một nửa của tần số đặt MCPWM 
            2.1.1.5. Gọi hàm mcpwm_init() để các cài đặt có hiệu lực.
                        esp_err_t mcpwm_init(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, const mcpwm_config_t *mcpwm_conf)
                            mcpwm_num: set MCPWM unit(0-1)
                            timer_num: set timer number(0-2) of MCPWM, each MCPWM unit has 3 timers.
                            mcpwm_conf: configure structure mcpwm_config_t

                        VD: mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
            2.1.1.6. Ngắt 
                        Gọi ngắt bằng cách tạo hàm mcpwm_isr_register().
                        esp_err_tmcpwm_isr_register(mcpwm_unit_t mcpwm_num, void (*fn)(void *), void *arg, int intr_alloc_flags, intr_handle_t *handle, )
                        Register MCPWM interrupt handler, the handler is an ISR. the handler will be attached to the same CPU core that this function is running on.
                        mcpwm_num: cài đặt MCPWM unit(0-1)
                        fn: khi ngắt xảy ra thì hàm này sẽ được thực hiện.
                        arg: user-supplied argument passed to the handler function.
                        intr_alloc_flags: flags used to allocate the interrupt. One or multiple (ORred) ESP_INTR_FLAG_* values. see esp_intr_alloc.h for more info.
                        handle: pointer to return handle. If non-NULL, a handle for the interrupt will be returned here.
*/
#include "driver/gpio.h"
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"


static esp_adc_cal_characteristics_t *adc_chars;                    //khai báo con trỏ adc_char
float adc_reading;
uint32_t voltage;
uint32_t angle, count ;


//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);    //Set GPIO 18 as PWM0A, to which servo is connected
}

/**
 * @brief Use this function to calcute pulse width for per degree rotation
 *
 * @param  degree_of_rotation the angle in degree to which servo has to rotate
 *
 * @return
 *     - calculated pulse width
 */
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)  //giá trị truyền vào là độ dài xung nằm trong khoảng 1000 tới 2000 xung
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));        
    return cal_pulsewidth;
}

/**
 * @brief Configure MCPWM module
 */
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void mcpwm_example_servo_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();                                                //cấu hình chân băm xung 

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        float raw_count = adc1_get_raw(ADC1_CHANNEL_4);                                 // raw_count là giá trị đọc trực tiếp từ bộ adc
        //printf("raw_count: %f\n", raw_count); 
        uint32_t raw1_count = (raw_count*90)/4095;                                      // raw1_count là giá trị quy đổi ra góc 
        //printf("raw1_count: %d\n", raw1_count);
        count = (uint32_t)raw1_count;                                                   // ép kiểu cho biến count
        //printf("Angle of rotation: %d\n", count);
        angle = servo_per_degree_init(count);                                           // tính ra chu kì xung tương ứng với góc 1000~2000 us
        //printf("Angle of rotation: %d\n", count);
        //printf("pulse width: %dus\n", angle);
        //printf("count = %d\n", count);
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);          // cài đặt dutycycle
        vTaskDelay(10);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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
    voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);                       //giá trị raw của adc || adc_chars Con trỏ đến cấu trúc đã khởi tạo có chứa các đặc tính ADC
    printf("Raw: %f\tVoltage: %dmV\n", adc_reading, voltage); 
    printf("Angle of rotation: %d\n", count); 
    printf("pulse width: %dus\n", angle);         
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));                       // 1 là số phần tử được cấp phát cho mảng || sizeof(esp_adc_cal_characteristics_t) là kích thước theo byte của phần tử
    
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,     1100     ,  adc_chars);
    //                                                                      DEFAULT_VREF || adc_chars : con trỏ tới ngăn nhớ rỗng để lưu giá trị adc                     
    nvs_flash_init();

    adc1_config_width(ADC_WIDTH_12Bit);                                                   // chỉnh độ phân giải kênh 1 || 12 bit = 2^12 = 4096
    adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_11db);                             // adc1 channel 4 là gpio 32 xem api manual

    xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(&dac_out_task, "dac_out_task", 2048, NULL, 5,NULL, 0);  
}
