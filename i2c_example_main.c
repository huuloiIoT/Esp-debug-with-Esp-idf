
/********************************************************************************************************************
 *  replace file này vào i2c_self_test
 *  Tạo một bộ buffer cho i2c slave tx kiểu int chứa giá trị nhiệt độ || địa chỉ 0x01  || scl = 5 || sda = 4
 *  2 byte đầu chứa giá trị nhiệt độ  
 *  Dùng i2c master để đọc giá trị này 
 *  in ra giá trị nhiệt độ độ ẩm dạng raw
 *  Humidity: 01F4 = 1×256+15×16+4 =500 => humidity = 500÷10=50.0%RH;
 *  tính toán in ra giá trị nhiệt độ kiểu độ C 
 * 
*/

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define ESP_TEMP_SLAVE_ADDR 0x01 
#define ESP_TEMP_SLAVE_SCL 5
#define ESP_TEMP_SLAVE_SDA 4
#define DATA_TEMP_LENGTH 512
#define RW_TEMP_TEST_LENGTH 128
#define I2C_TEMP_SLAVE_RX_BUF_LEN (2 * DATA_TEMP_LENGTH)
#define I2C_TEMP_SLAVE_TX_BUF_LEN (2 * DATA_TEMP_LENGTH)
#define TEMP_PORT 0

#define ESP_HUMI_SLAVE_ADDR 0x02
#define ESP_HUMI_SLAVE_SCL 2
#define ESP_HUMI_SLAVE_SDA 15
#define DATA_HUMI_LENGTH 512
#define RW_HUMI_TEST_LENGTH 128
#define I2C_HUMI_SLAVE_RX_BUF_LEN (2 * DATA_HUMI_LENGTH)
#define I2C_HUMI_SLAVE_TX_BUF_LEN (2 * DATA_HUMI_LENGTH)
#define HUMI_PORT 2

#define ESP_HUMI_MASTER_SCL 19
#define ESP_HUMI_MASTER_SDA 18
#define MASTER_PORT 1
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_FREQ_HZ 100000

#define GPIO_PULLUP_ENABLE  1
#define I2C_MODE_SLAVE 0
#define I2C_MODE_MASTER 1
#define WRITE_BIT 0                             /*!< I2C master write */
#define READ_BIT 1                              /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

static const char *TAG = "i2c-example";

SemaphoreHandle_t print_mux = NULL;

static esp_err_t i2c_temp_slave_init(void)
{                                                                   
    int i2c_slave_port                  =   TEMP_PORT ;                             // #define CONFIG_I2C_SLAVE_PORT_NUM 0
    i2c_config_t conf_temp_slave;
    conf_temp_slave.sda_io_num          =   ESP_TEMP_SLAVE_SDA;                     // #define CONFIG_I2C_SLAVE_SDA 4       || cấu hình cho bus SDA của slave
    conf_temp_slave.sda_pullup_en       =   GPIO_PULLUP_ENABLE;                  // GPIO_PULLUP_ENABLE = 0x1             || kích hoạt điện trở kéo lên cho bus SDA || giao tiếp spi có 2 điện trở kéo lên ở SCL và SDA
    conf_temp_slave.scl_io_num          =   ESP_TEMP_SLAVE_SCL;                     // #define CONFIG_I2C_SLAVE_SCL 5       || cấu hình cho bus SCL của slave
    conf_temp_slave.scl_pullup_en       =   GPIO_PULLUP_ENABLE;                  // GPIO_PULLUP_ENABLE = 0x1             || kích hoạt điện trở kéo lên cho bus SCL
    conf_temp_slave.mode                =   I2C_MODE_SLAVE;                           // I2C_MODE_SLAVE = 0 || I2C_MODE_MASTER = 1 || chọn chế độ là master hay slave 
    conf_temp_slave.slave.addr_10bit_en =   0;            // uint8_t addr_10bit_en                || cấu hình địa chỉ của slave ở dạng 10 bit
    conf_temp_slave.slave.slave_addr    =   ESP_TEMP_SLAVE_ADDR;            // #define CONFIG_I2C_SLAVE_ADDRESS 0x28|| cho địa chỉ của slave là 0x28
    i2c_param_config(i2c_slave_port, &conf_temp_slave); // I2C parameter initialization
    return i2c_driver_install(i2c_slave_port, conf_temp_slave.mode, I2C_TEMP_SLAVE_RX_BUF_LEN, I2C_TEMP_SLAVE_TX_BUF_LEN, 0);  //esp_err_t i2c_driver_install(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len, int intr_alloc_flags);
}

//static esp_err_t i2c_humi_slave_init(void)
//{                                                                   
//    int i2c_slave_port                  =   HUMI_PORT;                             // #define CONFIG_I2C_SLAVE_PORT_NUM 0
//    i2c_config_t conf_humi_slave;
//    conf_humi_slave.sda_io_num          =   ESP_HUMI_SLAVE_SDA;                    // #define CONFIG_I2C_SLAVE_SDA 4       || cấu hình cho bus SDA của slave
//    conf_humi_slave.sda_pullup_en       =   GPIO_PULLUP_ENABLE;                  // GPIO_PULLUP_ENABLE = 0x1             || kích hoạt điện trở kéo lên cho bus SDA || giao tiếp spi có 2 điện trở kéo lên ở SCL và SDA
//    conf_humi_slave.scl_io_num          =   ESP_HUMI_SLAVE_SCL;                     // #define CONFIG_I2C_SLAVE_SCL 5       || cấu hình cho bus SCL của slave
//    conf_humi_slave.scl_pullup_en       =   GPIO_PULLUP_ENABLE;                  // GPIO_PULLUP_ENABLE = 0x1             || kích hoạt điện trở kéo lên cho bus SCL
//    conf_humi_slave.mode                =   I2C_MODE_SLAVE;                           // I2C_MODE_SLAVE = 0 || I2C_MODE_MASTER = 1 || chọn chế độ là master hay slave 
//    conf_humi_slave.slave.addr_10bit_en =   0;            // uint8_t addr_10bit_en                || cấu hình địa chỉ của slave ở dạng 10 bit
//    conf_humi_slave.slave.slave_addr    =   ESP_HUMI_SLAVE_ADDR;            // #define CONFIG_I2C_SLAVE_ADDRESS 0x28|| cho địa chỉ của slave là 0x28
//    i2c_param_config(i2c_slave_port, &conf_humi_slave); // I2C parameter initialization
//    return i2c_driver_install(i2c_slave_port, conf_humi_slave.mode, I2C_HUMI_SLAVE_RX_BUF_LEN, I2C_HUMI_SLAVE_TX_BUF_LEN, 0);  //esp_err_t i2c_driver_install(i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len, int intr_alloc_flags);
//}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = 1;                            // #define CONFIG_I2C_MASTER_PORT_NUM 1
    i2c_config_t conf_Master;                                              
    conf_Master.mode                =   I2C_MODE_MASTER;                                    // I2C_MODE_MASTER = 1
    conf_Master.sda_io_num          =   ESP_HUMI_MASTER_SDA;                            // #define CONFIG_I2C_MASTER_SDA 18
    conf_Master.sda_pullup_en       =   GPIO_PULLUP_ENABLE;                        // Enable GPIO pull-up resistor
    conf_Master.scl_io_num          =   ESP_HUMI_MASTER_SCL;                            // #define CONFIG_I2C_MASTER_SCL 19
    conf_Master.scl_pullup_en       =   GPIO_PULLUP_ENABLE;                        // Enable GPIO pull-up resistor 
    conf_Master.master.clk_speed    =   I2C_MASTER_FREQ_HZ;                     // #define CONFIG_I2C_MASTER_FREQUENCY 100000
    i2c_param_config(i2c_master_port, &conf_Master);                       // I2C parameter initialization
    return i2c_driver_install(i2c_master_port, conf_Master.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0); // master không cần bộ buffer cho tx va cả rx
}

static esp_err_t i2c_master_read_temp_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();     //cmd = head              // i2c_cmd_link_t *head;     /*!< head of the command link */
    i2c_master_start(cmd);                                                      // Queue command for I2C master to generate a start signal || cmd I2C command link
    i2c_master_write_byte(cmd, (ESP_TEMP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int calculate_temp_C(uint8_t *buf, int len)
{
    //BIN:          1111 1111 1111 1111
    //DEC            a    b    c    d
    //tempC_result: b*(2^8) + c*(2^4) +d*(2^0)  
    int b,c,d,temp;
    b = buf[0] & 15;    // 15  = 00001111 in bin
    c = buf[1] & 240;   // 240 = 11110000 in bin
    c = c >> 4;         // dịch phải 4 để có giá trị đúng của c
    d = buf[1] & 15;    // 15  = 00001111 in bin
    temp = (b*256 + c*16 + d)/10;
    //01F4 = 1×256+15×16+4 =500 => humidity = 500÷10=50.0%RH;
    return temp;
}

static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        //printf("%02x ", buf[i]);  // in mã hex
        printf("%x", buf[i]);      // in mã dec
        if ((i + 1) % 10 == 0) {    // cứ 10 phần tử thì xuống hàng một lần      
            printf("\n");
        }
    }
    printf("\n");

}

static void i2c_read_temp_task(void *arg)
{
    //int i = 0;
    int ret;
    uint8_t *data_temp = (uint8_t *)malloc(DATA_TEMP_LENGTH);             // khởi tạo một vùng nhớ (DATA_LENGTH) và gán địa chỉ đâu tiên của bộ buffer này cho con trỏ data 
    uint8_t *data_rd_temp = (uint8_t *)malloc(DATA_TEMP_LENGTH);
    int cnt = 0;
    while (1) {
        ESP_LOGI(TAG, " test cnt: %d", cnt++);
        //---------------------------------------------------
        //for (i = 0; i <= RW_TEST_LENGTH; i++) {
        //    data[i] = i;                            // data là con trỏ trỏ tới vùng nhớ đã được cấp phát ở trên câu lệnh này ghi giá trị cho từng stack của vùng nhớ
        //}
        data_temp[0] = 0x01;   
        data_temp[1] = 0xf4; 

        // d_size là kiểu trả về của hàm i2c_slave_write_buffer 
        // Nó thể hiện số lượng byte đã được đẩy vào bộ buffer của bộ I2C slave
        size_t d_size_temp;           

        d_size_temp = i2c_slave_write_buffer(TEMP_PORT, data_temp, 2, 1000 / portTICK_RATE_MS);                                                                                             
        if (d_size_temp == 0) {
            ESP_LOGW(TAG, "i2c slave tx buffer full");
            ret = i2c_master_read_temp_slave(MASTER_PORT, data_rd_temp, DATA_TEMP_LENGTH);
        } else {
            ret = i2c_master_read_temp_slave(MASTER_PORT, data_rd_temp, RW_TEMP_TEST_LENGTH);
        }

        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C Timeout");
        } else if (ret == ESP_OK) {
            printf("*******************\n");
            printf(" MASTER READ TEMP FROM SLAVE\n");
            printf("*******************\n");
            printf("==== Slave temp buffer data ====\n");
            disp_buf(data_temp, d_size_temp);             // data là con trỏ kiểu uint8_t trỏ tới vùng nhớ được cấp phát đây là vùng nhớ của slave
            printf("==== Master read raw temp ====\n");
            disp_buf(data_rd_temp, d_size_temp);
            int temp = calculate_temp_C(data_rd_temp, d_size_temp);
            printf(" Giá trị nhiệt độ là: %d C \n", temp);
        } else {
            ESP_LOGW(TAG, "%s: Master read slave error, IO not connected...\n", esp_err_to_name(ret));
        }
        vTaskDelay(1000 / portTICK_RATE_MS);        
    }
    vTaskDelete(NULL);
}

void app_main(void)
{

    ESP_ERROR_CHECK(i2c_temp_slave_init());
    //ESP_ERROR_CHECK(i2c_humi_slave_init());
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(i2c_read_temp_task, "read the teamperature", 1024 * 2, (void *)0, 10, NULL);

}
