#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h" 

#define BUFFER_SIZE 1020                                        //day la mot macro co nhiem vu gan gia tri cho BUFFER_SIZE

int subtract(int a, int b){                                     //khai bao funcion subtract cos nhieem vu tru hai so voi 2 tham so dau vao la va b
    int c;                                                      //ham subtract co gia tri tra ve la c, voi dieu kien c phai co cung kieu du lieu voi subtract
    c = a - b;
    return c;
}

void app_main(void){
    int a = 5;
    int b = 2;
    while(1) {
        printf("a-b = %d || ",subtract(10,4));
        printf("%d - %d = %d \n", 10, 7, subtract(10, 7));
        printf("Gia tri cua buffer la: %d \n",BUFFER_SIZE);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    
}

