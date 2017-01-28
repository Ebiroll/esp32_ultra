//
// Lots of this based on ncolbans work
// https://github.com/nkolban/esp32-snippets
/*
MIT License

Copyright (c) 2017 Olof Astrand (Ebiroll)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
//#include "driver/adc.h"
#include <sys/time.h>
#include "HardwareSerial.h"
#include "ThingspeakConnection.h"


#define ECHO_PIN GPIO_NUM_4
#define TRIG_PIN GPIO_NUM_15

static const char *TAG = "ultra";


#include "Arduino.h"
#include "A6lib.h"
#include "A6httplib.h"


// I2CScanner from i2scanner.cpp
extern "C" {
	void I2CScanner();
}

// readline from readLine.c
extern "C" char *readLine(uart_port_t uart,char *line,int len);
extern "C" char *pollLine(uart_port_t uart,char *line,int len);



//  Pin number by which the power of module is controlled.
#ifndef ESP8266
  #define module_powerpin 0
#else
  #define module_powerpin D0
#endif

String apn="online.telia.se";
String host="ipinfo.io";
String path="/ip";

#define BUF_SIZE 512

char echoLine[512];


// The max sonar transmits R0000\n once exery 500ms
static void maxSonarInput(void *inpar) {

//        rxPin = 2
//        txPin   // Not used
  uint8_t* data;

  uart_port_t uart_num = UART_NUM_1;                                     //uart port number
  uart_config_t uart_config = {
      .baud_rate = 9600,                      //baudrate
      .data_bits = UART_DATA_8_BITS,          //data bit mode
      .parity = UART_PARITY_DISABLE,          //parity mode
      .stop_bits = UART_STOP_BITS_1,          //stop bit mode
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  //hardware flow control(cts/rts)
      .rx_flow_ctrl_thresh = 122,             //flow control threshold
  };
  ESP_LOGI(TAG, "Setting UART%d...", uart_num);
  ESP_ERROR_CHECK( uart_param_config(uart_num, &uart_config));
  QueueHandle_t uart_queue;
  ESP_ERROR_CHECK( uart_set_pin(uart_num, -1, 2, -1, -1));
  ESP_ERROR_CHECK( uart_driver_install(uart_num, 512 * 2, 512 * 2, 10,  &uart_queue,0));

  //const char* test_str = "This is a test string.\r\n";
  //uart_tx_chars(uart_num, (const char*)test_str,strlen(test_str));
  printf("ESP32 uart Send\n");
  data = (uint8_t*) malloc(BUF_SIZE);
  data[0]=0;
  int len=0;

  while(1) {
    // Sync, look for R
    while (data[0]!='R') {
         len=uart_read_bytes(uart_num, data, 1, 100 / portTICK_RATE_MS);
     }
     len = uart_read_bytes(uart_num, data, 4, 500 / portTICK_RATE_MS);
     if (len==4) {
         data[len]=0;
         printf("got %d:%s\n",len,data);
     }
  }

}



static void uartTestTask(void *inpar) {

//        rxPin = 16;
//        txPin = 17;
  uint8_t* data;

  uart_port_t uart_num = UART_NUM_2;                                     //uart port number
  uart_config_t uart_config = {
      .baud_rate = 115200,                    //baudrate
      .data_bits = UART_DATA_8_BITS,          //data bit mode
      .parity = UART_PARITY_DISABLE,          //parity mode
      .stop_bits = UART_STOP_BITS_1,          //stop bit mode
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  //hardware flow control(cts/rts)
      .rx_flow_ctrl_thresh = 122,             //flow control threshold
  };
  ESP_LOGI(TAG, "Setting UART configuration number %d...", uart_num);
  ESP_ERROR_CHECK( uart_param_config(uart_num, &uart_config));
  QueueHandle_t uart_queue;
  ESP_ERROR_CHECK( uart_set_pin(uart_num, 17, 16, -1, -1));
  ESP_ERROR_CHECK( uart_driver_install(uart_num, 512 * 2, 512 * 2, 10,  &uart_queue,0));

  const char* test_str = "This is a test string.\r\n";
  uart_tx_chars(uart_num, (const char*)test_str,strlen(test_str));
  printf("ESP32 uart Send\n");
  data = (uint8_t*) malloc(BUF_SIZE);

  // Poll task..
  //char *test=pollLine(uart_num,echoLine,256);
  //int slen=strlen(echoLine);
  //echoLine[slen]='\n';
  //echoLine[slen+1]=0;
  //printf("got %s\n",echoLine);


  while(1) {
     int len = uart_read_bytes(uart_num, data, BUF_SIZE, 1000 / portTICK_RATE_MS);

     if (len>0) {
         data[len]='\n';
         data[len+1]=0;
         printf("got %s\n",data);
     }

     vTaskDelay(100 / portTICK_PERIOD_MS);
     uart_tx_chars(uart_num, (const char*)test_str,strlen(test_str));   
  }

}



// This task only echoes what is received on UART2
static void uartECHOTask(void *inpar) {

//        rxPin = 16;
//        txPin = 17;
  char* data;

  uart_port_t uart_num = UART_NUM_2;                                     //uart port number
  uart_config_t uart_config = {
      .baud_rate = 115200,                    //baudrate
      .data_bits = UART_DATA_8_BITS,          //data bit mode
      .parity = UART_PARITY_DISABLE,          //parity mode
      .stop_bits = UART_STOP_BITS_1,          //stop bit mode
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  //hardware flow control(cts/rts)
      .rx_flow_ctrl_thresh = 122,             //flow control threshold
  };
  ESP_LOGI(TAG, "Setting UART configuration number %d...", uart_num);
  ESP_ERROR_CHECK( uart_param_config(uart_num, &uart_config));
  QueueHandle_t uart_queue;
  ESP_ERROR_CHECK( uart_set_pin(uart_num, 17, 16, -1, -1));
  ESP_ERROR_CHECK( uart_driver_install(uart_num, 512 * 2, 512 * 2, 10,  &uart_queue,0));

  printf("ESP32 uart echo\n");

  while(1) {
     data=readLine(uart_num,echoLine,256);
     vTaskDelay(100 / portTICK_PERIOD_MS);
     printf("U2:%s\n",data);
     int len=strlen(data);
     data[len]='\n';
     data[len+1]=0;     
     uart_tx_chars(UART_NUM_1, (const char*)data,strlen(data));   

     //uart_tx_chars(uart_num, (const char*)data,strlen(test_str));   
  }

}



esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

// Similar to uint32_t system_get_time(void)
uint32_t get_usec() {

 struct timeval tv;

 //              struct timeval {
 //                time_t      tv_sec;     // seconds
 //                suseconds_t tv_usec;    // microseconds
 //              };

 gettimeofday(&tv,NULL);
 return (tv.tv_sec*1000000 + tv.tv_usec);


  //uint64_t tmp=get_time_since_boot();
  //uint32_t ret=(uint32_t)tmp;
  //return ret;
}

static void A6TestTask(void *data) {

    A6lib A6l(17, 16);
    A6httplib httprequest(&A6l);

    A6l.blockUntilReady(115200);

    httprequest.ConnectGPRS(apn);
    String rcvd_data=httprequest.Get(host,path);
//    Serial.println(rcvd_data);
    httprequest.getResponseData(rcvd_data);

}

//
// Use the class ThinspeakConnection to try send data to thingspeak 
static void A6Thingspeak(void *data) {

  HardwareSerial* A6conn = new HardwareSerial(2);
  A6conn->begin(115200,SERIAL_8N1,16,17);

  Serial.begin(115200);

  ThingspeakConnection connection(*A6conn);
  float bat=2.0;
  int sensor1=10;
  int sensor2=20;

  boolean pushSuccess = connection.tryPushToThingSpeak(bat, sensor1, sensor2);
  if (pushSuccess) {
     printf("Logg success");
  }
  else 
  {
     printf("Logg falied\n");      
  }

}

//
// Use the module a6_ts try send data to thingspeak 
static void A6_TS(void *data) {


}


//
// Toggle trig pin and wait for input on echo pin 
//
static void ultraDistanceTask(void *inpar) {

    gpio_pad_select_gpio(TRIG_PIN);
    gpio_pad_select_gpio(ECHO_PIN);

    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

    while(1) {
        // HC-SR04P
        gpio_set_level(TRIG_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(TRIG_PIN, 0);
        uint32_t startTime=get_usec();

        while (gpio_get_level(ECHO_PIN)==0 && get_usec()-startTime < 500*1000)
        {
            // Wait until echo goes high
        }

        startTime=get_usec();

        while (gpio_get_level(ECHO_PIN)==1 && get_usec()-startTime < 500*1000)
        {
            // Wait until echo goes low again
        }

        if (gpio_get_level(ECHO_PIN) == 0)
        {
            uint32_t diff = get_usec() - startTime; // Diff time in uSecs
            // Distance is TimeEchoInSeconds * SpeeOfSound / 2
            double distance = 340.29 * diff / (1000 * 1000 * 2); // Distance in meters
            printf("Distance is %f cm\n", distance * 100);
        }
        else
        {
            // No value
            printf("Did not receive a response!\n");
        }
        // Delay and re run.
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }


}


//
// Measure analogue input to get distance
//
#ifdef MAX_SONAR_ANA
static void analogeDistanceTask(void *inpar) {


    // Configure analog input
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_12Bit));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_0db));

    int times=10;
    //int level = 0;
    while (true) {

        // Max sonar analog input
        times=2;
        while (times-->0) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            unsigned int val=adc1_get_voltage(ADC1_CHANNEL_7);
                printf("ADC value : %d\n",val );
                double distance = 1024.0*val/6.6;    // cm
                printf("Distance: %f cm\n",distance/1000.0 );  // mV??
        }
    }
}
#endif

char line[256];

//
// Wait for input on  
//
static void uartLoopTask(void *inpar)
{
    bool ultraDistanceRunning = false;
    QueueHandle_t uart_queue;

    // Could not get to work with UART0 
    uart_port_t uart_num = UART_NUM_1; // uart port number
    uart_config_t uart_config = {
        .baud_rate = 115200,                   //baudrate
        .data_bits = UART_DATA_8_BITS,         //data bit mode
        .parity = UART_PARITY_DISABLE,         //parity mode
        .stop_bits = UART_STOP_BITS_1,         //stop bit mode
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, //hardware flow control(cts/rts)
        .rx_flow_ctrl_thresh = 122,            //flow control threshold
    };
    ESP_LOGI(TAG, "Setting UART configuration number %d...", uart_num);

    // Should be set
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 23, 19, -1, -1));

    // driver_install, otherwise E (20206) uart: uart_read_bytes(841): uart driver error
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 512 * 2, 512 * 2, 10, &uart_queue, 0));
    sprintf(line,"Select 1,2,3,4 or 5\n");
    uart_tx_chars(UART_NUM_1, (const char *)line, strlen(line));
    
    printf("1. Scan i2c bus.\n");
    printf("2. Ultrasound measure.\n");
    printf("3. Echo to UART 2, add crlf before send.\n");
    printf("4. Try connecting over gprs with A6Lib.\n");
    printf("5. Try send data over gprs with A6Thingspeak.\n");


    //#if 0
    char *data;

    while (true)
    {
        data = readLine(uart_num, line, 256);
        if (strcmp(data, "1") == 0)
        {
            printf("I2CScanner\n");
            I2CScanner();
        }
        else if (strcmp(data, "2") == 0)
        {

            if (!ultraDistanceRunning)
            {
                printf("starting ultra task\n");
                xTaskCreatePinnedToCore(&ultraDistanceTask, "ultra", 4096, NULL, 20, NULL, 0);
                ultraDistanceRunning = true;
            }
        }
        else if (strcmp(data, "3") == 0)
        {
            printf("exit to exit echo");
            sprintf(line,"exit to exit echo\n");
            uart_tx_chars(UART_NUM_1, (const char *)line, strlen(line));
            data[0]=0;

            xTaskCreatePinnedToCore(&uartECHOTask, "echo", 4096, NULL, 20, NULL, 0);
            while ((strncmp(data, "exit",4) != 0))
            {
                data = readLine(uart_num, line, 256);
                int len = strlen(data);
                data[len] = '\r';
                data[len + 1] = '\n';
                data[len + 2] = 0;
                // Echo to UART2
                uart_tx_chars(UART_NUM_2, (const char *)data, strlen(data));
            }
            sprintf(line,"exited\n");
            uart_tx_chars(UART_NUM_1, (const char *)line, strlen(line));
        }
        else if (strcmp(data, "4") == 0)
        {
            A6TestTask(NULL);
        }
        else if (strcmp(data, "5") == 0)
        {
            A6Thingspeak(NULL);
        }
        else
        {
            printf("1. Scan i2c bus.\n");
            printf("2. Ultrasound measure.\n");
            printf("3. Echo to UART 2, add crlf before send.\n");
            printf("4. Try connecting over gprs with A6Lib.\n");
            printf("5. Try send data over gprs with A6Thingspeak.\n");

        }
    }
    //#endif
}

extern "C" void app_main(void)
{
    nvs_flash_init();

    //Serial.begin(115200);

    // Note!! Sonar also uses UART 1
    //xTaskCreatePinnedToCore(&maxSonarInput, "sonar", 8048, NULL, 5, NULL, 0);

    // Continusly transmit on uart2, (for qemu tests)
    //xTaskCreatePinnedToCore(&uartTestTask, "uart", 8048, NULL, 5, NULL, 0);

    //xTaskCreatePinnedToCore(&A6TestTask, "A6", 8048, NULL, 5, NULL, 0);
    //xTaskCreatePinnedToCore(&ultraDistanceTask, "ultra", 4096, NULL, 20, NULL, 0);

    xTaskCreatePinnedToCore(&uartLoopTask, "loop", 4096, NULL, 20, NULL, 0);


#if 0
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "access_point_name",
            .password = "password",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
#endif


// 




}
