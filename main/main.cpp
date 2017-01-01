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

#define ECHO_PIN GPIO_NUM_4
#define TRIG_PIN GPIO_NUM_15

static const char *TAG = "ultra";


#include "Arduino.h"
#include "A6lib.h"
#include "A6httplib.h"

// Instantiate the library with TxPin, RxPin.


//  Pin number by which the power of module is controlled.
#ifndef ESP8266
  #define module_powerpin 0
#else
  #define module_powerpin D0
#endif

String apn="airtelworld.com";
String host="ipinfo.io";
String path="/ip";

#define BUF_SIZE 512

// Later Try uart 1
static void maxSonarInput(void *inpar) {


//        rxPin = 9;
//        txPin = 10;  // Not used
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
  ESP_LOGI(TAG, "Setting UART configuration number %d...", uart_num);
  ESP_ERROR_CHECK( uart_param_config(uart_num, &uart_config));
  QueueHandle_t uart_queue;
  ESP_ERROR_CHECK( uart_set_pin(uart_num, -1, 2, -1, -1));
  ESP_ERROR_CHECK( uart_driver_install(uart_num, 512 * 2, 512 * 2, 10,  &uart_queue,0));

  //const char* test_str = "This is a test string.\r\n";
  //uart_tx_chars(uart_num, (const char*)test_str,strlen(test_str));
  printf("ESP32 uart Send\n");
  data = (uint8_t*) malloc(BUF_SIZE);
  // Sync
  data[0]=0;
  int len=0;
  //while (data[0]!='R') {
  //  len=uart_read_bytes(uart_num, data, 7, 100 / portTICK_RATE_MS);
  //}

  while(1) {
    while (data[0]!='R') {
         len=uart_read_bytes(uart_num, data, 1, 100 / portTICK_RATE_MS);
     }
     len = uart_read_bytes(uart_num, data, 4, 500 / portTICK_RATE_MS);
     if (len==4) {
         data[len]=0;
         //data[len+1]=0;
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

  while(1) {
     int len = uart_read_bytes(uart_num, data, BUF_SIZE, 1000 / portTICK_RATE_MS);

     if (len>0) {
         data[len]='\n';
         data[len+1]=0;
         printf("got %s\n",data);
         uart_write_bytes(uart_num, (const char*) data, len);
     }

     vTaskDelay(100 / portTICK_PERIOD_MS);
     uart_tx_chars(uart_num, (const char*)test_str,strlen(test_str));   
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

#if 0
static void A6TestTask(void *data) {

    A6lib A6l(7, 8);
    A6httplib httprequest(&A6l);

    A6l.blockUntilReady(115200);

    httprequest.ConnectGPRS(apn);
    String rcvd_data=httprequest.Get(host,path);
//    Serial.println(rcvd_data);
    httprequest.getResponseData(rcvd_data);

}
#endif

extern "C" void app_main(void)
{
    nvs_flash_init();

    Serial.begin(115200);



    xTaskCreatePinnedToCore(&maxSonarInput, "sonar", 8048, NULL, 5, NULL, 0);

    //xTaskCreatePinnedToCore(&uartTestTask, "uart", 8048, NULL, 5, NULL, 0);
    //xTaskCreatePinnedToCore(&A6TestTask, "A6", 8048, NULL, 5, NULL, 0);




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



    gpio_pad_select_gpio(TRIG_PIN);
    gpio_pad_select_gpio(ECHO_PIN);

    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);


#ifdef MAX_SONAR
    // Configure analog input
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_12Bit));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_0db));
#endif

    int times=10;
    //int level = 0;
    while (true) {

        // Max sonar analog input
        times=2;
#ifdef MAX_SONAR
        while (times-->0) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            unsigned int val=adc1_get_voltage(ADC1_CHANNEL_7);
                printf("ADC value : %d\n",val );
                double distance = 1024.0*val/6.6;    // cm
                printf("Distance: %f cm\n",distance/1000.0 );  // mV??
        }
#endif

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
