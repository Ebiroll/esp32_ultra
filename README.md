#Ultrasonic 3.3V sensor with ESP32 

I got an ultrasonic sensor that actually works with 3.3V VCC.
Note that a normal HC-SR04 sensor will not work. It requires 5V Vcc.
If you want to use it is vital to reduce the echo to 3.3V for input.

This sensor, however actually works on only 3.3V

https://www.aliexpress.com/item/HC-SR04P-Ultrasonic-Ranging-Module-Ranging-Sensor-Module-3-5-5V-Wide-Voltage-Performance-Is-Stronger/32711959780.html


I hooked it up to my ESP32 thing.

I use pin 4 as echo and 15 as trig.
  #define ECHO_PIN GPIO_NUM_4
  #define TRIG_PIN GPIO_NUM_15


https://www.sparkfun.com/products/13907

![connection](connect.png)

#Max sonar sensor.

http://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf

Later I will use the TTL uart interface. But as starting point I will use the analog-voltage pin.
On my sensor it should output a value of (Vcc*2/1024) per 10 mm.
Vdist = dist * 3.3*2/*1024        in cm
dist=Vdist*1024.0/6.6
```

ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_12Bit));
ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_0db));

int times=10;
while (times-->0) {
   vTaskDelay(1000 / portTICK_PERIOD_MS);
   unsigned int val=adc1_get_voltage(ADC1_CHANNEL_7);
        printf("ADC value : %d\n",val );
        double distance = 1024.0*val/6.6;    // mm
        printf("Distance: %f cm\n",distance/10000.0 );

}
```


---------------

When running in qemu, the following ports are accessed.

```
    gpio_pad_select_gpio(ECHO_PIN);
    gpio_pad_select_gpio(TRIG_PIN);


  io read 49048 
  io write 49048,2000 
  io read 4903c 
  io write 4903c,2000 
```



---------------

```
    gpio_set_level(TRIG_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(TRIG_PIN, 0);
    uint32_t startTime=get_usec();

  io write 4903c,0 
  io write 44024,8000 
  io read 440c4 
  io write 440c4,0 
  io read 49048 
  io write 49048,200 
  io write 44028,10 
  io read 44098 
  io write 44098,0 
```