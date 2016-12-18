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