#include "Arduino.h"
#include <sys/time.h>

uint32_t millis() {

 struct timeval tv;


 gettimeofday(&tv,NULL);
 return (tv.tv_sec*1000 + tv.tv_usec*1000);

}

