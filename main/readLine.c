#include "driver/uart.h"


char *readLine(uart_port_t uart,char *line,int len) {
	int size;
	char *ptr = line;
	while(1) {
		size = uart_read_bytes(uart, (unsigned char *)ptr, 1, portMAX_DELAY);
		if (size == 1) {
			if (*ptr == '\n') {
				*ptr = 0;
				return line;
			}
			ptr++;
		} // End of read a character
	} // End of loop
} // End of readLine
