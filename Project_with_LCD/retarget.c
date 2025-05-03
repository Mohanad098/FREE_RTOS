#include <stdio.h>
#include "TM4C123.h"
#include "core_cm4.h"  // Ensure the correct Cortex-M header is included

int __write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        ITM_SendChar(ptr[i]);  // Send output to ITM
    }
    return len;
}

int _sys_exit(int x) {
    x = x;
    return 0;
}
