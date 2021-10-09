#include <wiringPi.h>
#include "../include/libdrv8835.h"

int main(int argc, char *argv[]) {
    #ifdef _cplusplus
        using namespace drv8835;
        Motors::moveMotors(-100, 0);
    #endif
    DRV8835_moveMotors(34, 100);
}