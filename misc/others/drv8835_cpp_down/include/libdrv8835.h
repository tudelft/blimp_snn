#include <wiringPi.h>

#ifndef __GNUC__
    #error "This code can only be compiled by a gnu compiler"
#endif

// Hardware settings for the drv8835 motor 
// driver.
static const short MAX_SPEED = 480;
static const char PIN_LEFT = 12;
static const char PIN_RIGHT = 13;
static const char DIR_PIN_LEFT = 5;
static const char DIR_PIN_RIGHT = 6;


//Global speed and direction variables
static int lspeed = 0;  // left speed
static int rspeed = 0;  // right speed
static int ldir = 0;    // left direction (foward or backwards)
static int rdir = 0;    // right direction (forward or backwards)

// The constructor attribute is gcc specific.
// On the initalisation of a program that uses this
// library the function __on_startup is called.
__attribute__((constructor))
static void __on_startup() {
    //Set up the Gpio ports.
    wiringPiSetupGpio();

    // These are the pins used for spped
    //control.
    pinMode(PIN_LEFT, PWM_OUTPUT);
    pinMode(PIN_RIGHT, PWM_OUTPUT);

    // Initialise the PWM setttings
    // for our hardware.
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(MAX_SPEED);
    pwmSetClock(2);

    //These are the pins used for direction control.
    pinMode(DIR_PIN_LEFT, OUTPUT);
    pinMode(DIR_PIN_RIGHT, OUTPUT);
}

static inline void setSpeed(char motor_pin,int *dir_pin, int percent) {
    if(percent < 0) {
        *dir_pin = 1;
        percent = -1*percent;
    }else {
        *dir_pin = 0;
    }

    if(percent > MAX_SPEED*(percent/100)) {
        percent = MAX_SPEED;
    }else {
        percent = (int)MAX_SPEED*(percent/100);
    }

    digitalWrite(motor_pin -7, *dir_pin);
    pwmWrite(motor_pin, percent);
}


// This is a C interface to move the motors.
inline void DRV8835_moveMotors(int left_percent, int right_percent) {
    setSpeed(PIN_LEFT, &ldir, left_percent);
    setSpeed(PIN_RIGHT, &rdir, right_percent);
}


inline int DRV8835_getSpeed(int motor) {
    if (motor == 0) {
        return lspeed;
    }else {
        return rspeed;
    }
}

inline int DRV8835_getDir(int motor) {
    if(motor == 0) {
        return ldir;
    }else {
        return rdir;
    }
}   

// Static c++ interface with namespace to avoid 
// namespace clashes.
#ifdef __cplusplus
namespace drv8835 {
    class Motors {
        // There is no need to check whether we have initialised
        // since the constructor makes sure that 
        // the call __on_startup is run.
        public:
        // moveMotors This funciton expects two arguements, these are discrete percentages
        // varying from -100% to 100%, a negative percentage moves the motors in
        // a different direction to a positive percentage.
        static void moveMotors(int left_percent, int right_percent) {
            DRV8835_moveMotors(left_percent, right_percent);
        }
        static int getSpeed(int motor) {
            return DRV8835_getSpeed(motor);
        }
        static int getDir(int motor) {
            return DRV8835_getDir(motor);
        }   
    };
};
#endif



