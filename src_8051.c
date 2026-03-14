#include <reg51.h>

// Motors
sbit MotorA_IN1 = P1^0;
sbit MotorA_IN2 = P1^1;
sbit MotorB_IN1 = P1^2;
sbit MotorB_IN2 = P1^3;

// Fan
sbit Fan_IN1 = P1^4;
sbit Fan_IN2 = P1^5;

// Ultrasonic
sbit TRIG = P3^2;
sbit ECHO = P3^3;

// Temperature alert
sbit TEMP_ALERT = P3^0;

void delay_us(unsigned int us) {
    while(us--);
}

void delay_ms(unsigned int ms) {
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 1275; j++);
}

unsigned int get_distance_cm() {

    unsigned int time = 0;
    unsigned int timeout = 30000;

    TRIG = 0;
    delay_us(2);
    TRIG = 1;
    delay_us(20);
    TRIG = 0;

    while(!ECHO && timeout--)
        delay_us(1);

    if(timeout == 0)
        return 255;

    timeout = 30000;

    while(ECHO && timeout--) {
        time++;
        delay_us(1);
    }

    if(timeout == 0)
        return 255;

    return time / 58;
}

void move_forward() {
    MotorA_IN1 = 0;
    MotorA_IN2 = 1;
    MotorB_IN1 = 0;
    MotorB_IN2 = 1;
}

void move_backward() {
    MotorA_IN1 = 1;
    MotorA_IN2 = 0;
    MotorB_IN1 = 1;
    MotorB_IN2 = 0;
}

void stop_motors() {
    MotorA_IN1 = 0;
    MotorA_IN2 = 0;
    MotorB_IN1 = 0;
    MotorB_IN2 = 0;
}

void fan_on() {
    Fan_IN1 = 1;
    Fan_IN2 = 0;
}

void fan_off() {
    Fan_IN1 = 0;
    Fan_IN2 = 0;
}

void main() {

    unsigned int distance;

    while(1) {

        distance = get_distance_cm();

        if(TEMP_ALERT)
            fan_on();
        else
            fan_off();

        if(distance < 20)
            move_backward();
        else if(distance > 40)
            move_forward();
        else
            stop_motors();

        delay_ms(500);
    }
}