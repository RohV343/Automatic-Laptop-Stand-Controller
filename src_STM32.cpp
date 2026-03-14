#include "mbed.h"

// Motor A
DigitalOut MotorA_IN1(D2);
DigitalOut MotorA_IN2(D3);

// Motor B
DigitalOut MotorB_IN1(D4);
DigitalOut MotorB_IN2(D5);

// Fan
DigitalOut FAN_PIN(D6);

// Ultrasonic
DigitalOut TRIG(D8);
DigitalIn  ECHO(D9);

// Temperature
DigitalIn TEMP_ALERT(D10);
AnalogIn TEMP_SENSOR_PIN(A0);

// Serial
BufferedSerial pc(USBTX, USBRX, 9600);

 
long getDistanceCM()
{
    TRIG = 0;
    wait_us(2);

    TRIG = 1;
    wait_us(10);

    TRIG = 0;

    Timer t;
    t.start();

    while(!ECHO)
    {
        if(t.elapsed_time().count() > 30000) return 255;
    }

    t.reset();

    while(ECHO)
    {
        if(t.elapsed_time().count() > 30000) return 255;
    }

    long duration = t.elapsed_time().count();
    return duration / 58;
}



float readTemperatureC()
{
    float voltage = TEMP_SENSOR_PIN.read() * 5.0;
    return voltage * 100.0;
}


void moveForward()
{
    MotorA_IN1 = 0;
    MotorA_IN2 = 1;
    MotorB_IN1 = 0;
    MotorB_IN2 = 1;
}

void moveBackward()
{
    MotorA_IN1 = 1;
    MotorA_IN2 = 0;
    MotorB_IN1 = 1;
    MotorB_IN2 = 0;
}

void stopMotors()
{
    MotorA_IN1 = 0;
    MotorA_IN2 = 0;
    MotorB_IN1 = 0;
    MotorB_IN2 = 0;
}


void fanOn()
{
    FAN_PIN = 1;
}

void fanOff()
{
    FAN_PIN = 0;
}


int main()
{
    char buffer[100];

    stopMotors();
    fanOff();

    while(true)
    {
        long distance = getDistanceCM();
        float temperature = readTemperatureC();

        sprintf(buffer,
                "Distance: %ld cm  Temp: %.2f C\r\n",
                distance,
                temperature);

        pc.write(buffer, strlen(buffer));

        if(temperature > 30.0)
            fanOn();
        else
            fanOff();

        if(distance < 20)
            moveBackward();
        else if(distance > 40)
            moveForward();
        else
            stopMotors();

        ThisThread::sleep_for(500ms);
    }
}