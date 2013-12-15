#include <Servo.h> 
Servo motor_one;
Servo motor_two;
Servo motor_three;
Servo motor_four;

void setup() { 
    motor_one.attach(8); 
    motor_two.attach(9);
    motor_three.attach(10);
    motor_four.attach(11);

    motor_one.write(20);
    motor_two.write(20);
    motor_three.write(20);
    motor_four.write(20);
    delay(1000);

    int power = 20;

    motor_one.write(power);
    motor_two.write(power);
    // motor_three.write(power);
    // motor_four.write(power);
} 
 
void loop() {
}
