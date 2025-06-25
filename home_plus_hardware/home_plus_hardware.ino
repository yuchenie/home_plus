#include <EEPROM.h> 
#include <Servo.h>
// #include <Encoder.h>

float x, y, theta;

// === Servo Setup ===
Servo servo1, servo2, servo3, servo4, servo5;
int servo1Angle, servo2Angle, servo3Angle, servo4Angle, servo5Angle;

Servo wrist, elbow, shoulder;
int wristAngle, elbowAngle, shoulderAngle;

// === Timing for Servos ===
unsigned long previousServoMillis = 0;
const long servoInterval = 10; // 10ms update interval

// === Motor Pins ===

// back left wheel
#define SW_PWM 4 
#define SW_CW 53
#define SW_CCW 52
// Encoder enone(22, 23);

// front right wheel
#define NE_PWM 5 
#define NE_CCW 51
#define NE_CW 50
// Encoder entwo(24, 25);

// front left wheel
#define NW_PWM 6 
#define NW_CW 49
#define NW_CCW 48
// Encoder enthree(26, 27);

// back right wheel
#define SE_PWM 7
#define SE_CW 46
#define SE_CCW 47
// Encoder enfour(28, 29);

// === Linear Actuators ===
#define DIR1_LA 44
#define DIR2_LA 45
#define ENA1 41

#define DIR1_LA2 42
#define DIR2_LA2 43
#define ENA2 40

void stopAll();  // forward declaration

void setup() {
    Serial.begin(9600);

    /*
    servo1.attach(10);
    servo2.attach(11);
    servo3.attach(12);
    servo4.attach(13);
    servo5.attach(9);

    servo1Angle = EEPROM.read(0);
    servo2Angle = EEPROM.read(1);
    servo3Angle = EEPROM.read(2);
    servo4Angle = EEPROM.read(3);
    servo5Angle = EEPROM.read(4);

    if (servo1Angle < 0 || servo1Angle > 180) servo1Angle = 90;
    if (servo2Angle < 0 || servo2Angle > 180) servo2Angle = 90;
    if (servo3Angle < 0 || servo3Angle > 180) servo3Angle = 90;
    if (servo4Angle < 0 || servo4Angle > 180) servo4Angle = 90;
    if (servo5Angle < 0 || servo5Angle > 180) servo5Angle = 90;

    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);
    servo4.write(servo4Angle);
    servo5.write(servo5Angle);
    */

    wrist.attach(13);
    elbow.attach(12);

    Serial.println("USB Serial Ready");
    // printServoAngles();
}


void parseMessage(String input) {
  int firstSpace = input.indexOf(' ');
  int secondSpace = input.indexOf(' ', firstSpace + 1);

  x = input.substring(0, firstSpace).toFloat();
  y = input.substring(firstSpace + 1, secondSpace).toFloat();
  theta = input.substring(secondSpace + 1).toFloat();
}

void loop() {
    unsigned long currentMillis = millis();
    static String lastCommand = "x";

    if (Serial.available() > 0) {
        lastCommand = "";
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n') break;
            lastCommand += c;
            delay(1);
        }
    }

    Serial.println(lastCommand);

    if (lastCommand.length() == 1) {
        char cmd = lastCommand.charAt(0);
        switch (cmd) {
            case '1':
                writePercent(1, 0, 0);
                delay(3000);

                writePercent(0, 0, 0);
                delay(500);

                writePercent(0, 1, 0);
                delay(5000);

                writePercent(0, 0, 0);
                break;
            case '2':
                writePercent(0, -1, 0);
                delay(5000);

                writePercent(0, 0, 0);
                delay(500);
                
                writePercent(-1, 0, 0);
                delay(3000);
                
                writePercent(0, 0, 0);
                break;
            case ' ': writePercent(0, 0, 0); break;
        }
    } else {
        parseMessage(lastCommand);
        writePercent(x, y, theta);
        int time = 2000 * (abs(x)+abs(y)+abs(theta));
        delay(time);
        writePercent(0, 0, 0);
    }
    
    lastCommand = " ";

    /* if (currentMillis - previousServoMillis >= servoInterval) {
        previousServoMillis = currentMillis;
        bool angleChanged = false;

        for (int i = 0; i < lastCommand.length(); i++) {
            char c = lastCommand.charAt(i);
            switch (c) {
                case '1': if (servo1Angle > 0) { servo1Angle--; servo1.write(servo1Angle); EEPROM.update(0, servo1Angle); angleChanged = true; } break;
                case '2': if (servo1Angle < 180) { servo1Angle++; servo1.write(servo1Angle); EEPROM.update(0, servo1Angle); angleChanged = true; } break;
                case '3': if (servo2Angle > 0) { servo2Angle--; servo2.write(servo2Angle); EEPROM.update(1, servo2Angle); angleChanged = true; } break;
                case '4': if (servo2Angle < 180) { servo2Angle++; servo2.write(servo2Angle); EEPROM.update(1, servo2Angle); angleChanged = true; } break;
                case '5': if (servo3Angle > 0) { servo3Angle--; servo3.write(servo3Angle); EEPROM.update(2, servo3Angle); angleChanged = true; } break;
                case '6': if (servo3Angle < 180) { servo3Angle++; servo3.write(servo3Angle); EEPROM.update(2, servo3Angle); angleChanged = true; } break;
                case '7': if (servo4Angle > 0) { servo4Angle--; servo4.write(servo4Angle); EEPROM.update(3, servo4Angle); angleChanged = true; } break;
                case '8': if (servo4Angle < 180) { servo4Angle++; servo4.write(servo4Angle); EEPROM.update(3, servo4Angle); angleChanged = true; } break;
                case '9': if (servo5Angle < 145) { servo5Angle++; servo5.write(servo5Angle); EEPROM.update(4, servo5Angle); angleChanged = true; } break;
                case '0': if (servo5Angle > 80) { servo5Angle--; servo5.write(servo5Angle); EEPROM.update(4, servo5Angle); angleChanged = true; } break;
            }
        }

        if (angleChanged) {
            printServoAngles();
        }
    } */
}

void executeSequence() {
    // drive(0, 1);
    Serial.println("Car moving forward...");
    delay(2000);
    // stopAll();
    Serial.println("Car stopped.");

    controlLA(-1);
    Serial.println("Linear Actuator Moving Down...");
    delay(3000);
    controlLA(0);
    Serial.println("Linear Actuator Stopped.");

    servo1Angle = constrain(servo1Angle + 10, 0, 180);
    servo2Angle = constrain(servo2Angle + 10, 0, 180);
    servo3Angle = constrain(servo3Angle + 10, 0, 180);
    servo4Angle = constrain(servo4Angle + 10, 0, 180);
    servo5Angle = constrain(servo5Angle + 10, 0, 180);

    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
    servo3.write(servo3Angle);
    servo4.write(servo4Angle);
    servo5.write(servo5Angle);

    EEPROM.update(0, servo1Angle);
    EEPROM.update(1, servo2Angle);
    EEPROM.update(2, servo3Angle);
    EEPROM.update(3, servo4Angle);
    EEPROM.update(4, servo5Angle);

    Serial.println("Servos moved +10 degrees.");
    printServoAngles();
}

void printServoAngles() {
    Serial.print("Servo Angles -> ");
    Serial.print("Servo1: "); Serial.print(servo1Angle);
    Serial.print("°, Servo2: "); Serial.print(servo2Angle);
    Serial.print("°, Servo3: "); Serial.print(servo3Angle);
    Serial.print("°, Servo4: "); Serial.print(servo4Angle);
    Serial.print("°, Servo5: "); Serial.print(servo5Angle);
    Serial.println("°");
}

void writePercent(float x, float y, float theta) {
    float nw = (x - y - theta);
    float ne = (x + y - theta);
    float sw = (x + y + theta);
    float se = (x - y + theta);

    float max = max(abs(nw), max(abs(ne), max(abs(sw), abs(se))));
    if (max > 1.0) {
        nw /= max;
        ne /= max;
        sw /= max;
        se /= max;
    } 

    setMotorPower(NW_PWM, NW_CW, NW_CCW, nw);
    setMotorPower(NE_PWM, NE_CW, NE_CCW, ne);
    setMotorPower(SW_PWM, SW_CCW, SW_CW, sw);
    setMotorPower(SE_PWM, SE_CCW, SE_CW, se);
}

void setMotorPower(int pwm, int cw, int ccw, float percent) {
    if (percent < 0) {
        digitalWrite(cw, LOW);
        digitalWrite(ccw, HIGH);
    } else if (percent > 0) {
        digitalWrite(cw, HIGH);
        digitalWrite(ccw, LOW);
    } else {
        digitalWrite(cw, LOW);
        digitalWrite(ccw, LOW);
    }
    analogWrite(pwm, (int)(200 * constrain(abs(percent), 0, 1)));
}

void controlLA(int direction) {
    if (direction > 0) {
        digitalWrite(DIR1_LA, HIGH);
        digitalWrite(DIR2_LA, LOW);
        digitalWrite(ENA1, HIGH);
    } else if (direction < 0) {
        digitalWrite(DIR1_LA, LOW);
        digitalWrite(DIR2_LA, HIGH);
        digitalWrite(ENA1, HIGH);
    } else {
        digitalWrite(DIR1_LA, LOW);
        digitalWrite(DIR2_LA, LOW);
        digitalWrite(ENA1, LOW);
    }
}

void controlLA2(int direction) {
    if (direction > 0) {
        digitalWrite(DIR1_LA2, HIGH);
        digitalWrite(DIR2_LA2, LOW);
        digitalWrite(ENA2, HIGH);
    } else if (direction < 0) {
        digitalWrite(DIR1_LA2, LOW);
        digitalWrite(DIR2_LA2, HIGH);
        digitalWrite(ENA2, HIGH);
    } else {
        digitalWrite(DIR1_LA2, LOW);
        digitalWrite(DIR2_LA2, LOW);
        digitalWrite(ENA2, LOW);
    }
}
