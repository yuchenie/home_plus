#include <EEPROM.h> 
#include <Servo.h>
#include <Encoder.h>
#include <Math.h>

float vx, vy, omega;
int32_t target_x, target_y, target_theta;
int32_t x, y, theta;
float dx, dy, dtheta;
float threshold = 100;

float kP = 0.002;

float wheel_radius = 0.096; // 96 mm
float base_radius = 0.5 * (0.264+0.072*2) * sqrt(2); // square base with sides 264mm
float wheel_circumference = PI * 0.096; // wheels are 96mm diameter
float resolution = 2786.2;

#define MAX_COMMANDS 20
String commandQueue[MAX_COMMANDS];
int commandCount = 0;
int currentCommandIndex = 0;

// === Servo Setup ===
Servo hand, wrist, elbow, shoulder;
int hand_pos, wrist_pos, elbow_pos, shoulder_pos;

// === Motor Pins ===
#define NW_PWM 6 
#define NW_CW 49
#define NW_CCW 48
Encoder NW_ENCODER(21, 23);

#define NE_PWM 5 
#define NE_CW 50
#define NE_CCW 51
Encoder NE_ENCODER(20, 22);

#define SW_PWM 4 
#define SW_CW 53
#define SW_CCW 52
Encoder SW_ENCODER(2, 15);

#define SE_PWM 7
#define SE_CW 46
#define SE_CCW 47
Encoder SE_ENCODER(3, 14);

#define LA_DOWN 44
#define LA_UP 45
#define LA_PWM 41

#define FRAME_UP 42
#define FRAME_DOWN 43
#define FRAME_PWM 40

void setup() {
    Serial.begin(9600);

    hand.attach(13);  
    wrist.attach(12);
    elbow.attach(11);
    shoulder.attach(10);

    hand_pos = 1000;
    wrist_pos = 900;
    elbow_pos = 1600;
    shoulder_pos = 2100;

    hand.write(hand_pos);
    wrist.write(wrist_pos);
    elbow.write(elbow_pos);
    shoulder.write(shoulder_pos);

    Serial.println("USB Serial Ready");
    // printServoAngles();
}

void loop() {
    static String lastCommand = "0 0 0";

    if (Serial.available() > 0) {
        // lastCommand = "";
        String fullInput = "";
        while (Serial.available() > 0) {
            char c = Serial.read();
            // if (c == '\n') break;
            // lastCommand += c;
            fullInput += c;
            delay(1);
        }
        parseCommandQueue(fullInput);
    }

    read();
    // parseMessage(lastCommand);

    dx = target_x - x;
    dy = target_y - y;
    dtheta = target_theta - theta;
    
    if (abs(dx) > threshold) { vx = kP*dx; } else { vx = 0; }
    if (abs(dy) > threshold) { vy = kP*dy; } else { vy = 0; }
    if (abs(dtheta) > threshold) { omega = kP*dtheta; } else { omega = 0; }

    // reset when goal is reached
    if (vx == 0  && vy == 0 && omega == 0) {
        NW_ENCODER.write(0);
        NE_ENCODER.write(0);
        SW_ENCODER.write(0);
        SE_ENCODER.write(0);

        target_x = 0;
        target_y = 0;
        target_theta = 0;

        // lastCommand = "0 0 0";
        if (currentCommandIndex < commandCount) {
            parseMessage(commandQueue[currentCommandIndex]);
            currentCommandIndex++;
        }
    }

    drive(vx, vy, omega); 
    // Serial.println(String(target_x) + " " + String(target_y) + " " + String(target_theta));

    /*
    if (lastCommand.length() == 1) {
        char cmd = lastCommand.charAt(0);
        switch (cmd) {
            case ' ': drive(0, 0, 0); break;
        }
    } else {
        parseMessage(lastCommand);
        drive(vx, vy, omega);
        // int time = 2000 * (abs(x)+abs(y)+abs(theta));
        // delay(time);
        // drive(0, 0, 0);
    }
    // lastCommand = " ";
    */
}

void read() {
    // positive counterclockwise
    int32_t nw = NW_ENCODER.read();
    int32_t ne = NE_ENCODER.read();
    int32_t sw = SW_ENCODER.read();
    int32_t se = SE_ENCODER.read();

    theta = (+ nw + ne + sw + se) / 4.0;
    x = (+ nw - ne + sw - se) / 4.0;
    y = (+ nw + ne - sw - se) / 4.0;
}

void parseMessage(String input) {
    Serial.println(input);
    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);

    // velocity control
    // vx = input.substring(0, firstSpace).toFloat();
    // vy = input.substring(firstSpace + 1, secondSpace).toFloat();
    // omega = input.substring(secondSpace + 1).toFloat();

    float x_ = input.substring(0, firstSpace).toFloat();
    float y_ = input.substring(firstSpace + 1, secondSpace).toFloat();
    float theta_ = input.substring(secondSpace + 1).toFloat();

    // convert meters / degrees to encoder pulses
    target_x = (x_ / wheel_circumference) * resolution;
    target_y = (y_ / wheel_circumference) * resolution;
    target_theta = (((PI / 180) * base_radius * theta_) / wheel_circumference) * resolution;
}

void parseCommandQueue(String input) {
    commandCount = 0;
    currentCommandIndex = 0;
    int startIdx = 0;

    while (startIdx < input.length() && commandCount < MAX_COMMANDS) {
        int endIdx = input.indexOf('n', startIdx);
        if (endIdx == -1) endIdx = input.length();

        String command = input.substring(startIdx, endIdx);
        command.trim();

        if (command.length() > 0) {
            commandQueue[commandCount++] = command;
        }

        startIdx = endIdx + 1;
    }

    if (commandCount > 0) {
        parseMessage(commandQueue[currentCommandIndex]);
        currentCommandIndex++;
    }
}

int32_t sgn(int32_t val) {
    return val / abs(val);
}

void drive(float x, float y, float theta) {
    float nw = (-y - x - theta);
    float ne = (-y + x - theta);
    float sw = (-y + x + theta);
    float se = (-y - x + theta);

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
        digitalWrite(LA_UP, HIGH);
        digitalWrite(LA_DOWN, LOW);
        digitalWrite(LA_PWM, HIGH);
    } else if (direction < 0) {
        digitalWrite(LA_UP, LOW);
        digitalWrite(LA_DOWN, HIGH);
        digitalWrite(LA_PWM, HIGH);
    } else {
        digitalWrite(LA_UP, LOW);
        digitalWrite(LA_DOWN, LOW);
        digitalWrite(LA_PWM, LOW);
    }
}

void controlFrame(int direction) {
    if (direction > 0) {
        digitalWrite(FRAME_UP, HIGH);
        digitalWrite(FRAME_DOWN, LOW);
        digitalWrite(FRAME_PWM, HIGH);
    } else if (direction < 0) {
        digitalWrite(FRAME_UP, LOW);
        digitalWrite(FRAME_DOWN, HIGH);
        digitalWrite(FRAME_PWM, HIGH);
    } else {
        digitalWrite(FRAME_UP, LOW);
        digitalWrite(FRAME_DOWN, LOW);
        digitalWrite(FRAME_PWM, LOW);
    }
}