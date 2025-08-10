#include <EEPROM.h> 
#include <Servo.h>
#include <Encoder.h>
#include <Math.h>
#include <PID_v1.h>

unsigned long current_millis = 0;
unsigned long prev_millis = 0;

double vx, vy, omega;
double vx_, vy_, omega_;
double target_x, target_y, target_theta;
double x, y, theta;
double dx, dy, dtheta;
int drive_threshold = 200;

float kP = 0.0015;
float kI = 0.0;
float kD = 0.0;
PID xPID(&x, &vx, &target_x, kP, kI, kD, DIRECT);
PID yPID(&y, &vy, &target_y, kP, kI, kD, DIRECT);
PID thetaPID(&theta, &omega, &target_theta, kP, kI, kD, DIRECT);

float wheel_radius = 0.096; // 96 mm
float base_radius = 0.5 * (0.264+0.072*2) * sqrt(2); // square base with sides 264mm
float wheel_circumference = PI * 0.096; // wheels are 96mm diameter
float resolution = 2786.2;

String serialBuffer = "";
const int MAX_COMMANDS = 20;
String commandQueue[MAX_COMMANDS];
int commandCount = 0;
int currentCommandIndex = 0;

// === Servo Setup ===
Servo grip, hand, wrist, elbow, shoulder;
float grip_target, hand_target, wrist_target, elbow_target, shoulder_target;
float grip_pos, hand_pos, wrist_pos, elbow_pos, shoulder_pos;
float dgrip, dhand, dwrist, delbow, dshoulder;
float servo_threshold = 10.0;

// === Drive Setup ===
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

// === Linear Actuator Setup ===
float la_target, frame_target;
float la_pos, frame_pos;
float dla, dframe;
float vla, vframe;
float la_threshold = 1.0;

#define LA_DOWN 44
#define LA_UP 45
#define LA_PWM 41 // 12mm/s, max 450mm

#define FRAME_UP 42
#define FRAME_DOWN 43
#define FRAME_PWM 40 // 12mm/s

void setup() {
    Serial.begin(9600);

    grip.attach(9);
    hand.attach(13);  
    wrist.attach(12);
    elbow.attach(11);
    shoulder.attach(10);

    grip_target = 2100;
    hand_target = 1000;
    wrist_target = 900;
    elbow_target = 1500;
    shoulder_target = 2100;

    grip_pos = 2100;
    hand_pos = 1000;
    wrist_pos = 900;
    elbow_pos = 1500;
    shoulder_pos = 2100;

    grip.write(grip_target);
    hand.write(hand_target);
    wrist.write(wrist_target);
    elbow.write(elbow_target);
    shoulder.write(shoulder_target);

    la_target = 450;
    la_pos = 450; // assuming it starts at top
    // la_pos = 0; // assuming worst case

    frame_target = 0;
    frame_pos = 0; // assuming it starts at bottom
    // frame_pos = 100; // assuming worst case

    Serial.println("USB Serial Ready");
}

void loop() {
    prev_millis = current_millis;
    current_millis = millis();

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n') {
            // End of message received
            parseCommandQueue(serialBuffer);
            serialBuffer = "";  // clear for next message
        } else {
            serialBuffer += c;
        }
    }

    update_pos();
    
    if (abs(dx) > drive_threshold) { vx_ = vx; } else { vx_ = 0; }
    if (abs(dy) > drive_threshold) { vy_ = vy; } else { vy_ = 0; }
    if (abs(dtheta) > drive_threshold) { omega_ = omega; } else { omega_ = 0; }

    // reset when goal is reached
    if (status()) {
        // NW_ENCODER.write(0);
        // NE_ENCODER.write(0);
        // SW_ENCODER.write(0);
        // SE_ENCODER.write(0);

        // target_x = 0;
        // target_y = 0;
        // target_theta = 0;
        
        if (currentCommandIndex < commandCount) {
            parseMessage(commandQueue[currentCommandIndex]);
            currentCommandIndex++;
        }
    }

    drive(vx_, vy_, omega_); 
    grip.write(grip_target);
    hand.write(hand_target);
    wrist.write(wrist_target);
    elbow.write(elbow_target);
    shoulder.write(shoulder_target);
    controlLA(vla);
    controlFrame(vframe);

    // Serial.println(String(la_pos) + " " + String(frame_pos));
}

bool status() {
    return ((abs(dx) < drive_threshold) && 
            (abs(dy) < drive_threshold) && 
            (abs(dtheta) < drive_threshold) && 
            (abs(dgrip) < servo_threshold) && 
            (abs(dhand) < servo_threshold) && 
            (abs(dwrist) < servo_threshold) && 
            (abs(delbow) < servo_threshold) && 
            (abs(dshoulder) < servo_threshold) &&
            (abs(dla) < la_threshold) && 
            (abs(dframe) < la_threshold));
}

void update_pos() {
    float dt = current_millis - prev_millis;

    // === Drive ===
    // positive counterclockwise
    int32_t nw = NW_ENCODER.read();
    int32_t ne = NE_ENCODER.read();
    int32_t sw = SW_ENCODER.read();
    int32_t se = SE_ENCODER.read();

    theta = (+ nw + ne + sw + se) / 4.0;
    x = ((+ nw - ne + sw - se) / 4.0) * sqrt(2);
    y = ((+ nw + ne - sw - se) / 4.0) * sqrt(2);

    dx = target_x - x;
    dy = target_y - y;
    dtheta = target_theta - theta;

    // === Servos ===
    // 40 RPM no stall, maps to 0.2 microseconds per millisecond
    dgrip = grip_target - grip_pos;
    dhand = hand_target - hand_pos;
    dwrist = wrist_target - wrist_pos;
    delbow = elbow_target - elbow_pos;
    dshoulder = shoulder_target - shoulder_pos;

    if (abs(dgrip) > servo_threshold) { grip_pos += (sgn(dgrip) * dt * 0.2); }
    if (abs(dhand) > servo_threshold) { hand_pos += (sgn(dhand) * dt * 0.2); }
    if (abs(dwrist) > servo_threshold) { wrist_pos += (sgn(dwrist) * dt * 0.2); }
    if (abs(delbow) > servo_threshold) { elbow_pos += (sgn(delbow) * dt * 0.2); }
    if (abs(dshoulder) > servo_threshold) { shoulder_pos += (sgn(dshoulder) * dt * 0.7*0.2); } // seems like worst case is ~0.7x speed

    // === Linear Actuators ===
    dla = la_target - la_pos;
    dframe = frame_target - frame_pos;

    vla = (abs(dla) > la_threshold) ? sgn(dla) : 0;
    vframe = (abs(dframe) > la_threshold) ? sgn(dframe) : 0;

    if (abs(dla) > la_threshold) { la_pos += (sgn(dla) * dt * 0.012); }
    if (abs(dframe) > la_threshold) { frame_pos += (sgn(dframe) * dt * 0.012); }
}

int splitString(String input, char delimiter, String* tokens, int maxTokens) {
    int tokenCount = 0;
    int start = 0;
    int end = input.indexOf(delimiter);

    while (end != -1 && tokenCount < maxTokens) {
        tokens[tokenCount++] = input.substring(start, end);
        start = end + 1;
        end = input.indexOf(delimiter, start);
    }

    // Add the last token
    if (tokenCount < maxTokens) {
        tokens[tokenCount++] = input.substring(start);
    }

    return tokenCount;
}

void parseMessage(String input) {
    Serial.println(input);

    const int maxTokens = 10;
    String tokens[maxTokens];
    int tokenCount = splitString(input, ' ', tokens, maxTokens);

    if (tokenCount != 10) {
        Serial.println("Error: Incorrect number of inputs.");
        return;
    }

    float x_ = 0.01 * tokens[0].toFloat();
    float y_ = 0.01 * tokens[1].toFloat();
    float theta_ = tokens[2].toFloat();
    float grip_ = tokens[3].toFloat();
    float hand_ = tokens[4].toFloat();
    float wrist_ = tokens[5].toFloat();
    float elbow_ = tokens[6].toFloat();
    float shoulder_ = tokens[7].toFloat();
    float la_ = tokens[8].toFloat();
    float frame_ = tokens[9].toFloat();

    // convert meters / degrees to encoder pulses
    target_x = (x_ / wheel_circumference) * resolution;
    target_y = (y_ / wheel_circumference) * resolution;
    target_theta = (((PI / 180) * base_radius * theta_) / wheel_circumference) * resolution;
    grip_target = map(grip_, 0, 100, 2100, 1000);
    hand_target = map(hand_, 0, 100, 920, 1110);
    wrist_target = map(wrist_, 0, 100, 900, 2400);
    elbow_target = map(elbow_, 0, 100, 2100, 900);
    shoulder_target = map(shoulder_, 0, 100, 2100, 900);
    la_target = constrain(la_, 0, 450);
    frame_target = constrain(frame_, 0, 175);
}

void parseCommandQueue(String input) {
    Serial.println(input);
    commandCount = 0;
    currentCommandIndex = 0;
    int startIdx = 0;

    while (startIdx < input.length() && commandCount < MAX_COMMANDS) {
        int endIdx = input.indexOf(',', startIdx);
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

float sgn(int32_t x) {
    return (x > 0) - (x < 0);  // 1 for positive, -1 for negative, and 0 for 0
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
    bool dir = (percent > 0);
    digitalWrite(cw, dir);
    digitalWrite(ccw, !dir);
    analogWrite(pwm, (int)(200 * constrain(abs(percent), 0, 1)));
}

void controlLA(int direction) {
    bool dir = (direction > 0);
    digitalWrite(LA_UP, dir);
    digitalWrite(LA_DOWN, !dir);

    bool en = (direction != 0);
    digitalWrite(LA_PWM, en);
}

void controlFrame(int direction) {
    bool dir = (direction > 0);
    digitalWrite(FRAME_UP, dir);
    digitalWrite(FRAME_DOWN, !dir);

    bool en = (direction != 0);
    digitalWrite(FRAME_PWM, en);
}