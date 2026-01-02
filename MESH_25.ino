
//This is my base code for a bot without oled and this may need some tweaks or adjustments to fit your bot

#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

#define Twhite 150  // Threshold for white surface detection
#define Tblack 400  // Threshold for black line detection
#define NUM_SENSORS 16  // Number of sensors used in the QTR array

// PID calculation variables
int error_dir = 0;
int last_error = 0;  // Last error value for PID calculation
int I = 0;           // PID terms
float Kp = 0.05, Ki = 0.0005, Kd = 0.3;
int center_position = 7500;
uint8_t MAX_SPEED_R = 255, MAX_SPEED_L = 255;
uint8_t BASE_SPEED_R = 220, BASE_SPEED_L = 220;
#define TURN_SPEED 150

// Create QTR object for IR
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// Motor driver pin definitions (TB6612FNG)
#define LIN1 2
#define LIN2 3
#define LPWM 1

#define RIN1 5
#define RIN2 6
#define RPWM 7
Motor motor_r = Motor(RIN1, RIN2, RPWM, -1, 99);
Motor motor_l = Motor(LIN1, LIN2, LPWM, -1, 99);
#define LED_L 0
#define LED_R 33

// Button definitions
#define RSLB 12
#define LSRB 10
bool ABUT = false;
bool FBUT = false;
bool SBUT = false;

// Path variables
bool LP = false;
bool RP = false;
bool FP = false;
char Turn;
char turn;
char path[100];
int pathLength = 0;

// Function prototypes
void setMotor(int set_speed_r, int set_speed_l);
void moveForward();
void turnLeft();
void turnRight();
void uTurn();
void stopMotors();
void recordPath(char move);
void optimizePath();
void pid_control();
void mspeed(int posa, int posb);
void finalRun();

void setup() {
    Serial.begin(9600);
    
    // Initialize QTR sensor array
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){23,22,21,20,19,18,17,16,15,14,32,41,40,39,38,37}, NUM_SENSORS);

    pinMode(LED_L, OUTPUT);
    pinMode(LED_R, OUTPUT);
    
    // Setup Motor pins
    pinMode(RIN1, OUTPUT);
    pinMode(RIN2, OUTPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(LIN1, OUTPUT);
    pinMode(LIN2, OUTPUT);
    pinMode(LPWM, OUTPUT);

    // Initialize motors but stop initially
    motor_r.brake();
    motor_l.brake();
    delay(2);

    // Setup Buttons
    pinMode(RSLB, INPUT);
    pinMode(LSRB, INPUT);

    // Wait for the first button press to start calibration
    while (true) {
        if (digitalRead(RSLB) == HIGH) {
            Serial.println("First butt..");
            delay(600);
            break;
        }
    }

    // Calibrate IR sensors
    for (int i = 0; i < 100; i++) {
        if (i < 50) {
            setMotor(-100, 100);
        } else {
            setMotor(100, -100);
        }
        Serial.print("calibrating sensors");
        qtr.calibrate();
        delay(20);
    }
    motor_r.brake();
    motor_l.brake();
    delay(2);

    // Wait for the second button press to start the main loop
    while (true) {
        Serial.println("inside while");
        if (digitalRead(LSRB) == HIGH) {
            Serial.println("After First butt..");
            delay(600);
            ABUT = true;
            break;
        }
        if (digitalRead(RSLB) == HIGH) {
            delay(600);
            Serial.println("After 2nd butt..");
            ABUT = false;
            break;
        }
    }
}

void loop() {
  moveForward();
 // pid_control();
    // while (1) {
    //     FP = false;
    //     RP = false;
    //     LP = false;
    //     pid_control();
    //     qtr.readLineWhite(sensorValues);
        
    //     // Check for left turn intersection || sensorValues[1] < Twhite || sensorValues[2] < Twhite
    //     if (sensorValues[0] < Twhite ) {
    //         LP = true;
    //         Turn = 'L';
    //         digitalWrite(LED_L, HIGH);
            
    //         // Check for right turn intersection as well   || sensorValues[14] < Twhite || sensorValues[13] < Twhite
    //         for (int i = 0; i < 10; i++) {
    //             qtr.readLineWhite(sensorValues);
    //             if (sensorValues[15] < Twhite ) {
    //                 RP = true;
    //                 Turn = 'R';
    //                 digitalWrite(LED_R, HIGH);
    //                 break;
    //             }
    //         }
    //         break;
    //     } else {
    //         digitalWrite(LED_L, LOW);
    //     }
        
    //     // Check for right turn intersection  || sensorValues[14] < Twhite || sensorValues[13] < Twhite
    //     if (sensorValues[15] < Twhite ) {
    //         RP = true;
    //         Turn = 'R';
    //         digitalWrite(LED_R, HIGH);
            
    //         // Check for left turn intersection as well  || sensorValues[1] < Twhite || sensorValues[2] < Twhite
    //         for (int i = 0; i < 10; i++) {
    //             qtr.readLineWhite(sensorValues);
    //             if (sensorValues[0] < Twhite ) {
    //                 LP = true;
    //                 Turn = 'L';
    //                 digitalWrite(LED_L, HIGH);
    //                 break;
    //             }
    //         }
    //         break;
    //     } else {
    //         digitalWrite(LED_R, LOW);
    //     }
        
    //     // Check for U-turn (all sensors off the line)
    //     if (sensorValues[0] > Tblack && sensorValues[1] > Tblack && sensorValues[2] > Tblack && sensorValues[3] > Tblack && sensorValues[4] > Tblack && sensorValues[5] > Tblack && sensorValues[6] > Tblack && sensorValues[7] > Tblack && sensorValues[8] > Tblack && sensorValues[9] > Tblack && sensorValues[10] > Tblack && sensorValues[11] > Tblack && sensorValues[12] > Tblack && sensorValues[13] > Tblack && sensorValues[14] > Tblack && sensorValues[15] > Tblack) {
    //         uTurn();
    //         recordPath('U');
    //         optimizePath();
    //     }
    // }
    
    // stopMotors();
    // delay(1000);
    // moveForward();
    // stopMotors();
    // delay(1000);
    // qtr.readLineWhite(sensorValues);
    
    // // Final run condition check (all sensors read Twhite)
    // if (sensorValues[0] < Twhite && sensorValues[1] < Twhite && sensorValues[2] < Twhite && sensorValues[3] < Twhite && sensorValues[4] < Twhite && sensorValues[5] < Twhite && sensorValues[6] < Twhite && sensorValues[7] < Twhite && sensorValues[8] < Twhite && sensorValues[9] < Twhite && sensorValues[10] < Twhite && sensorValues[11] < Twhite && sensorValues[12] < Twhite && sensorValues[13] < Twhite && sensorValues[14] < Twhite && sensorValues[15] < Twhite) {
    //     while (true) {
    //         if (digitalRead(RSLB) == HIGH) {
    //             delay(600);
    //             break;
    //         }
    //     }
    //     finalRun();
    // }
    
    //   qtr.readLineWhite(sensorValues);

    // // Check for straight path
    // if (sensorValues[5] > Tblack && sensorValues[6] > Tblack && sensorValues[7] > Tblack && sensorValues[8] > Tblack && sensorValues[9] > Tblack && sensorValues[10] > Tblack && sensorValues[11] > Tblack && sensorValues[12] > Tblack ) {
    //     FP = false;
    // } else {
    //     FP = true;
    // }
    
    // // Navigation logic based on button state (ABUT)
    // if (ABUT) {
    //     if (LP) {
    //         turnLeft();
    //         recordPath('L');
    //         optimizePath();
    //     } else if (FP) {
    //         //moveForward();
    //         recordPath('S');
    //         optimizePath();
    //     } else if (RP) {
    //         turnRight();
    //         recordPath('R');
    //         optimizePath();
    //     }
    // } else {
    //     if (RP) {
    //         turnRight();
    //         recordPath('R');
    //         optimizePath();
    //     } else if (FP) {
    //         //moveForward();
    //         recordPath('S');
    //         optimizePath();
    //     } else if (LP) {
    //         turnLeft();
    //         recordPath('L');
    //         optimizePath();
    //     }
    // }
    // //delay(1000);
    // digitalWrite(LED_L, LOW);
    // digitalWrite(LED_R, LOW);
}

// Function to read the position of the line using QTR sensor array
int readLine() {
    int position = qtr.readLineWhite(sensorValues);
    return position;
}

// Move forward
void moveForward() {
    setMotor(120, 120);
    delay(200);
}

// Turn left
void turnLeft() {
    while (sensorValues[9] < Twhite || sensorValues[8] < Twhite) {
        qtr.readLineWhite(sensorValues);
        setMotor(120, -120);
    }
    while (sensorValues[9] > Tblack || sensorValues[8] > Tblack) {
        qtr.readLineWhite(sensorValues);
        setMotor(120, -120);
    }
    // while (sensorValues[0] < Twhite || sensorValues[15] < Twhite) {
    //     qtr.readLineWhite(sensorValues);
    //     setMotor(90, -90);
    // }
}

// Turn right
void turnRight() {
    while (sensorValues[9] < Twhite || sensorValues[8] < Twhite) {
        qtr.readLineWhite(sensorValues);
        setMotor(-120, 120);
    }
    while (sensorValues[9] > Tblack || sensorValues[9] > Tblack) {
        qtr.readLineWhite(sensorValues);
        setMotor(-120, 120);
    }
    // while (sensorValues[0] != 1000 || sensorValues[15] != 1000) {
    //     qtr.readLineWhite(sensorValues);
    //     setMotor(-90, 90);
    // }
}

// U-turn
void uTurn() {
    while (sensorValues[7] > Tblack || sensorValues[8] > Tblack) {
        qtr.readLineWhite(sensorValues);
        setMotor(-120, 120);
    }
    stopMotors();
}

// Stop motors
void stopMotors() {
    motor_r.brake();
    motor_l.brake();
    delay(10);
}

// Record the movement in the path
void recordPath(char move) {
    if (pathLength < 100) {
        path[pathLength++] = move;
    }
}

// Optimize the path
void optimizePath() {
    if (pathLength < 3 || path[pathLength - 2] != 'U') {
        return;
    }
    
    int total_angle = 0;
    
    for (int m = 1; m <= 3; m++) {
        switch (path[pathLength - m]) {
            case 'R':
                total_angle += 90;
                break;
            case 'L':
                total_angle += 270;
                break;
            case 'U':
                total_angle += 180;
                break;
        }
    }
    
    total_angle %= 360;
    
    switch (total_angle) {
        case 0:
            path[pathLength - 3] = 'S';
            break;
        case 90:
            path[pathLength - 3] = 'R';
            break;
        case 180:
            path[pathLength - 3] = 'U';
            break;
        case 270:
            path[pathLength - 3] = 'L';
            break;
    }
    pathLength -= 2;
}

// PID control function
void pid_control() {
    uint16_t position = qtr.readLineWhite(sensorValues);

    // FIX 1: Invert the error calculation so the signs match the motor logic
    // Range: -7500 (Left) to +7500 (Right)
    int error = position - center_position; 

    // P Term
    int P = error;

    // D Term
    int D = error - last_error;
    last_error = error;

    // I Term (Optional: Disabled for now to ensure stability)
    // Only enable I if the robot oscillates around the center but never quite settles
    // I = I + error;
    // int I_term = I * Ki; 

    // PID Calculation
    // Note: Removed I for initial tuning
    float motorAdjustment = (P * Kp) + (D * Kd); 

    // Calculate motor speeds
    int rightMotorSpeed = BASE_SPEED_R - motorAdjustment;
    int leftMotorSpeed  = BASE_SPEED_L + motorAdjustment;

    // FIX 2: Allow negative speeds for sharp turns (Pivot turning)
    // The SparkFun library handles negative numbers by reversing the motor direction
    rightMotorSpeed = constrain(rightMotorSpeed, -MAX_SPEED_R, MAX_SPEED_R);
    leftMotorSpeed  = constrain(leftMotorSpeed,  -MAX_SPEED_L, MAX_SPEED_L);

    // Set motor speeds
    mspeed(rightMotorSpeed, leftMotorSpeed);
}

// Sets the speed for both motors using the motor objects
void mspeed(int posa, int posb) {
    motor_r.drive(posa);
    motor_l.drive(posb);
}

// Sets the speed for both motors (alternative)
void setMotor(int set_speed_r, int set_speed_l) {
    motor_r.drive(set_speed_r);
    motor_l.drive(set_speed_l);
}

// Final run function to follow the optimized path
void finalRun() {
    int i = 0;
    while (i < pathLength) {
        while (true) {
            qtr.readLineWhite(sensorValues);
            pid_control();
            if (sensorValues[0] < Twhite || sensorValues[1] < Twhite || sensorValues[2] < Twhite || sensorValues[13] < Twhite || sensorValues[14] < Twhite || sensorValues[15] < Twhite) {
                break;
            }
        }
        stopMotors();
        setMotor(100, 100);
        delay(75);

        if (path[i] == 'L') {
            turnLeft();
        } else if (path[i] == 'S') {
            moveForward();
        } else if (path[i] == 'R') {
            turnRight();
        } else if (path[i] == 'U') {
            uTurn();
        }

        i++;
    }
    
    motor_r.brake();
    motor_l.brake();
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_L, HIGH);
    delay(50000);
}
