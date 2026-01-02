#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <Fonts/FreeSansBold9pt7b.h> 

// =========================================
// 1. HARDWARE & DISPLAY CONFIGURATION
// ==========================================

// OLED Display Settings 
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
#define OLED_RESET -1    
#define i2c_Address 0x3C 
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire2, OLED_RESET);

// Sensor Thresholds 
#define Twhite 250  
#define Tblack 400  
#define NUM_SENSORS 16  

// QTR Sensor Object
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// Motor Pins
#define LIN1 2
#define LIN2 3
#define LPWM 1
#define RIN1 5
#define RIN2 6
#define RPWM 7

// Motor Objects
Motor motor_r = Motor(RIN1, RIN2, RPWM, -1, 99);
Motor motor_l = Motor(LIN1, LIN2, LPWM, -1, 99);
#define LED_L 0
#define LED_R 33

// --- BUTTONS ---
#define BTN_NEXT   10 // LSRB (Used as LEFT in Snake)
#define BTN_SELECT 12 // RSLB (Used as RIGHT in Snake)
// NEW BUTTONS
const int UP_BUTTON_PIN = 11;   // Used as UP
const int DOWN_BUTTON_PIN = 9;  // Used as DOWN

// ==========================================
// 2. PID & MOTION VARIABLES
// ==========================================

int last_error = 0;
// PID Constants 
float Kp = 0.084, Ki = 0.0001, Kd = 0.7; 
int center_position = 7500;

// Speed Constants
uint8_t MAX_SPEED_R = 250, MAX_SPEED_L = 250;
uint8_t BASE_SPEED_R = 220, BASE_SPEED_L = 220;
#define TURN_SPEED 120 

// ==========================================
// 3. MAZE SOLVING & TIMING VARIABLES
// ==========================================

char path[100];
int pathLength = 0;
bool ABUT = false; 
bool FP,RP,LP;
char Turn;

// --- TIMER VARIABLES ---
unsigned long startTime = 0;
unsigned long exploreTime = 0;
unsigned long finalRunTime = 0;

// FSM States
enum State {
    CALIBRATING,
    MENU_SELECTION,
    EXPLORING_SUBMENU,
    LINEFOLLOW_MODE,
    EXPLORING_MODE,
    FINAL_RUN_PENDING,
    FINAL_RUN,
    FINISHED,
    // --- NEW GAME STATES ---
    GAME_MENU_STATE,
    PLAY_SNAKE,
    PLAY_FLAPPY
};
State currentState = CALIBRATING;

// Menu States
enum MainMenuState { OPT_EXPLORE, OPT_LINEFOLLOW, OPT_GAMES }; // Added OPT_GAMES
MainMenuState currentMenu = OPT_EXPLORE;

enum SubMenuState { OPT_LEFT_FIRST, OPT_RIGHT_FIRST };
SubMenuState currentSubMenu = OPT_LEFT_FIRST;

// Game Menu States
enum GameMenuOption { GAME_OPT_SNAKE, GAME_OPT_FLAPPY };
GameMenuOption currentGameOption = GAME_OPT_SNAKE;

// ==========================================
// 4. FUNCTION PROTOTYPES
// ==========================================
void handleCalibration();
void handleMainMenu();
void handleSubMenu();
void handleExploring();
void handleFinalRun();
void pid_control();
void mspeed(int posa, int posb);
void setMotor(int r, int l);
void moveForward();
void turnLeft();
void turnRight();
void uTurn();
void stopMotors();
void recordPath(char move);
void optimizePath();
bool checkEndOfMaze();
void breakMotor();
void handleFinalRunPending();
void handleFinished();

// --- NEW GAME FUNCTIONS ---
void handleGameMenu();
void playSnake();
void playFlappyBird();

// Helper to draw centered text 
void drawCentered(String text, int y) {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - w) / 2, y);
    display.print(text);
}

// ==========================================
// 5. SETUP
// ==========================================
void setup() {
    Serial.begin(9600);

    if (!display.begin(i2c_Address, true)) { 
        Serial.println(F("SH1106 allocation failed"));
        for (;;);
    }
    
    display.clearDisplay();
    display.setFont(&FreeSansBold9pt7b); 
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);
    drawCentered("Meshmerize", 25);
    display.setFont(NULL); 
    drawCentered("Initializing...", 50);
    display.display();

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){23,22,21,20,19,18,17,16,15,14,32,41,40,39,38,37}, NUM_SENSORS);

    pinMode(LED_L, OUTPUT);
    pinMode(LED_R, OUTPUT);
    
    pinMode(RIN1, OUTPUT); pinMode(RIN2, OUTPUT); pinMode(RPWM, OUTPUT);
    pinMode(LIN1, OUTPUT); pinMode(LIN2, OUTPUT); pinMode(LPWM, OUTPUT);
    motor_r.brake(); motor_l.brake();

    // Existing Buttons
    pinMode(BTN_NEXT, INPUT);   
    pinMode(BTN_SELECT, INPUT);
    
    // --- NEW BUTTONS ---
    pinMode(UP_BUTTON_PIN, INPUT);
    pinMode(DOWN_BUTTON_PIN, INPUT);

    display.clearDisplay();
    display.setFont(&FreeSansBold9pt7b);
    drawCentered("Ready!", 25);
    display.setFont(NULL);
    drawCentered("Press SELECT", 45);
    display.display();
}

// ==========================================
// 6. MAIN LOOP (FSM)
// ==========================================
void loop() {
    switch (currentState) {
        case CALIBRATING:       handleCalibration(); break;
        case MENU_SELECTION:    handleMainMenu(); break;
        case EXPLORING_SUBMENU: handleSubMenu(); break;
        case LINEFOLLOW_MODE:   pid_control(); break; 
        case EXPLORING_MODE:    handleExploring(); break; 
        case FINAL_RUN_PENDING: handleFinalRunPending(); break;
        case FINAL_RUN:         handleFinalRun(); break;
        case FINISHED:          handleFinished(); break;
        // --- NEW STATES ---
        case GAME_MENU_STATE:   handleGameMenu(); break;
        case PLAY_SNAKE:        playSnake(); break;
        case PLAY_FLAPPY:       playFlappyBird(); break;
    }
}

// ==========================================
// 7. STATE HANDLERS
// ==========================================

void handleMainMenu() {
    // Navigate (Cycle 3 options now)
    if (digitalRead(BTN_NEXT) == HIGH) { 
        delay(200);
        if (currentMenu == OPT_EXPLORE) currentMenu = OPT_LINEFOLLOW;
        else if (currentMenu == OPT_LINEFOLLOW) currentMenu = OPT_GAMES;
        else currentMenu = OPT_EXPLORE;
    }

    display.clearDisplay();
    display.setFont(&FreeSansBold9pt7b); 
    display.setTextColor(SH110X_WHITE);
    
    display.setCursor(0, 15);
    display.println("Mode:");

    display.setCursor(10, 35); 
    if (currentMenu == OPT_EXPLORE) display.print("> Explore");
    else display.print("  Explore");

    display.setCursor(10, 50);
    if (currentMenu == OPT_LINEFOLLOW) display.print("> Line Only"); 
    else display.print("  Line Only");
    
    display.setCursor(10, 63); // 3rd Option
    if (currentMenu == OPT_GAMES) display.print("> Games"); 
    else display.print("  Games");
    
    display.display();

    if (digitalRead(BTN_SELECT) == HIGH) {
        delay(200);
        if (currentMenu == OPT_EXPLORE) {
            currentState = EXPLORING_SUBMENU;
        } else if (currentMenu == OPT_LINEFOLLOW) {
            currentState = LINEFOLLOW_MODE;
            display.clearDisplay(); 
            display.setFont(&FreeSansBold9pt7b);
            drawCentered("Line Follow", 35);
            display.display();
            delay(200);
        } else {
            // Go to Game Menu
            currentState = GAME_MENU_STATE;
        }
    }
}

// --- NEW GAME MENU HANDLER ---
void handleGameMenu() {
    // Navigate Games
    if (digitalRead(BTN_NEXT) == HIGH) { 
        delay(200);
        currentGameOption = (currentGameOption == GAME_OPT_SNAKE) ? GAME_OPT_FLAPPY : GAME_OPT_SNAKE;
    }

    display.clearDisplay();
    display.setFont(&FreeSansBold9pt7b); 
    display.setTextColor(SH110X_WHITE);
    
    display.setCursor(0, 15);
    display.println("Select Game:");

    display.setCursor(10, 35); 
    if (currentGameOption == GAME_OPT_SNAKE) display.print("> Snake");
    else display.print("  Snake");

    display.setCursor(10, 53);
    if (currentGameOption == GAME_OPT_FLAPPY) display.print("> Flappy Bird"); 
    else display.print("  Flappy Bird");
    
    display.display();

    if (digitalRead(BTN_SELECT) == HIGH) {
        delay(200);
        if (currentGameOption == GAME_OPT_SNAKE) currentState = PLAY_SNAKE;
        else currentState = PLAY_FLAPPY;
        
        // Brief loading screen
        display.clearDisplay();
        drawCentered("Loading...", 35);
        display.display();
        delay(500);
    }
}

// ==========================================
// 8. GAME IMPLEMENTATIONS
// ==========================================

void playSnake() {
    // Snake Configuration
    const int BLOCK_SIZE = 4;
    int snakeX[60], snakeY[60];
    int snakeLen = 3;
    int dir = 1; // 0=Up, 1=Right, 2=Down, 3=Left
    int foodX = 0, foodY = 0;
    bool gameOver = false;
    
    // Init Snake
    snakeX[0] = 64; snakeY[0] = 32;
    // Init Food
    foodX = random(0, SCREEN_WIDTH/BLOCK_SIZE) * BLOCK_SIZE;
    foodY = random(0, SCREEN_HEIGHT/BLOCK_SIZE) * BLOCK_SIZE;

    while (!gameOver) {
        // 1. Input (4 Buttons)
        // BTN_NEXT is Pin 10 (Left), BTN_SELECT is Pin 12 (Right), UP=11, DOWN=9
        if (digitalRead(UP_BUTTON_PIN) == HIGH && dir != 2) dir = 0;
        else if (digitalRead(DOWN_BUTTON_PIN) == HIGH && dir != 0) dir = 2;
        else if (digitalRead(BTN_NEXT) == HIGH && dir != 1) dir = 3; // Left
        else if (digitalRead(BTN_SELECT) == HIGH && dir != 3) dir = 1; // Right
        
        // 2. Logic - Move Body
        for (int i = snakeLen - 1; i > 0; i--) {
            snakeX[i] = snakeX[i - 1];
            snakeY[i] = snakeY[i - 1];
        }
        
        // Move Head
        if (dir == 0) snakeY[0] -= BLOCK_SIZE;
        if (dir == 1) snakeX[0] += BLOCK_SIZE;
        if (dir == 2) snakeY[0] += BLOCK_SIZE;
        if (dir == 3) snakeX[0] -= BLOCK_SIZE;
        
        // 3. Collision - Walls
        if (snakeX[0] < 0 || snakeX[0] >= SCREEN_WIDTH || snakeY[0] < 0 || snakeY[0] >= SCREEN_HEIGHT) {
            gameOver = true;
        }
        // Collision - Self
        for (int i = 1; i < snakeLen; i++) {
            if (snakeX[0] == snakeX[i] && snakeY[0] == snakeY[i]) gameOver = true;
        }
        
        // 4. Food
        if (abs(snakeX[0] - foodX) < BLOCK_SIZE && abs(snakeY[0] - foodY) < BLOCK_SIZE) {
            snakeLen++;
            if(snakeLen >= 60) snakeLen = 60; // Max cap
            foodX = random(0, SCREEN_WIDTH/BLOCK_SIZE) * BLOCK_SIZE;
            foodY = random(0, SCREEN_HEIGHT/BLOCK_SIZE) * BLOCK_SIZE;
        }
        
        // 5. Draw
        display.clearDisplay();
        display.setFont(NULL); // System font
        
        // Draw Food
        display.fillRect(foodX, foodY, BLOCK_SIZE, BLOCK_SIZE, SH110X_WHITE);
        
        // Draw Snake
        for (int i = 0; i < snakeLen; i++) {
            display.fillRect(snakeX[i], snakeY[i], BLOCK_SIZE, BLOCK_SIZE, SH110X_WHITE);
        }
        
        display.display();
        delay(100); // Game Speed
    }
    
    // Game Over Screen
    display.clearDisplay();
    display.setFont(&FreeSansBold9pt7b);
    drawCentered("Game Over", 20);
    display.setFont(NULL);
    display.setCursor(40, 40);
    display.print("Score: ");
    display.print(snakeLen - 3);
    display.display();
    
    delay(2000);
    while(digitalRead(BTN_SELECT) == LOW); // Wait for click
    currentState = GAME_MENU_STATE; // Return to menu
}

void playFlappyBird() {
    float birdY = 32;
    float velocity = 0;
    float gravity = 0.6;
    float lift = -4.0; // Jump strength
    
    int wallX = SCREEN_WIDTH;
    int wallGapY = 32;
    int wallWidth = 10;
    int gapSize = 25;
    int score = 0;
    bool gameOver = false;

    while (!gameOver) {
        // 1. Input (Jump with UP button)
        if (digitalRead(UP_BUTTON_PIN) == HIGH) {
            velocity = lift;
        }
        
        // 2. Physics
        velocity += gravity;
        birdY += velocity;
        
        // Wall movement
        wallX -= 2;
        if (wallX < -wallWidth) {
            wallX = SCREEN_WIDTH;
            wallGapY = random(10, SCREEN_HEIGHT - 10 - gapSize);
            score++;
        }
        
        // 3. Collision
        // Floor/Ceiling
        if (birdY > SCREEN_HEIGHT || birdY < 0) gameOver = true;
        
        // Wall
        if (wallX < 14 && wallX + wallWidth > 4) { // Bird x is fixed approx at 10, width 4
            if (birdY < wallGapY || birdY > wallGapY + gapSize) {
                gameOver = true;
            }
        }
        
        // 4. Draw
        display.clearDisplay();
        
        // Bird
        display.fillRect(10, (int)birdY, 6, 6, SH110X_WHITE);
        
        // Walls
        // Top wall
        display.fillRect(wallX, 0, wallWidth, wallGapY, SH110X_WHITE);
        // Bottom wall
        display.fillRect(wallX, wallGapY + gapSize, wallWidth, SCREEN_HEIGHT - (wallGapY + gapSize), SH110X_WHITE);
        
        // Score
        display.setFont(NULL);
        display.setCursor(0,0);
        display.print(score);
        
        display.display();
        delay(20);
    }
    
    // Game Over Screen
    display.clearDisplay();
    display.setFont(&FreeSansBold9pt7b);
    drawCentered("Crashed!", 20);
    display.setFont(NULL);
    display.setCursor(40, 40);
    display.print("Score: ");
    display.print(score);
    display.display();
    
    delay(2000);
    while(digitalRead(BTN_SELECT) == LOW); // Wait for click
    currentState = GAME_MENU_STATE; // Return to menu
}

// ==========================================
// 9. EXISTING FUNCTIONS (UNCHANGED)
// ==========================================

void handleFinished() {
    stopMotors(); 
    display.clearDisplay();
    display.setFont(&FreeSansBold9pt7b);
    display.setTextSize(1);
    drawCentered("Final Time:", 15);
    display.setCursor(20, 40); 
    display.print(finalRunTime / 1000.0, 3); 
    display.print(" s");
    display.setTextSize(0); 
    drawCentered("SELECT to Re-Run", 60);
    display.display();

    if (digitalRead(BTN_SELECT) == HIGH) {
        delay(200); 
        display.clearDisplay();
        display.setFont(&FreeSansBold9pt7b);
        drawCentered("Get Ready...", 35);
        display.display();
        delay(1000); 
        startTime = millis();
        currentState = FINAL_RUN;
    }
}

void handleCalibration() {
    if (digitalRead(BTN_SELECT) == HIGH) {
        delay(500); 
        display.clearDisplay();
        display.setFont(&FreeSansBold9pt7b);
        drawCentered("Calibrating...", 35);
        display.display();
        for (int i = 0; i < 100; i++) {
            if (i < 50) setMotor(-100, 100);
            else setMotor(100, -100);
            qtr.calibrate();
            delay(20);
        }
        stopMotors();
        delay(500);
        currentState = MENU_SELECTION;
    }
}

void handleSubMenu() {
    if (digitalRead(BTN_NEXT) == HIGH) { 
        delay(200);
        currentSubMenu = (currentSubMenu == OPT_LEFT_FIRST) ? OPT_RIGHT_FIRST : OPT_LEFT_FIRST;
    }
    display.clearDisplay();
    display.setFont(&FreeSansBold9pt7b);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 15);
    display.println("Priority:");
    display.setCursor(10, 35);
    if (currentSubMenu == OPT_LEFT_FIRST) display.print("> Left First");
    else display.print("  Left First");
    display.setCursor(10, 53);
    if (currentSubMenu == OPT_RIGHT_FIRST) display.print("> Right First");
    else display.print("  Right First");
    display.display();

    if (digitalRead(BTN_SELECT) == HIGH) {
        delay(200);
        if (currentSubMenu == OPT_LEFT_FIRST) {
            ABUT = true; 
        } else if (currentSubMenu == OPT_RIGHT_FIRST) {
            ABUT = false; 
        }
        display.clearDisplay(); 
        display.setFont(&FreeSansBold9pt7b);
        drawCentered("Exploring...", 35); 
        display.display();
        delay(200);
        startTime = millis();
        currentState = EXPLORING_MODE;
    }
}

void handleFinalRunPending() {
    static bool screenDrawn = false;
    if (!screenDrawn) {
        display.clearDisplay();
        display.setFont(&FreeSansBold9pt7b);
        display.setTextColor(SH110X_WHITE);
        display.setTextSize(1);
        drawCentered("Explored!", 15);
        display.setCursor(20, 40); 
        display.print(exploreTime / 1000.0, 2); 
        display.print(" s");
        display.setTextSize(0); 
        drawCentered("SELECT to Run", 60);
        display.display();
        screenDrawn = true;
    }
    if(digitalRead(BTN_SELECT) == HIGH) {
        delay(200); 
        screenDrawn = false; 
        display.clearDisplay();
        display.setFont(&FreeSansBold9pt7b);
        drawCentered("Get Ready!", 35);
        display.display();
        delay(200); 
        startTime = millis();
        currentState = FINAL_RUN;
    }
}

void handleExploring() {
   while (1) {
        FP = false; RP = false; LP = false;
        pid_control();
        qtr.readLineWhite(sensorValues);
        
        if (sensorValues[0] < Twhite) {
            LP = true;
            Turn = 'L';
            digitalWrite(LED_L, HIGH);
            for (int i = 0; i < 20; i++) {
                qtr.readLineWhite(sensorValues);
                if (sensorValues[15] < Twhite) {
                    RP = true;
                    Turn = 'R';
                    digitalWrite(LED_R, HIGH);
                    break;
                }
            }
            break;
        } else {
            digitalWrite(LED_L, LOW);
        }
        
        if (sensorValues[15] < Twhite) {
            RP = true;
            Turn = 'R';
            digitalWrite(LED_R, HIGH);
            for (int i = 0; i < 20; i++) {
                qtr.readLineWhite(sensorValues);
                if (sensorValues[0] < Twhite) {
                    LP = true;
                    Turn = 'L';
                    digitalWrite(LED_L, HIGH);
                    break;
                }
            }
            break;
        } else {
            digitalWrite(LED_R, LOW);
        }
        
        if (sensorValues[0] > Tblack && sensorValues[15] > Tblack && 
            sensorValues[7] > Tblack && sensorValues[8] > Tblack) {
            uTurn();
            recordPath('U');
            optimizePath();
        }
    }
    
    breakMotor();
    moveForward();
    stopMotors();
    qtr.readLineWhite(sensorValues);
    
    if ((sensorValues[0] < Twhite && sensorValues[1] < Twhite && sensorValues[2] < Twhite && sensorValues[3] < Twhite && sensorValues[4] < Twhite && sensorValues[5] < Twhite) || 
        (sensorValues[10] < Twhite && sensorValues[11] < Twhite && sensorValues[12] < Twhite && sensorValues[13] < Twhite && sensorValues[14] < Twhite && sensorValues[15] < Twhite)) {
        stopMotors();
        exploreTime = millis() - startTime;
        currentState = FINAL_RUN_PENDING;
        return; 
    }

        // Check for straight path
    if (sensorValues[5] > Tblack && sensorValues[6] > Tblack && sensorValues[7] > Tblack && sensorValues[8] > Tblack && sensorValues[9] > Tblack && sensorValues[10] > Tblack && sensorValues[11] > Tblack && sensorValues[12] > Tblack ) {
        FP = false;
    } else {
        FP = true;
    }
    if (ABUT) {
        if (LP) {
            turnLeft();
            recordPath('L');
            optimizePath();
        } else if (FP) {
            moveForward();
            recordPath('S');
            optimizePath();
        } else if (RP) {
            turnRight();
            recordPath('R');
            optimizePath();
        }
    } else {
        if (RP) {
            turnRight();
            recordPath('R');
            optimizePath();
        } else if (FP) {
            moveForward();
            recordPath('S');
            optimizePath();
        } else if (LP) {
            turnLeft();
            recordPath('L');
            optimizePath();
        }
    }
    digitalWrite(LED_L, LOW);
    digitalWrite(LED_R, LOW);
}

void handleFinalRun() {
  int i = 0;
  uint16_t position = qtr.readLineWhite(sensorValues);
  while (pathLength) {
      while (1) {
      qtr.readLineWhite(sensorValues);
      pid_control();
      if (sensorValues[0] < Twhite || sensorValues[15] < Twhite) {
        break;
      }
    }
    stopMotors();
    delay(10);
    setMotor(50,50);
    delay(25);
    stopMotors();
    delay(10);
    
    if (path[i] == 'L') {
      turnLeft();
    } else if (path[i] == 'S') {
      moveForward();
    } else if (path[i] == 'R') {
      turnRight();
    }

    i++;
    if(i > pathLength){
      break;
    }
  }
    stopMotors();
    finalRunTime = millis() - startTime;
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_L, HIGH);
    currentState = FINISHED;
}

void pid_control() {

    // uint16_t position = qtr.readLineWhite(sensorValues);

    // // FIX 1: Invert the error calculation so the signs match the motor logic
    // // Range: -7500 (Left) to +7500 (Right)
    // int error = position - center_position; 

    // // P Term
    // int P = error;

    // // D Term
    // int D = error - last_error;
    // last_error = error;

    // // I Term (Optional: Disabled for now to ensure stability)
    // // Only enable I if the robot oscillates around the center but never quite settles
    // // I = I + error;
    // // int I_term = I * Ki; 

    // // PID Calculation
    // // Note: Removed I for initial tuning
    // float motorAdjustment = (P * Kp) + (D * Kd); 

    // // Calculate motor speeds
    // int rightMotorSpeed = BASE_SPEED_R - motorAdjustment;
    // int leftMotorSpeed  = BASE_SPEED_L + motorAdjustment;

    // // FIX 2: Allow negative speeds for sharp turns (Pivot turning)
    // // The SparkFun library handles negative numbers by reversing the motor direction
    // rightMotorSpeed = constrain(rightMotorSpeed, -MAX_SPEED_R, MAX_SPEED_R);
    // leftMotorSpeed  = constrain(leftMotorSpeed,  -MAX_SPEED_L, MAX_SPEED_L);

    // // Set motor speeds
    // mspeed(rightMotorSpeed, leftMotorSpeed);

    qtr.readLineWhite(sensorValues);
    long numerator = 0;
    long denominator = 0;
    for (int i = 4; i <= 11; i++) {
        int value = 1000 - sensorValues[i];
        if (value < 0) value = 0; 
        numerator += (long)value * (i * 1000);
        denominator += value;
    }
    uint16_t position = center_position; 
    if (denominator > 0) {
        position = numerator / denominator;
    } else {
        if (last_error > 0) position = 12000; 
        else if (last_error < 0) position = 3000; 
    }
    int error = position - center_position; 
    int P = error;
    int D = error - last_error;
    last_error = error;
    float motorAdjustment = (P * Kp) + (D * Kd); 
    int rightMotorSpeed = BASE_SPEED_R - motorAdjustment;
    int leftMotorSpeed  = BASE_SPEED_L + motorAdjustment;
    rightMotorSpeed = constrain(rightMotorSpeed, -MAX_SPEED_R, MAX_SPEED_R);
    leftMotorSpeed  = constrain(leftMotorSpeed,  -MAX_SPEED_L, MAX_SPEED_L);
    mspeed(rightMotorSpeed, leftMotorSpeed);
}

void mspeed(int posa, int posb) {
    motor_r.drive(posa);
    motor_l.drive(posb);
}

void setMotor(int set_speed_r, int set_speed_l) {
    motor_r.drive(set_speed_r);
    motor_l.drive(set_speed_l);
}

bool checkEndOfMaze() {
    qtr.readLineWhite(sensorValues);
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > Twhite) return false;
    }
    return true;
}

void moveForward() {
    setMotor(30, 30);
    delay(20);
}

void turnLeft() {
     setMotor(120, -120);
     delay(100);
   while (sensorValues[9] < Twhite || sensorValues[8] < Twhite) {
        qtr.readLineWhite(sensorValues);
        setMotor(120, -120);
    }
    while (sensorValues[6] > Tblack || sensorValues[7] > Tblack) {
        qtr.readLineWhite(sensorValues);
        setMotor(120, -120);
    }
     stopMotors();
}

void turnRight() {
      setMotor(-120, 120);
      delay(100);
   while (sensorValues[9] < Twhite || sensorValues[8] < Twhite) {
        qtr.readLineWhite(sensorValues);
        setMotor(-120, 120);
    }
    while (sensorValues[10] > Tblack || sensorValues[11] > Tblack) {
        qtr.readLineWhite(sensorValues);
        setMotor(-120, 120);
    }
    stopMotors();
}

void uTurn() {
    while (sensorValues[8] > Tblack && sensorValues[9]) {
        qtr.readLineWhite(sensorValues);
        setMotor(-120, 120);
    }
    stopMotors();
}

void breakMotor() {
    setMotor(-255,-255);
    delay(45);
    motor_r.brake();
    motor_l.brake();
    delay(10);
}
void stopMotors() {
    motor_r.brake();
    motor_l.brake();
    delay(10);
}

void recordPath(char move) {
    if (pathLength < 100) {
        path[pathLength++] = move;
    }
}

void optimizePath() {
  if (pathLength < 3 || path[pathLength - 2] != 'U') 
    return;
  int total_angle = 0;
  int m;
  for (m = 1; m <= 3; m++) {
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
  total_angle = total_angle % 360;
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