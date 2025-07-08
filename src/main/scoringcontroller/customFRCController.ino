// Custom operator controller for 2025 Reefscape FRC Competition.
// Allows selection which coral arm to score on, and at which level.
// Uses an Arduino Leonardo.

// Requires downloading the "ArduinoJoystickLibrary" (https://github.com/MHeironimus/ArduinoJoystickLibrary/tree/version-1.0)


// // Uncomment the program for running on Arduino
// #include <Joystick.h>

// // Hardware declaration
// const int CORAL_POTENTIOMETER = A0;
// const int LEVEL_POTENTIOMETER = A1;

// // Potentiometer values
// const int MAX_ROTATIONAL_POTENTIOMETER_RANGE = 300;
// const int MAX_LINEAR_POTENTIOMETER_RANGE = 200;

// // Joystick Contants
// const int PRESSED = 1;
// const int RELEASED = 0;

// // Keep track of previous values
// int previousCoral = 1;
// int previousLevel = A2;
// int previousLevelLight = A3;

// //// Led Light pins
// // Coral Lights
// const int A1_LED = 11;
// const int A2_LED = 0;
// const int B1_LED = 1;
// const int B2_LED = 2;
// const int C1_LED = 3;
// const int C2_LED = 4;
// const int D1_LED = 5;
// const int D2_LED = 6;
// const int E1_LED = 7;
// const int E2_LED = 8;
// const int F1_LED = 9;
// const int F2_LED = 10;

// // Level lights
// const int LEVEL_1 = A2;
// const int LEVEL_2 = A3;
// const int LEVEL_3 = A4;
// const int LEVEL_4 = A5;


// void setup() {
//   // Initialize serial communication at 9600 bits per second:
//   Serial.begin(9600);

//   // Initialize Led lights
//   pinMode(A1_LED, OUTPUT); 
//   pinMode(A2_LED, OUTPUT); 
//   pinMode(B1_LED, OUTPUT); 
//   pinMode(B2_LED, OUTPUT); 
//   pinMode(C1_LED, OUTPUT); 
//   pinMode(C2_LED, OUTPUT); 
//   pinMode(D1_LED, OUTPUT); 
//   pinMode(D2_LED, OUTPUT); 
//   pinMode(E1_LED, OUTPUT); 
//   pinMode(E2_LED, OUTPUT); 
//   pinMode(F1_LED, OUTPUT); 
//   pinMode(F2_LED, OUTPUT); 
//   pinMode(LEVEL_1, OUTPUT); 
//   pinMode(LEVEL_2, OUTPUT); 
//   pinMode(LEVEL_3, OUTPUT); 
//   pinMode(LEVEL_4, OUTPUT); 
// }

// void loop() {
//   // Read value from potentiometers
//   int coralValue = analogRead(CORAL_POTENTIOMETER);
//   int levelValue = analogRead(LEVEL_POTENTIOMETER);

//   // Rescale to potentiometer angle
//   coralValue = map(coralValue, 0, 1023, 0, MAX_ROTATIONAL_POTENTIOMETER_RANGE);
//   levelValue = map(levelValue, 0, 1023, 0, MAX_LINEAR_POTENTIOMETER_RANGE);

//   // Int to keep track of what value to change
//   int currentCoral = -1;
//   int currentLevel = -1;

//   // Pin to change for level
//   int currentLevelLight = 0;

//   //// Coral output
//   // A2
//   if (coralValue >= 299) {
//     currentCoral = 2;
//   // B1
//   } else if (coralValue >= 282) {
//     currentCoral = 3;
//   // B2
//   } else if (coralValue >= 246) {
//     currentCoral = 4;
//   // C1
//   } else if (coralValue >= 213) {
//     currentCoral = 5;
//   // C2
//   } else if (coralValue >= 180) {
//     currentCoral = 6;
//   // D1
//   } else if (coralValue >= 150) {
//     currentCoral = 7;
//   // D2
//   } else if (coralValue >= 115) {
//     currentCoral = 8;
//   // E1
//   } else if (coralValue >= 78) {
//     currentCoral = 9;
//   // E2
//   } else if (coralValue >= 47) {
//     currentCoral = 10;
//   // F1
//   } else if (coralValue >= 12) {
//     currentCoral = 11;
//   // F2
//   } else if (coralValue >= 1) {
//     currentCoral = 12;
//   // A1
//   } else if (coralValue < 1) {
//     currentCoral = 1;
//   // Something has gone wrong
//   } else {
//     Serial.print("UNKNOWN CORAL VALUE ERROR");
//   }

//   /// Level output
//   // Level 1
//   if (levelValue >= 190) {
//     currentLevel = 16;
//     currentLevelLight = LEVEL_1;
//   // Level 2
//   } else if (levelValue >= 120) {
//     currentLevel = 17;
//     currentLevelLight = LEVEL_2;
//   // Level 3
//   } else if (levelValue >= 55) {
//     currentLevel = 18;
//     currentLevelLight = LEVEL_3;
//   // Level 4
//   } else if (levelValue < 55) {
//     currentLevel = 19;
//     currentLevelLight = LEVEL_4;
//   // Something has gone wrong
//   } else {
//     Serial.print("UNKNOWN LEVEL VALUE ERROR");
//   }

//   //// Set buttons
//   // Coral
//   if (currentCoral != -1) {
//     Joystick.setButton(previousCoral, RELEASED);
//     digitalWrite(previousCoral-2, LOW);
//     previousCoral = currentCoral;
//     Joystick.setButton(currentCoral, PRESSED);
//     if (currentCoral == 1) {
//       digitalWrite(11, HIGH);
//     } else {
//       digitalWrite(11, LOW);
//     }
//     digitalWrite(currentCoral-2, HIGH);
//   }

//   // Level
//   if (currentLevel != -1) {
//     Joystick.setButton(previousLevel, RELEASED);
//     analogWrite(previousLevelLight, 0);
//     previousLevel = currentLevel;
//     previousLevelLight = currentLevelLight;
//     Joystick.setButton(currentLevel, PRESSED);
//     analogWrite(currentLevelLight, 255);
//   }

//   // analogWrite(A5, 255);

//   // Send Joystick values to Driver Station
//   Joystick.sendState();

//   // Print out the Potentiometer values
//   Serial.print("coralValue: ");
//   Serial.print(coralValue);
//   Serial.print(", levelValue: ");
//   Serial.println(levelValue);
//   Serial.print("Current Coral: ");
//   Serial.print(currentCoral);
//   Serial.print(", Current Level: ");
//   Serial.println(currentLevel);
//   delay(50);
// }
