#pragma once
#include <Arduino.h>

void Init_all_gpio();


//-----Serial 1----------//
#define SERIAL1_TX_PIN 17
#define SERIAL1_RX_PIN 18

#define STEP1 5 //Joint 1
#define DIR1 4
#define LIMIT1 13

#define STEP2 7 //Joint 2
#define DIR2 6
#define LIMIT2 14

#define STEP3 16 //Joint 3
#define DIR3 15
#define LIMIT3 19

//-----Serial 2----------//

#define SERIAL2_TX_PIN 9
#define SERIAL2_RX_PIN 10

#define STEP4 8 //Joint 4
#define DIR4 3
#define LIMIT4 20

#define STEP5 45 //Joint 5
#define DIR5 48
#define LIMIT5 21

#define STEP6 11 //Joint 6
#define DIR6 12
#define LIMIT6 47

