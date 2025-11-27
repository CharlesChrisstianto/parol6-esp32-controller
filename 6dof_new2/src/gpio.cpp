#include "gpio.h"
#include "parameter.h"

void Init_all_gpio(){
    pinMode(STEP1, OUTPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(LIMIT1, INPUT_PULLUP);
    
    pinMode(STEP2, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(LIMIT2, INPUT_PULLUP);
    
    pinMode(STEP3, OUTPUT);
    pinMode(DIR3, OUTPUT);
    pinMode(LIMIT3, INPUT_PULLUP);

    pinMode(STEP4, OUTPUT);
    pinMode(DIR4, OUTPUT);
    pinMode(LIMIT4, INPUT_PULLUP);

    pinMode(STEP5, OUTPUT);
    pinMode(DIR5, OUTPUT);
    pinMode(LIMIT5, INPUT_PULLUP);

    pinMode(STEP6, OUTPUT);
    pinMode(DIR6, OUTPUT);
    pinMode(LIMIT6, INPUT_PULLUP);
    
}