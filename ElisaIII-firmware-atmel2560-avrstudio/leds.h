#include "variables.h"

#ifndef LEDS_H
#define LEDS_H

void initRGBleds();
void toggleBlueLed();
void updateRedLed(unsigned char value);
void updateGreenLed(unsigned char value);
void updateBlueLed(unsigned char value);

#endif

