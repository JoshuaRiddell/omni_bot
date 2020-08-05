#include "stm32f4xx_hal.h"

void stepper_setup();
void motion_setup();
void enable_steppers();
void disable_steppers();
void move_robot(float x, float y, float w);
