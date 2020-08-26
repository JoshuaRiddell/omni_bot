#include "stm32f4xx_hal.h"

typedef struct {
	double x;
	double y;
	double theta;
} position_t;

void pin_setup(void);
void timer_setup(void);
void motion_setup();
void enable_steppers();
void kinematics_setup();
void disable_steppers();
void move_robot(float x, float y, float w);
float get_speed(uint8_t motor_num);
position_t get_position();
