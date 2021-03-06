#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include "omni_bot.h"

#define freq_source 84000000 //internal clock source freq for all timers


//--------------------------------------------------------------------------------------
//following constants and variables are used for generating step pulse train

#define prescaler_motor 40 //prescaler used by step timers
#define init_speed 25000 //this sets the acceleration by setting the timing of first step, the smaller the number the faster the acceleration
#define SPR 3200 //steps per revolution of the stepper motor

uint32_t  *motor_en[3] = {&TIM3->CR1, &TIM4->CR1, &TIM5->CR1}; //pointers used to enable/disable motor timers
uint32_t  *motor_ARR[3] = {&TIM3->ARR, &TIM4->ARR, &TIM5->ARR}; //pointer used to update ARR for motor timers
uint32_t  *motor_ODR[3] = {&GPIOA->ODR, &GPIOD->ODR, &GPIOA->ODR}; //pointers used to switch dir pin
uint32_t dir_pin[3] = {GPIO_ODR_OD5, GPIO_ODR_OD11, GPIO_ODR_OD2}; //dir pin number to write to the ODR

uint32_t freq_motor_counter; //the freq of the timer calculated from freq_source and prescaler

float tick_freq[3]; //the freq that the steps need to be calculated from frq_counter RPM and SPR
float speed[3] = {init_speed, init_speed, init_speed} ; //the current speed measured by timer ticks in ARR value to count up to
float target_speed[3]; //the target speed that speed is accelerating towards

int32_t n[3];
int8_t curret_dir[3] = {1, 1, 1};
int8_t target_dir[3] = {1, 1, 1};
int8_t RPM_zero[3];
int8_t stepper_enable;

double motor_rpm[3] = {0.};
//------------------------------------------------------------------------------------------


//folowing constants and varibles are used for the dead reckening

#define wheel_circumference 318.8  //the was calculated by making robot do exactly 10 wheel rotation then measuring and divide by 10
#define robot_dia 136.5 //this was done by doing 100 rotation of robot then working backewards from number of steps taken
#define SPR_divider 16 //this will be used to divide up the micro stepping

double robot_arc, delta_theda, delta_distance;
double robot_position[3]; //this will store x, y, w position of the robot
double motor_y_offsets[3]; //this will hold the y offset angles of the motors.
double motor_x_offsets[3]; //this will hold the x offset angles of the motors.

uint8_t divider_counter[3];

//--------------------------------------------------------------------------------
//following variables are used for motion kinematics

float alpha1, alpha2, alpha3; //the angles of force from motors
float a, b, c, d, e, f, g, h, i;//the input matrix
float det, a2, b2, c2, d2, e2,f2, g2, h2, i2;// the inverse matrix
//--------------------------------------------------------------------------------





void disable_steppers(void);
void enable_steppers(void);
void set_speed(uint8_t motor_num, float RPM); //low level control of motor speed
void move_robot (float x, float y, float w); //kinematic movement of robot
void kinematics_setup(void);


position_t get_position() {
	position_t position;
	position.x = robot_position[0] / 1000.;
	position.y = robot_position[1] / 1000.;
	position.theta = robot_position[2];
	return position;
}

void set_position(position_t position) {
	robot_position[0] = position.x * 1000.;
	robot_position[1] = position.y * 1000.;
	robot_position[2] = position.theta;
}

//this fuction is very rough and needs a lot of work before it is good
//ATM it only goto (0,0,0) and is just a proof of concept
void goto_position (double x_finish, double y_finish, double w_finish){

	float x_speed, y_speed, w_speed;

	while((robot_position[2] > 0.05) | (robot_position[2] < -0.05)){
		if(robot_position[2]>0)move_robot(0 , 0,  60);
		else move_robot(0 , 0,  -60);
		HAL_Delay(10);

		  printf("first  w= ");
		  print_float(robot_position[2]);
		  printf("  x= ");
		  print_float(robot_position[0]);
		  printf("  y= ");
		  print_float(robot_position[1]);
//		  printf("  yaw_st = %d  x_st = %d  y_st = %d   ch_6 = %d\r\n", yaw_stick, x_stick, y_stick, ch_6);
	}
	move_robot(0 , 0,  0);


	float delta_x = x_finish - robot_position[0];
	float delta_y = y_finish - robot_position[1];
	float distance_2_finish_squared = delta_x * delta_x + delta_y * delta_y;
	float angle_2_finish = tan(delta_x / delta_y);


	while (distance_2_finish_squared > 100){

		delta_x = x_finish - robot_position[0];
		delta_y = y_finish - robot_position[1];
		distance_2_finish_squared = delta_x * delta_x + delta_y * delta_y;

		if(delta_x > 0){
			if (delta_x > 30)x_speed = 50;
			else x_speed = 20;
		}else if(delta_x < -30)x_speed = -50;
				else x_speed = -20;

		if(delta_y > 0){
			if (delta_y > 30)y_speed = 50;
			else y_speed = 20;
		}else if(delta_y < -30)y_speed = -50;
				else y_speed = -20;

		move_robot(x_speed , y_speed,  0);

		HAL_Delay(10);

		  printf("second  w= ");
		  print_float(robot_position[2]);
		  printf("  x= ");
		  print_float(robot_position[0]);
		  printf("  y= ");
		  print_float(robot_position[1]);
		  //printf("  yaw_st = %d  x_st = %d  y_st = %d   ch_6 = %d\r\n", yaw_stick, x_stick, y_stick, ch_6);

	}

	move_robot(0 ,0,  0);

	while((robot_position[2] > 0.01) | (robot_position[2] < -0.01)){
		if(robot_position[2]>0)move_robot(0 , 0,  20);
		else move_robot(0 , 0,  -20);
		//HAL_Delay(10);

		  printf("first  w= ");
		  print_float(robot_position[2]);
		  printf("  x= ");
		  print_float(robot_position[0]);
		  printf("  y= ");
		  print_float(robot_position[1]);
//		  printf("  yaw_st = %d  x_st = %d  y_st = %d   ch_6 = %d\r\n", yaw_stick, x_stick, y_stick, ch_6);
	}
	move_robot(0 , 0,  0);




}



void pin_setup(void){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN; //enable port A and port D clock

	//setup timer pins
	GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE6_1; //setup pin A0 and pin A6 to AF
	GPIOD->MODER |= GPIO_MODER_MODER12_1; //setup pin D12 to AF mode
	GPIOA->AFR[0] = (GPIOD->AFR[1] &  ~(0b1111 | (0b1111<<(6*4)))) | 0b0010 | (0b0010<<(6*4)); //set pin A0 and pin A6 to AF timer mode       ;
	GPIOD->AFR[1] = (GPIOD->AFR[1] & ~(0b1111<<(4*(12-8)))) | 0b0010<<(4*(12-8)); //set pin D12 to AF timer mode

	//setup direction and enable pins
	GPIOA->MODER |= GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 |  GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOD->MODER |= GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0;

	//set all 3 enable pins to low
	GPIOA->ODR &= ~(GPIO_ODR_OD1 | GPIO_ODR_OD4);
	GPIOD->ODR &= ~GPIO_ODR_OD10;

	//set all 3 dir pins
	GPIOA->ODR &= ~(GPIO_ODR_OD2 | GPIO_ODR_OD5);
	GPIOD->ODR &= ~GPIO_ODR_OD11;
}


void timer_setup(void){


	freq_motor_counter = freq_source / (prescaler_motor + 1); //calculate the motor timer freq

	//enable the 3 clocks
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN |RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN;

	//timer 3 is used for motor 0
	TIM3->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM3->PSC = prescaler_motor;   //set prescale
	TIM3->CCMR1 = (TIM3->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM3->CCR1 = 10; //set to min rise time
	TIM3->ARR = init_speed; //set to timing
	TIM3->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM3->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM3->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt(NVIC level)
	NVIC_SetPriority 	(TIM3_IRQn, 0);

	//timer 4 is used for motor 1
	TIM4->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM4->PSC = prescaler_motor;   //set prescale
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM4->CCR1 = 10; //set to min rise time
	TIM4->ARR = init_speed; //set to timing
	TIM4->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM4->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM4->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)
	NVIC_SetPriority 	(TIM4_IRQn, 0);

	//timer 5 is used for motor 2
	TIM5->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM5->PSC = prescaler_motor;   //set prescale
	TIM5->CCMR1 = (TIM5->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM5->CCR1 = 10; //set to min rise time
	TIM5->ARR = init_speed; //set to timing
	TIM5->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM5->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM5->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM5_IRQn); // Enable interrupt(NVIC level)
	NVIC_SetPriority 	(TIM5_IRQn, 0);
}


void motion_setup(){
	  //calucate force direction from motors in radians
	   alpha1 = M_PI * 8 / 6;
	   alpha2 = M_PI * 4 / 6;
	   alpha3 = 0;

	   //fill input matrix
	   a = cos(alpha1);
	   b = cos(alpha2);
	   c = cos(alpha3);
	   d = sin(alpha1);
	   e = sin(alpha2);
	   f = sin(alpha3);
	   g = 1;
	   h = 1;
	   i = 1;

	   //caluate the determint
	   det = a * e * i + b * f * g + c * d * h - c * e * g - a * f * h - b * d * i;

	   //calulate the inverse
	   a2 = (e * i - f * h) / det;
	   b2 = (h * c - i * b) / det;
	   c2 = (b * f - c * e) / det;
	   d2 = (g * f - d * i) / det;
	   e2 = (a * i - g * c) / det;
	   f2 = (d * c - a * f) / det;
	   g2 = (d * h - g * e) / det;
	   h2 = (g * b - a * h) / det;
	   i2 = (a * e - d * b) / det;
}

void kinematics_setup(void){

	robot_arc = robot_dia * 2;  //all trig functions are in radians

	delta_theda = wheel_circumference /  (200 * (robot_arc + robot_dia));

	delta_distance = robot_arc * sin(delta_theda);

	//calculate the offsets in radians
	motor_x_offsets[0] = M_PI * 4 / 3; //240 degrees
	motor_x_offsets[1] = M_PI * 2 / 3;  //120 degrees
	motor_x_offsets[2] = 0; //0 degrees

	motor_y_offsets[0] = M_PI * 5 / 6;  //150 degrees
	motor_y_offsets[1] = M_PI / 6;  //30 degrees
	motor_y_offsets[2] = M_PI / -2;  //-90 degrees
}



void disable_steppers(void){

	//disable all 3 timers
	*motor_en[0] &= ~TIM_CR1_CEN;
	*motor_en[1] &= ~TIM_CR1_CEN;
	*motor_en[2] &= ~TIM_CR1_CEN;

	//reset all 3 timers
	speed[0] = init_speed;
	speed[1] = init_speed;
	speed[2] = init_speed;
	n[0] = 0;
	n[1] = 0;
	n[2] = 0;

	//set all 3 enable pins to low
	GPIOA->ODR &= ~(GPIO_ODR_OD1 | GPIO_ODR_OD4);
	GPIOD->ODR &= ~GPIO_ODR_OD10;

	//set the enable flag
	stepper_enable = 0;
}


void enable_steppers(void){

	//set all 3 enable pins to high
	GPIOA->ODR |= GPIO_ODR_OD1 | GPIO_ODR_OD4;
	GPIOD->ODR |= GPIO_ODR_OD10;

	//set the enable flag
	stepper_enable = 1;
}


float get_speed(uint8_t motor_num) {
	return motor_rpm[motor_num];
}

void set_speed(uint8_t motor_num, float RPM){
	motor_rpm[motor_num] = RPM;

	if(stepper_enable==0)return;

	RPM = RPM * -1; //This is used because I have dir pin setup backwards at some point this needs to be fixed :(

	if(RPM==0){
		RPM_zero[motor_num] = 1;
		target_speed[motor_num] = init_speed;
	}else{
		RPM_zero[motor_num] = 0;
		if(RPM>0)target_dir[motor_num] = 1;
		else{
			target_dir[motor_num] = 0;
			RPM = RPM *-1;
		}
		tick_freq[motor_num] = SPR * RPM / 60;
		target_speed[motor_num] = freq_motor_counter / tick_freq[motor_num];

		*motor_en[motor_num] |= TIM_CR1_CEN; //enable the timer
	}
}


void move_robot (float x, float y, float w){
	  set_speed(0, a2 * x + b2 * y + c2 * w);
	  set_speed(1, d2 * x + e2 * y + f2 * w);
	  set_speed(2, g2 * x + h2 * y + i2 * w);
}


void motor_update(uint8_t motor_num){

	//-------------------------------------------------------------------------
	//code to update the dead reckoning

	if((curret_dir[motor_num])){
		divider_counter[motor_num] ++;
		if(divider_counter[motor_num]==SPR_divider){

			divider_counter[motor_num] = 0;

			robot_position[0] -= delta_distance * cos( motor_x_offsets[motor_num] - robot_position[2] );
			robot_position[1] -= delta_distance * cos( motor_y_offsets[motor_num] - robot_position[2] );
			robot_position[2] += delta_theda;

			if(robot_position[2] > M_PI) robot_position[2] -= M_PI * 2;
		}
	}else{
		divider_counter[motor_num] --;
		if(divider_counter[motor_num]==0){

			divider_counter[motor_num] = SPR_divider;

			robot_position[0] += delta_distance * cos( motor_x_offsets[motor_num] - robot_position[2] );
			robot_position[1] += delta_distance * cos( motor_y_offsets[motor_num] - robot_position[2] );
			robot_position[2] -= delta_theda;

			if(robot_position[2] < (M_PI * -1)) robot_position[2] += M_PI * 2;
		}
	}
	//---------------------------------------------------------------------------------------------------
	//code to update the speed control

	//if the target speed is zero & current speed is slower init speed the disable the channel
		if (RPM_zero[motor_num] && (speed[motor_num] >= init_speed-100)){
			*motor_en[motor_num] &= ~TIM_CR1_CEN;
			speed[motor_num] = init_speed;
			n[motor_num]=0;
		}

		//if the current direction is same as target direction
		if(target_dir[motor_num] == curret_dir[motor_num]){

			//if target current speed is slower than init speed then set to init and reset n
			if (speed[motor_num]>=init_speed){
				speed[motor_num] = init_speed;
				n[motor_num]=0;
			}

			//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
			if((target_speed[motor_num] >= init_speed) && (speed[motor_num] >= init_speed - 100)){
				speed[motor_num] = target_speed[motor_num];
				n[motor_num]=0;

				//if current speed is slower than target then speed up else slow down
			}else if(speed[motor_num]>target_speed[motor_num]){
						n[motor_num]++;
						speed[motor_num] = speed[motor_num] - ( (2 * speed[motor_num]) / (4 * n[motor_num] + 1) );
			  	  }else if(n[motor_num]>0){
			  		  speed[motor_num] = (speed[motor_num] * (4 * n[motor_num] + 1) / (4 * n[motor_num] - 1));
			  		  n[motor_num]--;
			  	  }

		//else the current direction is not same as target direction
		}else{

			//if the current speed is slower than init speed then flip the direction pin and reset
			if(speed[motor_num] > init_speed - 100){
				if(target_dir[motor_num])*motor_ODR[motor_num] &= ~dir_pin[motor_num]; //set direction pin
				else *motor_ODR[motor_num] |= dir_pin[motor_num]; //set direction pin
				curret_dir[motor_num] = target_dir[motor_num];
				speed[motor_num] = init_speed;
				n[motor_num] = 0;

			//else slow down
			}else if(n[motor_num]>0){
				speed[motor_num] = (speed[motor_num] * (4 * n[motor_num] + 1) / (4 * n[motor_num] - 1));
				n[motor_num]--;
			}
		}

		if(speed[motor_num]<11)speed[motor_num]=11;
		if(speed[motor_num]>65535){
			*motor_en[motor_num] &= ~TIM_CR1_CEN;
			speed[motor_num] = init_speed;
			n[motor_num]=0;
		}

		*motor_ARR[motor_num] = (uint32_t)speed[motor_num];//update ARR

		//------------------------------------------------------------------------------------------------------------
}


void TIM3_IRQHandler(void){

	TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag

	motor_update(0); //This fuction calculates the next needed value for the ARR and switches the dir pin if needed
}


void TIM4_IRQHandler(void){

	TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag

	motor_update(1); //This fuction calculates the next needed value for the ARR and switches the dir pin if needed
}


void TIM5_IRQHandler(void){

	TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag

	motor_update(2); //This fuction calculates the next needed value for the ARR and switches the dir pin if needed
}




/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

