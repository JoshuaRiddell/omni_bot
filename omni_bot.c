#include "omni_bot.h"
#include "stm32f4xx_hal.h"
#include <math.h>


#define freq_source 84000000 //internal clock source freq

uint16_t prescaler = 40; //prescaler used by timer
uint32_t freq_counter; //the freq of the timer calculated from freq_source and prescaler
uint16_t init_speed = 10000; //this sets the acceleration by setting the timing of first step, the smaller the number the faster the acceleration
uint16_t SPR = 3200; //steps per revolution of the stepper motor
float tick_freq[3]; //the freq that the steps need to be calculated from frq_counter RPM and SPR
float speed[3]; //the current speed measured by timer ticks in ARR value to count up to
float target_speed[3]; //the target speed that speed is accelerating towards

int32_t n[3];
int8_t curret_dir[3], target_dir[3], RPM_zero[3];

float alpha1, alpha2, alpha3; //the angles of force from motors
float a, b, c, d, e, f, g, h, i;//the input matrix
float det, a2, b2, c2, d2, e2,f2, g2, h2, i2;// the inverse matrix
float x_speed, y_speed, w_speed; //desired speeds in 3 DOF


void stepper_setup(void){

	freq_counter = freq_source / (prescaler + 1); //calculate the timer freq

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

	//setup all 3 timers

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //enable the timer4 clock
	TIM3->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM3->PSC = prescaler;   //set prescale
	TIM3->CCMR1 = (TIM3->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM3->CCR1 = 10; //set to min rise time
	TIM3->ARR = init_speed; //set to timing
	TIM3->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM3->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM3->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt(NVIC level)

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enable the timer4 clock
	TIM4->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM4->PSC = prescaler;   //set prescale
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM4->CCR1 = 10; //set to min rise time
	TIM4->ARR = init_speed; //set to timing
	TIM4->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM4->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM4->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)


	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; //enable the timer4 clock
	TIM5->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM5->PSC = prescaler;   //set prescale
	TIM5->CCMR1 = (TIM4->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM5->CCR1 = 10; //set to min rise time
	TIM5->ARR = init_speed; //set to timing
	TIM5->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM5->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM5->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM5_IRQn); // Enable interrupt(NVIC level)

	//initialize variables
	speed[0] = init_speed;
	speed[1] = init_speed;
	speed[2] = init_speed;
	RPM_zero[0] = 1;
	RPM_zero[1] = 1;
	RPM_zero[2] = 1;
	curret_dir[0] = 1;
	curret_dir[1] = 1;
	curret_dir[2] = 1;
	target_dir[0] =1;
	target_dir[1] =1;
	target_dir[2] =1;
}

void disable_steppers(void){
	//set all 3 enable pins to low
	GPIOA->ODR &= ~(GPIO_ODR_OD1 | GPIO_ODR_OD4);
	GPIOD->ODR &= ~GPIO_ODR_OD10;
}

void enable_steppers(void){
	//set all 3 enable pins to high
	GPIOA->ODR |= GPIO_ODR_OD1 | GPIO_ODR_OD4;
	GPIOD->ODR |= GPIO_ODR_OD10;
}

void set_speed(uint8_t motor_num, float RPM){

	RPM = RPM * -1;
	if(fabs(RPM) < 0.01){
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
		target_speed[motor_num] = freq_counter / tick_freq[motor_num];

		if(motor_num==0)TIM3->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 3.
		if(motor_num==1)TIM4->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 4.
		if(motor_num==2)TIM5->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 5.
	}
}


void motion_setup(){
	  //calucate force direction from motors in radians
	   alpha1 = 240 * M_PI/180;
	   alpha2 = 120 * M_PI/180;
	   alpha3 = 0 * M_PI/180;

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

void move_robot (float x, float y, float w){
	  set_speed(0, a2 * x + b2 * y + c2 * w);
	  set_speed(1, d2 * x + e2 * y + f2 * w);
	  set_speed(2, g2 * x + h2 * y + i2 * w);
}


void TIM3_IRQHandler(void){

	TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[0] && (speed[0] >= init_speed))TIM3->CR1 &= ~TIM_CR1_CEN;

	//if the current direction is same as target direction
	if(target_dir[0] == curret_dir[0]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[0]>=init_speed){
			speed[0] = init_speed;
			n[0]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[0] >= init_speed) && (speed[0] >= init_speed)){
			speed[0] = target_speed[0];
			n[0]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[0]>target_speed[0]){
					n[0]++;
					speed[0] = speed[0] - ( (2 * speed[0]) / (4 * n[0] + 1) );
		  	  }else if(n[0]>0){
		  		speed[0] = (speed[0] * (4 * n[0] + 1) / (4 * n[0] - 1));
				n[0]--;
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[0] > init_speed-100){
			if(target_dir[0])GPIOA->ODR &= ~GPIO_ODR_OD5; //set direction pin
			else GPIOA->ODR |= GPIO_ODR_OD5; //set direction pin
			curret_dir[0] = target_dir[0];
			speed[0] = init_speed;
			n[0] = 0;

		//else slow down
		}else if(n[0]>0){
			speed[0] = (speed[0] * (4 * n[0] + 1) / (4 * n[0] - 1));
			n[0]--;
		}
	}

	if(speed[0]<11)speed[0]=11;
	if(speed[0]>65535){
		TIM5->CR1 &= ~TIM_CR1_CEN;
		speed[0] = init_speed;
		n[0]=0;
	}

	TIM3->ARR = (uint32_t)speed[0];//update ARR
}



void TIM4_IRQHandler(void){

	TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[1] && (speed[1] >= init_speed))TIM4->CR1 &= ~TIM_CR1_CEN;

	//if the current direction is same as target direction
	if(target_dir[1] == curret_dir[1]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[1]>=init_speed){
			speed[1] = init_speed;
			n[1]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[1] >= init_speed) && (speed[1] >= init_speed)){
			speed[1] = target_speed[1];
			n[1]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[1]>target_speed[1]){
					n[1]++;
					speed[1] = speed[1] - ( (2 * speed[1]) / (4 * n[1] + 1) );
		  	  }else if(n[1]>0){
		  		  	speed[1] = (speed[1] * (4 * n[1] + 1) / (4 * n[1] - 1));
					n[1]--;
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[1] > init_speed-100){
			if(target_dir[1])GPIOD->ODR &= ~GPIO_ODR_OD11; //set direction pin
			else GPIOD->ODR |= GPIO_ODR_OD11; //set direction pin
			curret_dir[1] = target_dir[1];
			speed[1] = init_speed;
			n[1] = 0;

		//else slow down
		}else if(n[1]>0){
			speed[1] = (speed[1] * (4 * n[1] + 1) / (4 * n[1] - 1));
			n[1]--;
		}
	}

	if(speed[1]<11)speed[1]=11;
	if(speed[1]>65535){
		TIM5->CR1 &= ~TIM_CR1_CEN;
		speed[1] = init_speed;
		n[1]=0;
	}

	TIM4->ARR = (uint32_t)speed[1];//update ARR
}


void TIM5_IRQHandler(void){

	TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[2] && (speed[2] >= init_speed))TIM5->CR1 &= ~TIM_CR1_CEN;


	//if the current direction is same as target direction
	if(target_dir[2] == curret_dir[2]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[2]>=init_speed){
			speed[2] = init_speed;
			n[2]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[2] >= init_speed) && (speed[2] >= init_speed)){
			speed[2] = target_speed[2];
			n[2]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[2]>target_speed[2]){
					n[2]++;
					speed[2] = speed[2] - ( (2 * speed[2]) / (4 * n[2] + 1) );
		  	  }else if(n[2]>0){
		  		  	speed[2] = (speed[2] * (4 * n[2] + 1) / (4 * n[2] - 1));
					n[2]--;
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[2] > init_speed-100){
			if(target_dir[2])GPIOA->ODR &= ~GPIO_ODR_OD2; //set direction pin
			else GPIOA->ODR |= GPIO_ODR_OD2; //set direction pin
			curret_dir[2] = target_dir[2];
			speed[2] = init_speed;
			n[2] = 0;

		//else slow down
		}else if(n[2]>0){
			speed[2] = (speed[2] * (4 * n[2] + 1) / (4 * n[2] - 1));
			n[2]--;
		}
	}

	if(speed[2]<11)speed[2]=11;
	if(speed[2]>65535){
		TIM5->CR1 &= ~TIM_CR1_CEN;
		speed[2] = init_speed;
		n[2]=0;
	}
	TIM5->ARR = (uint32_t)speed[2];//update ARR
}

