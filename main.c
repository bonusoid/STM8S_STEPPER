//2022-05-14
//Created by : bonusoid
//Stepper Motor Control

#include"delay.h"
#include"delay.c"
#include"periph_stm8s.h"
#include"periph_stm8s.c"
#include"REG/stm8s_gpio.h"

//MOTOR PINS
#define MOTP1	P3
#define MOTP2	P4
#define MOTP3	P5
#define MOTP4	P6
#define MOTODR	PC_ODR
#define MOTDDR	PC_DDR
#define MOTCR1	PC_CR1
#define MOTCR2	PC_CR2	

//BUTTON PINS
#define BTNCW	P1	//Button for Clockwise
#define BTNCCW	P2	//Button for Counter-Clockwise
#define BTNIDR	PA_IDR
#define BTNDDR	PA_DDR
#define BTNCR1	PA_CR1
#define BTNCR2	PA_CR2
#define BTNCW_MASKL	P1_MASKL
#define BTNCCW_MASKL	P2_MASKL	

//STEP SEQUENCE
unsigned char stepcw[4] = {0b1010,0b1001,0b0101,0b0110};  //Clockwise
unsigned char stepccw[4] = {0b0101,0b1001,0b1010,0b0110}; //Counter-Clockwise

void gpio_init();
void loop();
void setstep(unsigned char st); //Update rotor step

//^^^^^^^^^^ INIT ^^^^^^^^^^//
int main()
{
  clock_init();
  delay_init();
  gpio_init();
  
  loop();
  return 0;
}
//__________ INIT __________//


//^^^^^^^^^^ LOOP ^^^^^^^^^^//
void loop()
{
	unsigned char sti; //Step increment

	while(1)
	{
		if((BTNIDR|BTNCW_MASKL)==BTNCW_MASKL) //If CW Button is pressed (Active Low)
		{
			for(sti=0;sti<4;sti++)
			{
				setstep(stepcw[sti]); //Run CW sequence
				delay_ms(10);
			}
		}
		else if((BTNIDR|BTNCCW_MASKL)==BTNCCW_MASKL) //If CCW Button is pressed (Active Low)
		{
			for(sti=0;sti<4;sti++)
			{
				setstep(stepccw[sti]); //Run CCW sequence
				delay_ms(10);
			}
		}
		else
		{
			MOTODR = 0x00;
		}
	} 	
}
//__________ LOOP __________//

void gpio_init()
{
	MOTDDR |= (OUTPUT<<MOTP1)|(OUTPUT<<MOTP2)|(OUTPUT<<MOTP3)|(OUTPUT<<MOTP4);
	MOTCR1 |= (pushpull<<MOTP1)|(pushpull<<MOTP2)|(pushpull<<MOTP3)|(pushpull<<MOTP4);
	MOTCR2 |= (speed_2MHz<<MOTP1)|(speed_2MHz<<MOTP2)|(speed_2MHz<<MOTP3)|(speed_2MHz<<MOTP4);

	BTNDDR |= (INPUT<<BTNCW)|(INPUT<<BTNCCW);
	BTNCR1 |= (pullup<<BTNCW)|(pullup<<BTNCCW); //Use internal pull-up
	BTNCR2 |= (exti_disabled<<BTNCW)|(exti_disabled<<BTNCCW);
}

void setstep(unsigned char st)
{
	MOTODR = st<<MOTP1; //Update step
}
