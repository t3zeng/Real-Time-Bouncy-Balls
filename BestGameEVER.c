#include <LPC17xx.h>
#include <RTL.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <time.h>

#include "BestGameEVER.h"
#include "GLCD.h"

#define B     0xD3D3D3
#define F     Yellow
#define S			Green

//A few global variables are declared at the very top of the function for the purpose of changing their values using real time sensor values and using them in other functions
short interrupt_flag = 0;
short global_delay= 10;
volatile unsigned short int ADC_Value;
volatile unsigned char ADC_Done = 0;
short ball_counter = 0;
unsigned short *erase_bitmap;
unsigned short *ball_bitmap1;
unsigned short *ball_bitmap2;

//The struct allows us to pass the parameters of our ball into our ball creation thread while allowing the task to only require a void pointer input
typedef struct
{
	short x;
	short y;
	short width;
	short height;
}ball_parameters_t;

//Wait function that uses a for loop to pause the interupt handler
void wait(int delay)
{
	int n;
	for(n=0;n<delay*1000;n++){}
}	

//Bit map making which makes 3 different bit maps
//1)The erase bitmap that erases the previous ball instead of the entire screen to reduce the LCD refresh time
//2)The ball bitmap1 and the ball bitmap2 are seperate bitmaps that when switching between them simulate the ball rolling
void BMP_init()
{
	short i,j,counter=0, r = 8;
	
	erase_bitmap = malloc(sizeof(unsigned short)*4*r*r);
	ball_bitmap1 = malloc(sizeof(unsigned short)*4*r*r);
	ball_bitmap2 = malloc(sizeof(unsigned short)*4*r*r);
	for(i=-r;i<r;i++)
	{
		for(j=-r;j<r;j++)
		{
			if(i*i+j*j < r*r)
			{
				ball_bitmap1[counter] = F;
				ball_bitmap2[counter] = S;
				if(abs(i-j)<r/2)
				{
					ball_bitmap1[counter] = S;
					ball_bitmap2[counter] = F;
				}
			}
			else
			{
				ball_bitmap1[counter] = B;
				ball_bitmap2[counter] = B;
			}
			erase_bitmap[counter] = B;
			counter++;
		}
	}

}

//**********************************************
//*								LEDS  											 *
//**********************************************
//Initially sets all the LEDS to the off state
void LEDInit()
{
//Enable power 
  LPC_SC->PCONP       |=  (1 << 15); 
//LED connected to p1.28 is in GPIO mode
	LPC_PINCON->PINSEL3 &= ~(3 << 24); 
//LED connected to p1.28 is an output pin
	LPC_GPIO1->FIODIR   |=  (1 << 28);
//Turning on the LED
	//LPC_GPIO1->FIOSET   |=  (1 << 28);
//Turning off the LED
	LPC_GPIO1->FIOCLR   |=  (1 << 28);
	
	//Second LED
	LPC_GPIO1->FIODIR   |=  (1 << 29);
	LPC_GPIO1->FIOCLR   |=  (1 << 29);
	
	//Third LED
	LPC_GPIO1->FIODIR   |=  (1 << 31);
	LPC_GPIO1->FIOCLR   |=  (1 << 31);

  //Fourth LED
	LPC_GPIO2->FIODIR   |=  (1 << 2);
	LPC_GPIO2->FIOCLR   |=  (1 << 2);
	
	//Fifth LED
	LPC_GPIO2->FIODIR   |=  (1 << 3);
	LPC_GPIO2->FIOCLR   |=  (1 << 3);
	
	//Sixth LED
	LPC_GPIO2->FIODIR   |=  (1 << 4);
	LPC_GPIO2->FIOCLR   |=  (1 << 4);
	
	//Seventh LED
	LPC_GPIO2->FIODIR   |=  (1 << 5);
	LPC_GPIO2->FIOCLR   |=  (1 << 5);
	
	//Eighth LED
	LPC_GPIO2->FIODIR   |=  (1 << 6);
	LPC_GPIO2->FIOCLR   |=  (1 << 6);
}

//Using bit shifting to show in binary the number of ball on the screen 
void LEDSetter()
{
	LEDInit();
	if(ball_counter & 1<<0)
		LPC_GPIO2->FIOSET   |=  (1 << 6);
	if(ball_counter & 1<<1)
		LPC_GPIO2->FIOSET   |=  (1 << 5);
	if(ball_counter & 1<<2)
		LPC_GPIO2->FIOSET   |=  (1 << 4);
	if(ball_counter & 1<<3)
		LPC_GPIO2->FIOSET   |=  (1 << 3);
	if(ball_counter & 1<<4)
		LPC_GPIO2->FIOSET   |=  (1 << 2);
	if(ball_counter & 1<<5)
		LPC_GPIO1->FIOSET   |=  (1 << 31);
	if(ball_counter & 1<<6)
		LPC_GPIO1->FIOSET   |=  (1 << 29);
	if(ball_counter & 1<<7)
		LPC_GPIO1->FIOSET   |=  (1 << 28);
}

//**********************************************
//*							Draw Balls									  	*
//**********************************************
//Task to draw the balls
__task void Balls(void* INTO_THE_VOID)
{
	//convert void to real deal parameters
	ball_parameters_t *param = (ball_parameters_t*)INTO_THE_VOID;
	short x = param->x;
	short y = param->y;
	short width = param->width;
	short height = param->height;							 
	
	//Set the balls initial direction to be 45 degrees
	short directionX = 1;
	short directionY = 1;
	//Spion and counter are variables used to make the ball look like they are rolling
	short spin = 0;
	short counter = 0;
	while(1)
	{
		//Inifinte while loop that switches between the two different bitmaps 
		GLCD_Bitmap (x,y,width,height, (unsigned char*)erase_bitmap);
		x = x+directionX;
		y = y+directionY;
		//Shows 2 frames of the same bitmap so the ball does not roll too quickly
		//Uses bitmap1
		if(spin == 0 && counter <= 2)
		{
			GLCD_Bitmap (x,y,width,height, (unsigned char*)ball_bitmap1);
			if (counter == 2)
				spin = 1;
			counter = counter + 1;
		}
		//Uses bitmap2
		else if(spin == 1 && counter > 2)
		{
			GLCD_Bitmap (x,y,width,height, (unsigned char*)ball_bitmap2);
			if(counter == 4)
			{
				counter = 0;
				spin = 0;
			}
			counter = counter + 1;
		}
		
		//Changes the direction of the ball when it hits a wall
		if(abs(x - (320-width)) < 1 || x < 1)
			directionX = -1*directionX;
		if(abs(y - (240-height)) < 1 || y < 1)
			directionY = -1*directionY;
		os_dly_wait(global_delay);
	}
}

//************************************************
//*							Generate Ball  									 *
//************************************************
void generate_ball()
{
	ball_parameters_t *parameters = (ball_parameters_t *)malloc(sizeof(ball_parameters_t));
	//Giving the the ball inital coordinates of (0,0) and hight and width of 16
	parameters->x = 0;
	parameters->y = 0;
	parameters->width = 16;
	parameters->height = 16;
	
	//Increses the ball counter for when displaying the number of ball on the screen with the LEDSS
	ball_counter = ball_counter + 1;
	LEDSetter();
	os_tsk_create_ex( Balls, 3, parameters );
	free(parameters);
	parameters = NULL;
}


//************************************************
//*							Interrupts											 *
//************************************************
void INTOInit()
{
//Push-button connected to p2.10 is in GPIO mode
  LPC_PINCON->PINSEL4    &= ~( 3 << 20 ); 
//P2.10 is an input pin
  LPC_GPIO2->FIODIR      &= ~( 1 << 10 );
//P2.10 reads the falling edges to generate an interrupt
  LPC_GPIOINT->IO2IntEnF |=  ( 1 << 10 );
//IRQ is enabled in NVIC
	NVIC_EnableIRQ( EINT3_IRQn );
}

//************************************************
//*							Interrupt Routine 							 *
//************************************************
void EINT3_IRQHandler () {
	interrupt_flag = 1;
	wait(500);
	//Clear interrupt condition
	LPC_GPIOINT->IO2IntClr |=  (1 << 10);
}

//************************************************
//*							Potentiometer	Init  						 *
//************************************************
void POTInit()
{
	LPC_SC->PCONP |= ( 1 << 12 );
  LPC_PINCON->PINSEL1 &= ~( 0x3 << 18 );
	LPC_PINCON->PINSEL1 |=  ( 0x1 << 18 );
	LPC_ADC->ADCR = ( 1 <<  2 ) |
									( 4 <<  8 ) |
									( 0 << 24 ) |
									( 1 << 21 ); 
	LPC_ADC->ADINTEN = ( 1 <<  8);  
	NVIC_EnableIRQ( ADC_IRQn  ); 
  LPC_ADC->ADCR |= ( 1 << 24 );
}

//************************************************
//*						Potentiometer	Covert  						 *
//************************************************
void ADCConvert (void) {
	// Stop reading and converting the port channel AD0.2.
  LPC_ADC->ADCR &= ~( 7 << 24); 
	ADC_Done = 0;
	// Start reading and converting the analog input from P0.25, where Poti is connected
	//to the challen Ad0.2
  LPC_ADC->ADCR |=  ( 1 << 24) | (1 << 2);              //start conversion
}


//************************************************
//*					 Potentiometer	Handler  						 *
//************************************************
void ADC_IRQHandler()
{
	volatile unsigned int aDCStat;
	aDCStat = LPC_ADC->ADSTAT;
	ADC_Value = (LPC_ADC->ADGDR >> 4) & 0xFFF;
	if(ADC_Value < 700)
		ADC_Value = 700;
	
	global_delay = ADC_Value/700;
	
	ADC_Done = 1;
}

//**********************************************
//*					Read Potentiometer Task		      	 *
//**********************************************
__task void readPoti_task(void)
{	
	while( 1 ){
		ADCConvert();
		//Now wait for the other threads.
		os_dly_wait( 1 );
	}
}

__task void task_caller()
{
	os_tsk_prio_self(1);
	os_tsk_create( readPoti_task, 2);
	while(1)
	{
		if(interrupt_flag == 1)
		{
			interrupt_flag = 0;
			generate_ball();
		}
	}
}
//************************************************
//*							Main        										 *
//************************************************
int main( void ) {
	SystemInit();
	SystemCoreClockUpdate();
	
  GLCD_Init();
	GLCD_Clear(B);
	BMP_init();
	LEDInit();
	POTInit();
	INTOInit();
	os_sys_init(task_caller);
  while(1);
}
