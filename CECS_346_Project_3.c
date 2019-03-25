// Justin Maeder, Jason Chang
// CECS 346 
// Final Project

// _______________Sensor on_________________
//		green -- flash red -- blue
// _________________SW1 on__________________ 
//		green -- flash red -- blue
// _________________SW2 on__________________
//		blue  -- flash red -- green

// 0.Documentation Section 
// main.c
// Runs on LM4F120 or TM4C123
// Lab2_HelloLaunchPad, Input from PF4, output to PF3,PF2,PF1 (LED)
// Authors: Daniel Valvano, Jonathan Valvano and Ramesh Yerraballi
// Date: January 15, 2016

// LaunchPad built-in hardware
// SW1 left switch is negative logic PF4 on the Launchpad
// SW2 right switch is negative logic PF0 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad

// 1. Pre-processor Directives Section
// Constant declarations to access port registers using 
// symbolic names instead of addresses


#include <stdio.h>
#include <stdint.h>

//____________________________GPIO PORT ADDRESS___________________________//
//_______________________________PORT F___________________________________//
#define Light                   (*((volatile unsigned long *)0x40025038))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	
//_______________________________PORT D___________________________________//
#define GPIO_PORTD_DATA_BITS_R  ((volatile uint32_t *)0x40007000)
#define GPIO_PORTD_DATA_R       (*((volatile uint32_t *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile uint32_t *)0x40007400))
#define GPIO_PORTD_IS_R         (*((volatile uint32_t *)0x40007404))
#define GPIO_PORTD_IBE_R        (*((volatile uint32_t *)0x40007408))
#define GPIO_PORTD_IEV_R        (*((volatile uint32_t *)0x4000740C))
#define GPIO_PORTD_IM_R         (*((volatile uint32_t *)0x40007410))
#define GPIO_PORTD_RIS_R        (*((volatile uint32_t *)0x40007414))
#define GPIO_PORTD_MIS_R        (*((volatile uint32_t *)0x40007418))
#define GPIO_PORTD_ICR_R        (*((volatile uint32_t *)0x4000741C))
#define GPIO_PORTD_AFSEL_R      (*((volatile uint32_t *)0x40007420))
#define GPIO_PORTD_DR2R_R       (*((volatile uint32_t *)0x40007500))
#define GPIO_PORTD_DR4R_R       (*((volatile uint32_t *)0x40007504))
#define GPIO_PORTD_DR8R_R       (*((volatile uint32_t *)0x40007508))
#define GPIO_PORTD_ODR_R        (*((volatile uint32_t *)0x4000750C))
#define GPIO_PORTD_PUR_R        (*((volatile uint32_t *)0x40007510))
#define GPIO_PORTD_PDR_R        (*((volatile uint32_t *)0x40007514))
#define GPIO_PORTD_SLR_R        (*((volatile uint32_t *)0x40007518))
#define GPIO_PORTD_DEN_R        (*((volatile uint32_t *)0x4000751C))
#define GPIO_PORTD_LOCK_R       (*((volatile uint32_t *)0x40007520))
#define GPIO_PORTD_CR_R         (*((volatile uint32_t *)0x40007524))
#define GPIO_PORTD_AMSEL_R      (*((volatile uint32_t *)0x40007528))
#define GPIO_PORTD_PCTL_R       (*((volatile uint32_t *)0x4000752C))
#define GPIO_PORTD_ADCCTL_R     (*((volatile uint32_t *)0x40007530))
#define GPIO_PORTD_DMACTL_R     (*((volatile uint32_t *)0x40007534))
	
//_______________________________PORT E___________________________________//
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_LOCK_R       (*((volatile unsigned long *)0x40024520))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))

//________________________________INTERRUPTS______________________________//
//____________________________PORT E Interrupts___________________________//
#define GPIO_PORTE_IS_R         (*((volatile unsigned long *)0x40024404))
#define GPIO_PORTE_IBE_R        (*((volatile unsigned long *)0x40024408))
#define GPIO_PORTE_IEV_R        (*((volatile unsigned long *)0x4002440C))
#define GPIO_PORTE_IM_R         (*((volatile unsigned long *)0x40024410))
#define GPIO_PORTE_RIS_R        (*((volatile unsigned long *)0x40024414))
#define GPIO_PORTE_ICR_R        (*((volatile unsigned long *)0x4002441C)) 
	
#define NVIC_PRI1_R             (*((volatile unsigned long *)0xE000E404))

//____________________________PORT F Interrupts___________________________//
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
	

//________________________PORT E Interrupts NVIC___________________________//
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
//____________________________END INTERRUPTS_______________________________//

//__________________________Systick_Clock Init_____________________________//
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))

#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value

#define SYSCTL_RCGCGPIO_R       (*((volatile uint32_t *)0x400FE608))
	
#define blue  0x04
#define green 0x08
#define open  1 
#define close 0 
// 2. Declarations Section
//   Global Variables

volatile unsigned long door;
volatile unsigned int  i;
volatile unsigned long Counts;
volatile unsigned long sensor;
volatile unsigned long rise,fall;

struct State
{
  uint8_t Out;     // Output
  uint8_t Next[2]; // CW/CCW
};
typedef const struct State StateType;

#define clockwise 0        // Next index
#define counterclockwise 1 // Next index
StateType fsm[4]=
{
  {12,{1,3}},
  { 6,{2,0}},
  { 3,{3,1}},
  { 1,{0,2}}
};
unsigned char s; // current state

#define STEPPER  (*((volatile uint32_t *)0x4000703C))
	

//   Function Prototypes
void delay(void);
void delay1(void);
void Stepper_CW(void);
void Stepper_Init(void);
void SysTick_Wait(void);
void Stepper_CCW(void);

void open_cw(void);
void open_ccw(void);

void PortF_Init(void);
void flash(void);
void PortE_Init(void);
void EnableInterrupts(void);
void SysTick_Init(unsigned long period);

// 3. Subroutines Section
// MAIN: Mandatory for a C Program to be executable

/*
	0x02  == red
	0x08  == green 
	0x04  == blue
*/
int main(void)
{    
  PortF_Init();        // Call initialization of port PF4 PF2    
	PortE_Init();        
	SysTick_Init(8000000);  //500 ms delay 
	EnableInterrupts();
	Stepper_Init();
	Light = 0x08;
	door = 0;
	while(1){}
}
void GPIOPortF_Handler(void)   // sw1 and sw2
{
	if (GPIO_PORTF_RIS_R & 0x10)
	{
		GPIO_PORTF_ICR_R = 0x10; 
		if (Light & green)
		{
			open_cw();
			Light = blue;
		}
		else if(Light & blue){}
	}
	else if(GPIO_PORTF_RIS_R & 0x01)
	{
		GPIO_PORTF_ICR_R = 0x01;
		if  (Light & blue)
		{
			open_ccw();
			Light = green;
		}
		else if (Light & green){}
	}
}

void flash (void)
{
	Counts = 0;
	while (Counts < 6)
	{
		if 		  (Counts%2 == 0) Light = 0x00;
		else if (Counts%2 == 1) Light = 0x02;
	}
	Light = 0x00;
}

void GPIOPortE_Handler(void) //interrupt for pin PE0
{
	GPIO_PORTE_ICR_R= 0x01; 
	if((Light & green) && (GPIO_PORTE_DATA_R == 0x00))
	{
		open_cw();
		Light = blue;
		
	}
	else if((Light & green)&&(GPIO_PORTE_DATA_R == 0x01)){}
	else if((Light & blue) && (GPIO_PORTE_DATA_R == 0x01))
	{
		open_ccw();
		Light = green; 
	}
	else if ((Light & blue) && (GPIO_PORTE_DATA_R == 0x00)){}
	delay1();
}

void open_cw(void)
{
	i = 0;
	Counts = 0;
	while (i <1000)
	{
		i++;	
		if 		  (Counts%2 == 0) Light = 0x00;
		else if (Counts%2 == 1) Light = 0x02;
		Stepper_CW();
	}
}
void open_ccw(void){
	i = 0;
	Counts = 0;
	while (i <1000)
	{
		i++;	
		if 		  (Counts%2 == 0) Light = 0x00;
		else if (Counts%2 == 1) Light = 0x02;
		Stepper_CCW();
	}
}
void delay(void)
{
	unsigned int time = 0;
	time = 1*13000;
	while(time)
	{
		time--;
	}
}
void delay1(void)
{
	unsigned int time = 0;
	time = 727240*200/91;  // 0.1sec
	while(time)
	{
		time--;
	}
}
void SysTick_Handler(void)
{
	Counts = Counts + 1;	
}
// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
void PortE_Init(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC2_R       |=  0x00000010;     // 1) E clock
	delay = SYSCTL_RCGC2_R;                  // delay   
	GPIO_PORTE_CR_R      |=  0x1F;           // allow changes to PE0     
	GPIO_PORTE_AMSEL_R   &= ~0x1F;           // 3) disable analog function
	GPIO_PORTE_PCTL_R    &= ~0x0000001F;     // 4) GPIO clear bit PCTL  
	GPIO_PORTE_DIR_R     &= ~0x01;           // 5) PE0 are inputs 
	GPIO_PORTE_DIR_R     |=  0x1E;
	GPIO_PORTE_AFSEL_R   &= ~0x1F;           // 6) no alternate function
	GPIO_PORTE_PUR_R     &= ~0x1F;           // disable pullup resistors on PE0       
	GPIO_PORTE_DEN_R     |=  0x1F;           // 7) enable digital pins PE0    
	
	// Interrupts
	GPIO_PORTE_IS_R      &= ~0x01;           // Clear for Edge Sensitive
	GPIO_PORTE_IBE_R     |=  0x01;           // set for Both Edge
	GPIO_PORTE_IEV_R     |=  0x01;           // Set
	GPIO_PORTE_ICR_R     |=  0x01;           // Clear flags
	GPIO_PORTE_IM_R      |=  0x01;           // ARM interrupt
	
	// Priority
	NVIC_PRI1_R           = ((NVIC_PRI1_R & ~0x000000A0) | 0x000000A0) ; // Priority 5
	NVIC_EN0_R           |= 0x00000010;      // Enable interrupt 4 in NVIC
	sensor = 0;
	
}

void PortF_Init(void)
{ 
	volatile unsigned long delay;
	SYSCTL_RCGC2_R      |= 0x00000020;    // 1) F clock
	delay 							 = SYSCTL_RCGC2_R;// delay   
	GPIO_PORTF_LOCK_R    =  0x4C4F434B;   // 2) unlock PortF PF0  
	GPIO_PORTF_CR_R     |=  0x1F;         // allow changes to PF4-0       
	GPIO_PORTF_AMSEL_R  &= ~0x1F;         // 3) disable analog function
	GPIO_PORTF_PCTL_R   &= ~0x000FFFFF;   // 4) GPIO clear bit PCTL  
	GPIO_PORTF_DIR_R    |=  0x0E;   
	GPIO_PORTF_DIR_R    &= ~0x11;   		  // 5) PF4,PF0 input, PF3,PF2,PF1 output   
	GPIO_PORTF_AFSEL_R  &= ~0x1F;         // 6) no alternate function
	GPIO_PORTF_PUR_R    |=  0x11;         // enable pullup resistors on PF4,PF0       
	GPIO_PORTF_DEN_R    |=  0x1F;         // 7) enable digital pins PF4-PF0  
  
	// Interrupts
	GPIO_PORTF_IS_R    &= ~0x11;        // Clear for Edge Sensitive
	GPIO_PORTF_IBE_R   &= ~0x11;        // Clear for not Both Edge
	GPIO_PORTF_IEV_R   &= ~0x11;        // Clear for Falling Edge
	GPIO_PORTF_ICR_R   |=  0x11;        // Clear flags
	GPIO_PORTF_IM_R    |=  0x11;        // ARM interrupt
	
  // Set Interrupt Priority
	NVIC_PRI7_R    =   (NVIC_PRI7_R & ~0x00A00000) | 0x00A00000; // Priority 5
	NVIC_EN0_R    |=   0x40000000;         // Enable interrupt 4 in NVIC

	Light |= 0x0E;
}

// Interrupt service routine
// Executed every 62.5ns*(period)

void Stepper_CW(void)
{
	s = fsm[s].Next[clockwise]; // clock wise circular
	STEPPER = fsm[s].Out;       // step motor
	delay();
}
// Move 1.8 degrees counterclockwise, delay is wait after each step
void Stepper_CCW(void)
{
	s = fsm[s].Next[counterclockwise]; // counter clock wise circular
	STEPPER = fsm[s].Out;              // step motor
	delay();
}
// Initialize Stepper interface
void Stepper_Init(void)
{
	SYSCTL_RCGCGPIO_R  |= 0x08;       // 1) activate port D
	s = 0; 
                                    // 2) no need to unlock PD3-0
	GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
	GPIO_PORTD_PCTL_R  &= ~0x0000FFFF;// 4) GPIO configure PD3-0 as GPIO
	GPIO_PORTD_DIR_R   |=  0x0F;      // 5) make PD3-0 out
	GPIO_PORTD_AFSEL_R &= ~0x0F;      // 6) disable alt funct on PD3-0
	GPIO_PORTD_DR8R_R  |=  0x0F;      // enable 8 mA drive
	GPIO_PORTD_DEN_R   |=  0x0F;      // 7) enable digital I/O on PD3-0 
}

void SysTick_Init(unsigned long period)
{
	NVIC_ST_CTRL_R 	  = 0;         // disable SysTick during setup
	NVIC_ST_RELOAD_R  = period-1;  // reload value
	NVIC_ST_CURRENT_R = 0;         // any write to current clears it
	NVIC_SYS_PRI3_R   = (NVIC_SYS_PRI3_R & ~0x40000000)|0x40000000; // priority 2
                                 // enable SysTick with core clock and interrupts
	NVIC_ST_CTRL_R 	 |= 0x07;
	Counts = 0;
}
