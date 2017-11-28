// ADCTestMain.c
// Runs on LM4F120/TM4C123
// This program periodically samples ADC channel 1 and stores the
// result to a global variable that can be accessed with the JTAG
// debugger and viewed with the variable watch feature.
// Daniel Valvano
// October 20, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// input signal connected to PE2/AIN1

#include "ADCSWTrigger.h"
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include "Nokia5110.h"

#include "PLL.h"
#include "PWM.h"
#include <stdio.h>


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

unsigned short distance; 
volatile unsigned long ADCvalue;
uint16_t duty ; 


unsigned long ain1;
unsigned long ain2;
unsigned long ain9;
unsigned long ain8;

unsigned long distanceLeft; 
unsigned long distanceMiddle; 
unsigned long distanceRight; 

// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is an FIR filter:
// y(n) = (x(n) + x(n-1))/2
// Assumes: ADC initialized by previously calling ADC_Init2981()
void ReadADCFIRFilter(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8, unsigned long *ain1){
  //                   x(n-1)
  static unsigned long ain2previous=0;
  static unsigned long ain9previous=0;
  static unsigned long ain8previous=0;
  static unsigned long ain1previous=0;

  // save some memory; these do not need to be 'static'
  //            x(n)
  unsigned long ain2newest;
  unsigned long ain9newest;
  unsigned long ain8newest;
  unsigned long ain1newest;

   ADC_In2981(&ain2newest, &ain9newest, &ain8newest, &ain1newest ); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = (ain2newest + ain2previous)/2;
  *ain9 = (ain9newest + ain9previous)/2;
  *ain8 = (ain8newest + ain8previous)/2;
  *ain1 = (ain1newest + ain1previous)/2;

  ain2previous = ain2newest; ain9previous = ain9newest; ain8previous = ain8newest; ain1previous = ain1newest;
}

void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
  EnableInterrupts();
}
// Interrupt service routine
// Executed every 25ms*(period)
void SysTick_Handler(void){
  //ADC_In2981(&ain2, &ain9, &ain8, &ain1); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  ReadADCFIRFilter(&ain2, &ain9, &ain8, &ain1); 
  PWM0A_Duty(ain1 * (9.767765568));
  PWM0B_Duty(ain1 * (9.767765568));
}

// Subroutine to initialize port A pins for input and output
// PC4 & PC5 are outputs to the H-Bridge used to control direction
// Inputs: None
// Outputs: PC4 & PC5
void PortC_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000004;     // 1) A clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTC_CR_R = 0x30;           // allow changes to PC4 & PC5       
  GPIO_PORTC_AMSEL_R = 0x00;        // 2) disable analog function
  GPIO_PORTC_PCTL_R = 0x00000000;   // 3) GPIO clear bit PCTL  
  GPIO_PORTC_DIR_R = 0x30;          // 4) PC4 & PC5 output   
  GPIO_PORTC_AFSEL_R = 0x00;        // 5) no alternate function
  GPIO_PORTC_DEN_R = 0x30;          // 6) enable digital pins PC4 & PC5       
}

int main(void){unsigned long volatile delay;
  PLL_Init();            // 80 MHz
  Nokia5110_Init();      // initialize the Nokia LCD Screen 
  Nokia5110_Clear();
  PortC_Init();          // initialize PortC for purpose of driving motors
  ADC_Init2981();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5), AIN1 (PE2)
  PWM0A_Init(40000, ain1 * (9.767765568));          // initialize PWM0, 1000 Hz, 0% duty
  PWM0B_Init(40000, ain1 * (9.767765568));          // initialize PWM0, 1000 Hz, 0% duty
  SysTick_Init(1999999 * 8);   // initialize SysTick timer with corresponding 40Hz period 

  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF; // activate port F 
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_DIR_R |= 0x04;             // make PF2 out (built-in LED)
  GPIO_PORTF_AFSEL_R &= ~0x04;          // disable alt funct on PF2
  GPIO_PORTF_DEN_R |= 0x04;             // enable digital I/O on PF2 
                                        // configure PF2 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
  while(1){
    distanceLeft   = (40492.0468/ain8)  - 5.0134176; 
    distanceMiddle = (56441.51981/ain9) - 13.39044053; 
    distanceRight  = (35387.4376/ain2)  + 0.4844352; 
    Nokia5110_SetCursor(0, 0);
    Nokia5110_OutString("L:");
    Nokia5110_SetCursor(3, 0);
    Nokia5110_OutUDec(distanceLeft);
    Nokia5110_SetCursor(0, 1);
    Nokia5110_OutString("M:");
    Nokia5110_SetCursor(3, 1);
    Nokia5110_OutUDec(distanceMiddle);
    Nokia5110_SetCursor(0, 2);
    Nokia5110_OutString("R:");
    Nokia5110_SetCursor(3, 2);
    Nokia5110_OutUDec(distanceRight);    
    //Nokia5110_SetCursor(0, 3);
    //Nokia5110_OutUDec(ain1); 
    Nokia5110_SetCursor(0, 4);
    Nokia5110_OutString("Duty %");
    Nokia5110_SetCursor(7, 4);
    Nokia5110_OutUDec((ain1 * (9.767765568) / 40000) * 100); 

    //for(delay=0; delay<100000; delay++){};
    
  }
}

