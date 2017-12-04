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

    double LCDduty; 

    unsigned long ain1;
    unsigned long ain2;
    unsigned long ain9;
    unsigned long ain8;

    unsigned short distanceLeft; 
    unsigned short distanceMiddle; 
    unsigned short distanceRight; 
    
    
    // --------------------------Median--------------------------------
    // 
    //      Helper function for ReadADCMedianFilter()
    //      Inputs:  3 unsigned long values
    //      Outputs: the median 
    // 
    // ----------------------------------------------------------------
    unsigned long median(unsigned long u1, unsigned long u2, unsigned long u3){
    unsigned long result;
      if(u1>u2)
        if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
          else
            if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
            else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
      else
        if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
          else
            if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
            else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
      return(result);
    }
    
    
    // ------------------------Median Filter--------------------------
    // 
    // This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
    // returns the results in the corresponding variables.  Some
    // kind of filtering is required because the IR distance sensors
    // output occasional erroneous spikes.  This is a median filter:
    // y(n) = median(x(n), x(n-1), x(n-2))
    // Assumes: ADC initialized by previously calling ADC_Init298()
    // 
    // --------------------------------------------------------------
    void ReadADCMedianFilter(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8,  unsigned long *ain1){
      //                   x(n-2)        x(n-1)
      static unsigned long ain2oldest=0, ain2middle=0;
      static unsigned long ain9oldest=0, ain9middle=0;
      static unsigned long ain8oldest=0, ain8middle=0;
      static unsigned long ain1oldest=0, ain1middle=0;

      // save some memory; these do not need to be 'static'
      //            x(n)
      unsigned long ain2newest;
      unsigned long ain9newest;
      unsigned long ain8newest;
      unsigned long ain1newest;
      ADC_In2981(&ain2newest, &ain9newest, &ain8newest, &ain1newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
      *ain2 = median(ain2newest, ain2middle, ain2oldest);
      *ain9 = median(ain9newest, ain9middle, ain9oldest);
      *ain8 = median(ain8newest, ain8middle, ain8oldest);
      *ain1 = median(ain1newest, ain1middle, ain1oldest);
      ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle, ain1oldest = ain1middle;
      ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest, ain1middle = ain1newest;
    }
    
    
    // ---------Systick Periodic Interrupt Initialization---------
    // 
    //      Interrupt Service Routine
    //      Executed every 25ms*(40Hz) 
    //      Controls how frequent we read the input from our sensors
    // 
    // -----------------------------------------------------------
    void SysTick_Init(unsigned long period){
        NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
        NVIC_ST_RELOAD_R = period-1;// reload value
        NVIC_ST_CURRENT_R = 0;      // any write to current clears it
        NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
        // enable SysTick with core clock and interrupts
        NVIC_ST_CTRL_R = 0x07;
        EnableInterrupts();
    }
    
    
    // --------------Systick Periodic Interrupt Handler--------------
    // 
    //      Interrupt service routine
    //      Executed every 25ms*(40Hz) 
    //      Controls how frequent we read the input from our sensors
    // 
    // --------------------------------------------------------------
    void SysTick_Handler(void){
        ReadADCMedianFilter(&ain2, &ain9, &ain8, &ain1); 
    }
    
    
    // ---------------------Initialize PortC---------------------------
    // 
    //          PC4 & PC5 are outputs to the H-Bridge 
    //          used to control direction
    //          Inputs: None
    //          Outputs: PC4 & PC5
    // 
    // ----------------------------------------------------------------
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
    
    // ---------------------Initialize PortF---------------------------
    // 
    //          PF4 - onboard button used to 
    //          turn system on and off
    //          Inputs: PF4 
    //          Outputs: none
    // 
    // ----------------------------------------------------------------
    void PortF_Init(void){ volatile unsigned long delay;
        SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
        GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
        GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
        GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4   
        GPIO_PORTF_PCTL_R &= ~0x000F0000; // configure PF4 as GPIO
        GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
        GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
        GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
        GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
        GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
        GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
        GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
        NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
        NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
        EnableInterrupts();           // (i) Clears the I bit  
    }
    // ---------------------PortF ISR---------------------------
    // 
    //          PF4 - onboard button used to 
    //          turn system on and off
    //          Inputs: PF4 
    //          Outputs: none
    // 
    // ---------------------------------------------------------
    unsigned char flag = 0;
    void GPIOPortF_Handler(void){
        if (GPIO_PORTF_RIS_R & 0x10) {
            GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
            flag +=1;
            if (flag % 2 ==1) {
            // update PWM duty cycle according to the potentiometer
            PWM0L_Duty(ain1 * (9.767765568));
            PWM0R_Duty(ain1 * (9.767765568));
            }   
            else if (flag % 2 ==0) {
            PWM0L_Duty(0);
            PWM0R_Duty(0);
            }
  
        }   
    }
  
    
    int main(void){
        PLL_Init();            // speed the clock to 80 MHz
        Nokia5110_Init();      // initialize the Nokia LCD Screen 
        Nokia5110_Clear();
        PortC_Init();          // initialize PortC for purpose of driving motors
        PortF_Init();          // initialize PortC for purpose of driving motors
        ADC_Init2981();        // initialize ADC to sample AIN2 (PE1), AIN9 (PE4), AIN8 (PE5), AIN1 (PE2)
        ReadADCMedianFilter(&ain2, &ain9, &ain8, &ain1); 
        LCDduty = ain1 * (9.767765568); 
        PWM0L_Init(40000, 0);          // initialize left motor's PWM
        PWM0R_Init(40000, 0);          // initialize right motor's PWM
        SysTick_Init(1999999);   // initialize SysTick timer with corresponding 40Hz period 

        // TODO: sample the sensors outside of the loop 
        while(1){
            // TODO: 
            // (1) update LCD
            // (2) update PWM duty cycle if needed 
            // (3) start / stop motors 
            
            // Inverse regression formula obtained using data obtained when  
            // calibrating sensors by measuring ADC values at corresponding 
            // distances...
            distanceLeft   = (40492.04680/ain8)  - 5.013417600; 
            distanceMiddle = (56441.51981/ain9)  - 13.39044053; 
            distanceRight  = (35387.43760/ain2)  + 0.484435200; 
            
            // update PWM duty cycle according to the potentiometer
            //PWM0L_Duty(ain1 * (9.767765568));
            //PWM0R_Duty(ain1 * (9.767765568));
            
            
            //out of range 
            if (distanceLeft >= 70) { 
                PWM0L_Duty(ain1 * (9.767765568) * .4);
                PWM0R_Duty(ain1 * (9.767765568) * .8);
            }
            
            //out of range 
            else if (distanceRight >= 70) { 
                PWM0L_Duty(ain1 * (9.767765568) * .8);
                PWM0R_Duty(ain1 * (9.767765568) * .4);
            }
             
            else if (distanceMiddle >=70) {
                PWM0L_Duty(ain1 * (9.767765568));
                PWM0R_Duty(ain1 * (9.767765568));
            }
           
            else if (distanceLeft <= 50 ) {
                PWM0L_Duty(ain1 * (9.767765568) * .9);
                PWM0R_Duty(ain1 * (9.767765568) * .7);
                
            } 
            else if (distanceRight <= 50) { 
                PWM0L_Duty(ain1 * (9.767765568) * .7);
                PWM0R_Duty(ain1 * (9.767765568) * .9);
            }
            else 
                PWM0L_Duty(0);
                PWM0R_Duty(0);
            

            
            // Update LCD 
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
            Nokia5110_SetCursor(0, 4);
            Nokia5110_OutString("Duty %");
            Nokia5110_SetCursor(7, 4);
            Nokia5110_OutUDec((ain1 * (9.767765568) / 39999) * 100);  
      }
    }

