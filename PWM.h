// period is 16-bit number of PWM clock cycles in one period (3<=period)
// period for PB6 and PB7 must be the same
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
// PWM clock rate = processor clock rate/SYSCTL_RCC_PWMDIV
//                = BusClock/2 
//                = 80 MHz/2 = 40 MHz (in this example)
// Output on PB6/M0PWM0
void PWM0L_Init(uint16_t period, uint16_t duty);

// change duty cycle of PB6
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0L_Duty(uint16_t duty);

// period is 16-bit number of PWM clock cycles in one period (3<=period)
// period for PB6 and PB7 must be the same
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
// PWM clock rate = processor clock rate/SYSCTL_RCC_PWMDIV
//                = BusClock/2 
//                = 80 MHz/2 = 40 MHz (in this example)
// Output on PB7/M0PWM1
void PWM0R_Init(uint16_t period, uint16_t duty);

// change duty cycle of PB7
// duty is number of PWM clock cycles output is high  (2<=duty<=period-1)
void PWM0R_Duty(uint16_t duty);

