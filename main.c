/*
NOTES, IDEAS, CURRENT STATE, TITLE, LICENSE, ETC. SHOULD GO HERE.

TODO: modify further from juggleball. Keep PWM/ADC stuff. PWM should be on 40Khz-ish (Above audio, far below radio), on TIM14CH1 and TIM3CH1,2,3. ADC on PA5, switch on PA0.

*/
#include <stdbool.h>
#include "stm32f030xx.h" // the modified Frank Duignan header file. (I started from his "Blinky" example). 
#include "config.h"  // ?

volatile int adcresult, debug; // can be read in debugger too.
int tick;

void delay(int dly)
{
  while( dly--);
}

void initClock()
{
// Led lamp will use standard 8Mhz internal clock, so not much config needed

        // TODO:Set adc clock (Or leave it as is?)
      	ADC_CFGR2|=(BIT31); // Clock ADC with PCLOCK/4 =8Mhz/4=2Mhz

      	// enable pheripheral clock to timer3 (BIT1) & TIM14 (BIT8).
        RCC_APB1ENR |= (BIT1 | BIT8 );
        
        RCC_APB2ENR |= BIT9; // enable clock to adc
}


void setup_adc(){
	ADC_CR |= (BIT0); // Set ADEN / enable power to adc BEFORE making settings!
        
        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for further settings/starting a coversion
        
        // make rest of settings before starting conversion:
        ADC_CHSELR = (BIT5); // Ch5 = PA5, on pin11 (Set up channels)

        // It will scan "all these" 1 (one) channels, but it has only 1 data register for the result. So it will scan them (Low-High is default, so CH1,2,3,1,2,3,)
        ADC_CFGR1 |= (BIT12 | BIT16); // BIT12 set it to discard on overrun and overwrite with latest result 
                               // BIT16: DISCEN Discontinues operation (Don't auto scan, wait for trigger to scan next ch, cannot be used when CONT=1)
                               // BIT13: CONT. automatically restart conversion once previous conversion is finished. 

        ADC_SMPR |= ( BIT1 | BIT2 | BIT3); // Set sample rate (Default = as fast as it can: 1.5clk, with bit1&2 set 71.5clk, with just bit 1: 13.5clk, all three set: 239.5clck cycles. At 2Mhz that's ~8kHz     
        
        ADC_IER |=(BIT2) ; // Enable end of conversion interrupt (Bit2) 
	//Note: EOSEQ (End of Sequence) is bit 3.      
        
        /// ***interrupts*** ///
        /* from code example, on howto enable interrupt in NVIC. But nowhere in datasheet does it say how to init NVIC whithout those functions... 
        NVIC_EnableIRQ(ADC1_COMP_IRQn); // enable ADC interrupt
        NVIC_SetPriority(ADC1_COMP_IRQn,2); // set priority (to 2)
        Fortunately the STM32F0xxx Cortex-M0 programming manual (PM0215) does document the NVIC. somewhat. And the CMSIS libs can be downloaded from st.com
        I could just use them... But I don't :)
        */
        
        ISER |= (BIT12); // Enable IRQ12, (That's the adc)
        IPR3 |= 96; // set priority for IRQ12 (4*IPRn+IRQn), starting from 0, so for IRQ12 that's IPR3 bits 7 downto 0
        //Read the relevant part of PM0215. IRC number is the position listed in RM0360 table 11.1.3.
        
        while (!(ADC_ISR&BIT0));// check ADCRDY (In ADC_ISR, bit0) to see if ADC is ready for starting a coversion
        
        ADC_CR |= (BIT2); // Set ADSTART to start conversion  
}


int main() {
	initClock();

	// enable clock to Porta
	RCC_AHBENR |= BIT17;
	

	GPIOA_MODER |= (BIT9 | BIT11|BIT10 | BIT13 | BIT15 | BIT21 ) ; // PA0 INput for switch (No config needed), PA4 AF (PWM WW), PA5 Ain (Potmeter), PA6,7,10 AF (PWM Y,CW,R)
	
	//TODO: (Aanpassen naar ledstripdimmersituatie): Set unused pins to a defined state so floating inputs do not consume power
	GPIOF_MODER |= BIT0|BIT1|BIT2|BIT3; // PF0 and PF1 to Analog Input
	GPIOB_MODER |= BIT2|BIT3; //PB1 to AIN
		
	
	//Before enabling ADC, let it calibrate itself by settin ADCAL (And waiting 'till it is cleared again before enabling ADC)
        ADC_CR |= (BIT31); // set adcal	

	//set up timer 3 for PWM
	TIM3_PSC = 0; // prescaler. (8Mhz/psc+1=tim3clock) = 8Mhz
        TIM3_ARR = 2048;  // 16 bit timer, AutoReloadRegister (frequency) (8E6/((TIM3_PSC+1)*TIM3_ARR), about 4Khz, should be OK with FETs/EMC but may be audible...
        //TIM3_CCR1 = 2048; // Compare register 1, dutycycle on output 1 (It has 4)


        TIM3_CCMR1 |= (BIT3 | BIT5 | BIT6 | BIT11 | BIT14 | BIT13) ;      // Set OC1PE (bit3), OC2PE (bit11) preload enable, PWM mode for ch2 (BIT 14,13), ch1 (Bit 6,5) 
	//TIM3_CCMR2 |= (BIT3|BIT5|BIT6);          			  //XXX set OC3PE (Bit3), PWM mode for ch3 (Bit 6,5) (TIMER3CH3 has no output on TSSOP20, use T1C3 instead)
	TIM3_CCER |= (BIT0 | BIT4 ) ; // CC1P to set polarity of output (0=active high), CC1E (bit 0) to enable output on ch1, bit4 for ch2, XXX Bit8 voor ch3.
        TIM3_CR1 |= BIT7 ;        // Control register. Set ARPE (bit7). And CEN I suppose (Counter enable, bit 0)
        TIM3_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
        TIM3_CR1 |= BIT0 ; // start after updating registers!

        // Set AF(AF1) en MODER to select PWM output on pins
	GPIOA_AFRL |= (BIT24 |BIT28 | BIT18); ; // Bit27:24 for AFR6 / PA6, that should get set to AF1 (0001) for TIM3CH1, BIT28 is idem for PA7/TIM3ch2. And bit 18 for  AF4 on PA4: TIM14CH1 (PWM)
	GPIOA_AFRH |= (BIT9); // TIM3CH3 voor PA10
	
        //set up timer 14 for PWM
        TIM14_PSC = 0; // prescaler. (8Mhz/psc+1=tim14clock)
        TIM14_ARR = 2048;  // 16 bit timer, AutoReloadRegister (frequency) (8E6/((TIM14_PSC+1)*TIM14_ARR)
        TIM14_CCMR1 |= (BIT3 | BIT5 | BIT6 ) ;      // PWM mode (per output bit 4:6). Set OC1PE (bit5) preload enable. 
        TIM14_CCER |= (BIT0) ; //  CC1E (bit 0) to enable output on ch1.
        TIM14_CR1 |= BIT7 ;        // Control register. Set ARPE (bit7). And CEN I suppose (Counter enable, bit 0)
        TIM14_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
        TIM14_CR1 |= BIT0 ; // start after updating registers!
        
	// Set up Timer 1 for PWM (on CH3)
	TIM1_ARR = 2048;
	TIM1_CCMR2 |= (BIT7|BIT6|BIT5|BIT3); // PWM on CH3, output enable, preload enable
	TIM1_CCER |= (BIT8); // output enable
	TIM1_EGR |= BIT0 ; // set UG to generate update event so registers are read to the timer
        TIM1_CR1 |= BIT0 ; // start after updating registers!
	



        // Wait for ADCAL to be zero again:
        while (ADC_CR & (BIT31));
        // then power up and set up adc:
        setup_adc();

		
	while(1){	

	enum mode{DIM_LUXE,DIM_CWWW,DIM_CW,DIM_WW,DIM_R,DIM_Y} mode;

 	static int i=0;
		// PWM max = 2047, ADC max = 4095		

	//mode=DIM_CWWW;
	mode=DIM_R;
	//mode=DIM_Y;
	
	switch(mode) // TODO: use HW switch to change mode
	{
	case DIM_LUXE:
	break;

	case DIM_CWWW:
        TIM3_CCR1 = 0; 
        TIM3_CCR2 = 2048-adcresult/2;
	TIM1_CCR3 = 0;             	
	TIM14_CCR1 = adcresult/2;
	// TODO: stabilize / only change brightness when adcresult changed significantly and interpret everything near 0 as 0.
	break;

	case DIM_CW:
        TIM3_CCR1 = 0; 
        TIM3_CCR2 = adcresult/2;
	TIM1_CCR3 = 0;             	
	TIM14_CCR1 = 0;
	break;

	case DIM_WW:
	TIM3_CCR1 = 0; 
        TIM3_CCR2 = 0;
	TIM1_CCR3 = 0;             	
	TIM14_CCR1 = adcresult/2;
	break;

	case DIM_R:
	TIM3_CCR1 = 0; 
        TIM3_CCR2 = 0;
	TIM1_CCR3 = adcresult/2;             	
	TIM14_CCR1 = 0;	
	break;

	case DIM_Y:
	TIM3_CCR1 = adcresult/2; 
        TIM3_CCR2 = 0;
	TIM1_CCR3 = 0;             	
	TIM14_CCR1 = 0;
	break;

	default:
	mode=DIM_R;
	}

	
	debug = adcresult;

	if(i<2048){
		i++;
	}else i=0;

	ADC_CR |= (BIT2); // Set ADSTART to start (next) conversion	
	}

}



void ADC_Handler(){
	tick++; //XXX do I still need this?
        
        if(ADC_ISR&(BIT2)) // Check EOC (Could check EOSEQ when the sequence is only 1 conversion long)
        {
                adcresult=ADC_DR; // read adc result for debugger/global use. (Also clears flag)
        }
        
        debug = ADC_ISR;
        
}

void EXTI_Handler(void){
// empty for now, could do wake up stuff here later TODO
EXTI_PR |=(BIT5); // clear the flag. 
}

