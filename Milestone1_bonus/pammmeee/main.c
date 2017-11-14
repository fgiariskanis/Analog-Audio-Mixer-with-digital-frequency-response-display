/*********************************************************************
 * Milestone1 
 * Developed by: Chousos Christos AM:2012030117
 *	         Giariskanis Fotios AM:2011030087
 *Milestone1_bonus:
 *This code is written and tested using Atmel Studio 7.0(software) Windows 7 (operating system) version.
 *This code is a programm for AVR ATMega16 working on a STK500 development board.
 *Pulse With Modulation (PWM) ++ hardware.
 *More details about this code at Αναφορά-Milestone1.pdf.
 *Explanation of this code's functionality at README file.
 *********************************************************************/
#include <avr/delay.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "pcm_sample.h"
#include <avr/interrupt.h>
#define SAMPLE_RATE 8000;
#define F_CPU 4000000UL
#define Y 80
volatile uint16_t sample;
int sample_count;

void main(void)
{
	pwm_init();
	while(1){
		DDRB = 0xFF;
		PORTA |= (1 << PA3);
	}//do nothing
}
/* initialise the PWM */
void pwm_init(void)
{
    /* use OC1A pin as output */
    DDRD = _BV(PD5);

    /*
    * clear OC1A on compare match
    * set OC1A at BOTTOM, non-inverting mode
    * Fast PWM, 8bit
    */
    TCCR1A = _BV(COM1A1) | _BV(WGM10);
   
    /*
    * Fast PWM, 8bit
    * Prescaler: clk/1 = 8MHz
    * PWM frequency = 8MHz / (255 + 1) = 31.25kHz
    */
    TCCR1B = _BV(WGM12) | _BV(CS10);
   
    /* set initial duty cycle to zero */
    OCR1A = 0;
   
    /* Setup Timer0 */
 
    TCCR0|=(1<<CS00);
    TCNT0=0;
    TIMSK|=(1<<TOIE0);
    sample_count = 1;
    sei(); //Enable interrupts
}



ISR(TIMER0_OVF_vect)
{

		 
		 PORTA |= (1 << PA2);

		 if (bit_is_clear( PINA, PA2))
		 {


         sample_count--;
         if (sample_count == 0)
            {
             sample_count = 1;          
             OCR1A = pgm_read_byte(&pcm_samples[sample++]);
			 if(sample>pcm_length)sample=0;

			 if(pcm_samples[sample] < 50)
			 {
			 PORTB = 0b11111110;
			 }
			 else if(pcm_samples[sample] < 100)
			 {
				 PORTB = 0b11111100;
			 }
			  else if(pcm_samples[sample] < 150)
			  {
				  PORTB = 0b11111000;
			  }
			   else if(pcm_samples[sample] < 200)
			   {
				   PORTB = 0b11110000;
			   }
			    else if(pcm_samples[sample] < 250)
			    {
				    PORTB = 0b11100000;
			    }
				 else if(pcm_samples[sample] < 300)
				 {
					 PORTB = 0b11000000;
				 }
				  else if(pcm_samples[sample] < 350)
				  {
					  PORTB = 0b10000000;
				  }
				   else if(pcm_samples[sample] > 400)
				   {
					   PORTB = 0b00000000;
				   }
				   else
				   {
						 PORTB = 0b11111111;
				   }
            }

		}
}


