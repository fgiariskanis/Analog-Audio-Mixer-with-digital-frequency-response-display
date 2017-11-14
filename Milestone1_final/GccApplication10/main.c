/*********************************************************************
 * Milestone1 
 *Developed by: Chousos Christos AM:2012030117
 *	        Giariskanis Fotios AM:2011030087
 *Milestone1_final:
 *This code is written and tested using Atmel Studio 7.0(software) Windows 7 (operating system) version.
 *This code is a programm for AVR ATMega16 working on a STK500 development board.
 *This code is a project which uses timer/counters, interrupt service routines, I/O,LEDs,Buttons.
 *In this code there are three modes(I/O DEMO MODE,LED DELAY INIT MODE,RESET MODE).
 *More details about this code at ÁíáöïñÜ-Milestone1.pdf.
 *Explanation of this code's functionality at README file.
 *********************************************************************/
#define F_CPU 4000000UL
#include <avr/wdt.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/delay.h>

//declare global arrays for two patterns
unsigned char p1[4] = { 0b00000001,
						0b00000010,
						0b00000100,
						0b00001000 };

unsigned char i=0;  //Pattern counter  

void main (void)
{
	io_init(); //Hardware Init
	init_interrupt();//Interrupt Init
	while(1){led_use();}; //Demo I/O use 
}


void io_init()
{

DDRB = 0xFF;                //PB as output
PORTB= 0x00;                //keep all LEDs off

DDRA = 0x00;                //PA as input
PORTA |= 0b00000011;        //enable pull ups for
							//only first two pins
}

void led_use()
{
		if((PINA & 0b00000011)==0) //if PA0 && PA1 
		{
			wdt_enable (WDTO_2S);//set WatchDog Timer for 2s
			init_timer();//Init Timer, whos Interrupt use some Leds
		}
		
}

void init_timer() // Init Timer for Time = FCPU/1024
{
 // Prescaler = FCPU/1024
 TCCR0|=(1<<CS02)|(1<<CS00);
 //Enable Overflow Interrupt Enable
 TIMSK|=(1<<TOIE0);
 //Initialize Counter
 TCNT0=0;
 //Enable Global Interrupts
 sei();

}

void init_timer1() // Init Timer with prescaler = 64 and CTC mode
{
    // set up timer with prescaler = 64 and CTC mode
    TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);
    // initialize counter
    TCNT1 = 0;
    // initialize compare value
    OCR1A = 24999;
    // enable compare interrupt
    TIMSK |= (1 << OCIE1A);
    // enable global interrupts
    sei();
}

ISR(TIMER0_OVF_vect) //Interruption that use some Leds every Timer0 OVF
{
		PORTB=p1[i];    //output data
		if(i==3){i=0;}		//pattern counter
		else {i++;}
}

ISR(TIMER1_COMPA_vect)//Interruption that Inits Timer0 once at Timer1 COMPA    ----->   (TIMER0 WITH DELAY)
{
	init_timer();			//Timer0 that uses Leds
	TCCR1B &= 0B11111000;		//Stop Timer1
}

ISR(INT1_vect)//INIT TIMERS INTERRUPT
{//Interruption that comes with PD3(INT1) and check if PA0 button or PA1 button is pressed so it uses some Leds either with no delay, either with delay
		if((PINA & 0b00000010)==0)	{init_timer1();}//init Timer1
		else if((PINA & 0b00000001)==0){init_timer();}//init Timer0
}

ISR(INT0_vect)//RESET INTERRUPT
{//Interruption that comes with PD2(INT0) and check if PA0 button or PA1 button is pressed so it either resets the output port and stop timer0, either it enables the watchdog timer
	if((PINA & 0b00000001)==0){PORTB = 0;  TCCR0 &= 0B11111000;}//Clear Leds AND Reset Timer0
	else if ((PINA & 0b00000010)==0){wdt_enable (WDTO_2S);}//Enable WatchDog Timer
}



void init_interrupt()//enable Interrupts for ports PD2,PD3
{
	// disable external interrupts while initializing
	cli();
	//INIT INTERRUPT VALUES
	GICR =(1 << INT1 | 1 << INT0);
	MCUCR= 1<<ISC10 | 1<<ISC11;
	MCUCR= 1<<ISC00 | 1<<ISC01;
	// enable external interrupts
	sei();
}
