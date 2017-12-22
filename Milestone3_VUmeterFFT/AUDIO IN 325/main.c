
/*********************************************************************
 * Milestone3 
 * Developed by: Chousos Christos AM:2012030117
 *	             Giariskanis Fotios AM:2011030087
 *Milestone3_final:
 *This code is written and tested using Atmel Studio 7.0(software) Windows 7 (operating system) version.
 *This code is a programm for AVR ATMega16 working on a STK500 development board.
 *Analog mixer ++ FFT ++ VU METER ++ Watchdog Timer ++ Pulse With Modulation (PWM).
 *More details about this code at Αναφορά-Milestone1.pdf.
 *Explanation of this code's functionality at README file.
 *********************************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>	
#include <avr/delay.h>	
////////////////CLOCK////////////////
#define F_CPU 16000000UL
////////////LEDs VU RGB ////////////
#define		LEDs PORTB
#define		LEDsR PORTD
#define		SpecLeds PORTC
#define     LED_RED             PC1
#define     LED_GREEN           PC6
#define     LED_BLUE            PC7
#define     BASS_THRESHOLD      (30000)
#define     MIDRANGE_THRESHOLD  (80)
#define     TREBLE_THRESHOLD    (80)
//////Fast Fourier Transform///////
#define     N                   (6)
const int8_t W[N] = {10, 4, -5, -9, -4, 5};
int16_t samples[N] = {0};
int16_t re[N];
///////////VU Variables///////////
volatile uint8_t acquisition_cnt = 0;
unsigned char adc_data, adc_data_old, adc_data_ola;
int counter = 0, count =0, ready= 0, ready1= 0, ready2=0;
unsigned char adc_dataR, adc_data_oldR, adc_data_olaR;
int counterR = 0;
const unsigned char level[9] = {0b11111111,0b01111111,0b00111111,0b00011111,0b00001111,0b00000111,0b00000011,0b00000001,0b00000000};


// this function read the value of ADC channel 0
unsigned char read_adc(void)
{
	ADMUX = 0b00100000; // set ADC0
	ADCSRA |= 1<<ADSC;  //start conversion;
	while (ADCSRA&(1<<ADSC)); //wait conversion end
	ready =1;
	return ADCH;
}

unsigned char read_adc1(void)
{
	ADMUX = 0b00100001; // set ADC1
	ADCSRA |= 1<<ADSC;  //start conversion;
	while (ADCSRA&(1<<ADSC)); //wait conversion end
	ready1 =1;
	return ADCH;
}

unsigned char read_adc2(void)
{
	ADMUX = 0b00100010; // set ADC2
	ADCSRA |= 1<<ADSC;  //start conversion;
	while (ADCSRA&(1<<ADSC)); //wait conversion end
	ready2 =1;
	return ADCH;
}

void init(void){
	//adc init
	ADMUX = 0b00100000; // set ADC0
	ADCSRA = 0b10000111; //set ADEN, precale by 128
	DDRB = 0xFF; //portB as output
	DDRC = 0xFF; //portC as output
	DDRD = 0xFF; //portD as output
}

ISR(ADC_vect)
{
	uint8_t high, low;
	if (acquisition_cnt < N) {
		low = ADCL;
		high = ADCH;
		samples[acquisition_cnt] = (high << 8) | low;
		acquisition_cnt++;
		ADCSRA |= _BV(ADSC);
	}
}

static void dft(void)
{
	uint8_t a, k, n;

	for (k = 0; k < N; ++k)
	re[k] = 0;

	for (k = 0; k <= (N>>1); ++k) {
		a = 0;
		for (n = 0; n < N; ++n) {
			re[k] += W[a%N] * samples[n];
			a += k;
		}
		if (re[k] < 0)
		re[k] = -(re[k] + 1);
	}
}

void initRGB (void) {
	/* setup */
	DDRC |= _BV(LED_BLUE)|_BV(LED_GREEN)|_BV(LED_RED); // set LED pins as OUTPUT
	ADCSRA |= _BV(ADPS2)|_BV(ADPS1); // set ADC division factor 64;
	ADCSRA |= _BV(ADEN)|_BV(ADIE); // ADC interrupt enable
	ADMUX = 0b00100000; //_BV(MUX1); // set PA0 as audio input
	sei(); // enable global interrupts
	ADCSRA |= _BV(ADSC); // Start first signal acquisition
}

// main program
int main (void)
{
	init();
	initRGB();
	while(1){
		VU();
		SpecRGB();
	}
	return 1;
}


void SpecRGB(void)//Frequency Spectrogram to RGB Led
{ 
        if (acquisition_cnt == N) {
            dft();

            /* LED RED - Bass */
            if (re[0] >= BASS_THRESHOLD) {
                PORTC |= _BV(LED_RED);
            } else {
                PORTC &= ~_BV(LED_RED);
            }

            /* LED GREEN - Midrange */
            if (re[1]/150 > MIDRANGE_THRESHOLD) {
                PORTC |= _BV(LED_GREEN);
            } else {
                PORTC &= ~_BV(LED_GREEN);
            }

            /* LED BLUE - Treble */
            if (re[2]/280 > TREBLE_THRESHOLD) {
                PORTC |= _BV(LED_BLUE);
            } else {
                PORTC &= ~_BV(LED_BLUE);
            }

            /* Trigger next signal acquisition */
            acquisition_cnt = 0;
            ADCSRA |= _BV(ADSC);
        }
}


void VU(void) {//VU Meter
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Left Channel (0) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
adc_data = read_adc();
adc_data_ola += adc_data;
counter ++;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Right Channel (1) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
adc_dataR = read_adc1();
adc_data_olaR += adc_dataR;
counterR ++;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PRINTING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if(ready==1 && ready1 ==1){ //ADC0 && ADC1 ready

adc_data_old = adc_data_ola/counter;
adc_data_old = adc_data_old/15;
adc_data_oldR = adc_data_olaR/counterR;
adc_data_oldR = adc_data_oldR/15;

LEDs =~ level[adc_data_old*2];
LEDsR =~ level[adc_data_oldR*2];

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ RESETING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ready= 0;
ready1= 0;
adc_data_ola = 0;
counter = 0;
adc_data_olaR = 0;
counterR = 0;
}

}

 

