// program name: LED VU meter
// date: 
// target device: ATMEGA16
// author: 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>	
#include <avr/delay.h>	
//#include "ffft.h"		/* Defs for using Fixed-point FFT module */
//#include "lookup.h"

#define F_CPU 16000000UL

#define N 6
#define LEDs PORTB
#define LEDsR PORTD
#define SpecLeds PORTC
#define PeakPort PORTC

#define RED 0b00000001
#define GREEN 0b01000000
#define BLUE 0b10000000
//#define    LED_RED_P           PC4
//#define    LED_GREEN_P         PC1
//#define    LED_BLUE_P          PC5
#define    BASS_THRESHOLD_P      (400)
#define    MIDRANGE_THRESHOLD_P  (130)
#define    TREBLE_THRESHOLD_P    (110)
/////////////////////////////////////////
#define    LED_RED             PC1
#define    LED_GREEN           PC6
#define    LED_BLUE            PC7
#define    PeakLed             PC0

#define    BASS_THRESHOLD      (30000)
#define    MIDRANGE_THRESHOLD  (80)
#define    TREBLE_THRESHOLD    (80)

#define    N                   (6)

const int8_t W[N] = {10, 4, -5, -9, -4, 5};
int16_t samples[N] = {0};
int16_t re[N];
volatile uint8_t acquisition_cnt = 0;
//////////////////////////////////////

unsigned char adc_data, adc_data_old, adc_data_ola, adc_dataP, adc_data_oldP, adc_data_olaP;
int counter = 0, count =0, ready= 0, ready1= 0, ready2=0, counterP=0;
unsigned char adc_dataR, adc_data_oldR, adc_data_olaR;
int counterR = 0;
//volatile uint8_t count;
const unsigned char level[9] = {0b11111111,0b01111111,0b00111111,0b00011111,0b00001111,0b00000111,0b00000011,0b00000001,0b00000000};

/*void adc_init();
uint16_t adc_read_new();
void TRANSFORM();
void timer1_init();
	
int32_t fx[N];
int32_t Fu[N/2][2];
int fft_buf [N/2], bass = 0, mid = 0,high =0;
*/

/*------------------------------------------------*/
/* Global variables                               */


//char pool[16];	/* Console input buffer */

 //int16_t capture[FFT_N];			/* Wave captureing buffer */
 //complex_t bfly_buff[FFT_N];		/* FFT buffer */
// uint16_t spektrum[FFT_N/2];		/* Spectrum output buffer */

//const unsigned char level[9] = {0b11111111,0b01111111,0b00111111,0b00011111,0b00001111,0b00000111,0b00000011,0b00000001,0b00000000};


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
//ADCSRA = ADCSRA | 0x40;
//adc_init();

DDRB = 0xFF; //portB as output
DDRC = 0xFF; //portC as output
DDRD = 0xFF; //portD as output


//TCCR1B = 3;	/* clk/64 */
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
 DDRC |= _BV(LED_BLUE)|_BV(LED_GREEN)|_BV(LED_RED)|_BV(PeakLed); // set LED pins as OUTPUT
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

	//adc_init();
	//timer1_init();

	while(1){
		VU();
		//Spec();
		//Spec2();
		SpecRGB();
		//peak();
		}
	return 1;

}

/*
void Spec2(void){

 uint8_t mag;
 int i,j, temp_value;
 uint8_t temp_index;

 TCNT1 = 0;
 TIFR |= 1<<OCF1A;
 for(i=0;i<N;i++) {
	 while((TIFR & (1<<OCF1A)) == 0);
	 fx[i] = ((int16_t)adc_read_new());
	 TIFR |= 1<<OCF1A;
 }
 TRANSFORM();
 for(i =1; i<N/2; i++) {
	 if(Fu[i][0]<0)Fu[i][0]*=-1;
	 if(Fu[i][1]<0)Fu[i][1]*=-1;
	 mag = (uint8_t)(Fu[i][0] + Fu[i][1])/2;
	 fft_buf[i] = (mag);// - 7 - 1;

	 	if (i <8 && mag>1){
		 	bass ++;
	 	}
		else{
			//SpecLeds = 0x00;
		}
	 	 if (i <14 && mag>3){
		 	mid ++;
	 	}
		else{
			//SpecLeds = 0x00;
		}
	 	if (i <16 && mag>7){
		 	high ++;
	 	}
	 	else{
		 	//SpecLeds = 0x00;
	 	}
 }

 	if (high == 0 && mid == 0 && bass == 0 ) {
	 	SpecLeds = 0x00;
	 	bass =0;
	 	mid = 0;
	 	high = 0;
 	}
 if(high >0 || mid>0 || bass>0){
	if (bass >= mid && bass >= high  && bass >=bass/2){
		SpecLeds |= RED;
		high=0;
		mid=0;
		bass=0;
		//SpecLeds =~ GREEN;
		//SpecLeds =~ BLUE;
	//	_delay_ms(10);
		//SpecLeds = 0x00;
	}
	 if (mid >= bass && mid >= high && mid >=mid/2){
		SpecLeds |= GREEN;
		high=0;
		mid=0;
		bass=0;
				//SpecLeds =~ RED;
				//SpecLeds =~ BLUE;
		//				_delay_ms(10);
						//SpecLeds = 0x00;
	}
	 if (high >= mid && high >= bass  && high >=high/2){
		SpecLeds |= BLUE;
		high=0;
		mid=0;
		bass=0;
				//SpecLeds =~ GREEN;
				//SpecLeds =~ RED;
			//			_delay_ms(10);
						//SpecLeds = 0x00;
	}

	}
	//SpecLeds =~SpecLeds;
 }



 void TRANSFORM()
{
    int16_t count,degree;
    uint8_t u,k;
    count = 0;
    for (u=0; u<N/2; u++) {
        for (k=0; k<N; k++) {
            degree = (uint16_t)pgm_read_byte_near(degree_lookup + count)*2;
            count++;
            Fu[u][0] +=  fx[k] * (int16_t)pgm_read_word_near(cos_lookup + degree);
            Fu[u][1] += -fx[k] * (int16_t)pgm_read_word_near(sin_lookup + degree);
        }
        Fu[u][0] /= N;
        Fu[u][0] /= 10000;
        Fu[u][1] /= N;
        Fu[u][1] /= 10000;
    }
}

void timer1_init()
{
    //TCCR1B = 3;
    TCCR1B = (1<<WGM12)|(1<<CS10);
    OCR1A = 1;
}

void adc_init()
{
    ADMUX = 0b11000000;
    ADCSRA =0b10000010;
}

uint16_t adc_read_new()
{
    volatile uint16_t retl,reth;
    ADCSRA |= 1<<ADSC;
    while(!ADIF);
    ADCSRA |= 1<<ADIF;
    retl = ADCL;
    reth = ADCH;
    reth<<=8;
    reth|=retl;
    return reth;
}

/*


/*
void Spec(void) {
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ FFT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	
	uint16_t t1,t2,t3;
	uint16_t m, n, s;

capture_wave(capture, FFT_N);
TCNT1 = 0;	// performance counter 
fft_input(capture, bfly_buff);
t1 = TCNT1; TCNT1 = 0;
fft_execute(bfly_buff);
t2 = TCNT1; TCNT1 = 0;
fft_output(bfly_buff, spektrum);
t3 = TCNT1;
for (n = 0; n < FFT_N / 2; n++) {
	s = spektrum[n];
	s /= 512;
			SpecLeds = RED;
			_delay_ms(100);
			SpecLeds = 0x00;
			_delay_ms(100);
			SpecLeds = GREEN;
			_delay_ms(100);
			SpecLeds = 0x00;
			_delay_ms(100);
			SpecLeds = BLUE;
			_delay_ms(100);
			SpecLeds = 0x00;
			_delay_ms(100);


	
}

}

*///SPEC2


void SpecRGB(void){

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
			//PORTC ~= PORTC;
        }
}

void peak(void){
adc_dataP = read_adc();
adc_data_olaP += adc_dataP;
counterP ++;
if(ready2==1){
	adc_data_oldP = adc_data_olaP/counterP;
	adc_data_oldP = adc_data_oldP/15;
		if(adc_data_oldP >= 0.001){
			PORTC |= _BV(PeakLed);

		}
		else{
			PORTC &=  ~_BV(PeakLed);
		}
		ready2=0;
		adc_data_olaP = 0;
		counterP = 0;
}

}

void VU(void) {
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Left Channel (0) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
adc_data = read_adc();
//adc_data = readadchan(0);
adc_data_ola += adc_data;
counter ++;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Right Channel (1) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
adc_dataR = read_adc1();
//adc_dataR = readadchan(1);
adc_data_olaR += adc_dataR;
counterR ++;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PRINTING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if(ready==1 && ready1 ==1){

adc_data_old = adc_data_ola/counter;
adc_data_old = adc_data_old/15;
adc_data_oldR = adc_data_olaR/counterR;
adc_data_oldR = adc_data_oldR/15;

LEDs =~ level[adc_data_old];
LEDsR =~ level[adc_data_oldR];

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ RESETING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ready= 0;
ready1= 0;
adc_data_ola = 0;
counter = 0;
adc_data_olaR = 0;
counterR = 0;
}
//CLASIKO ADC//////////
		//adc_data = read_adc();

		//adc_data = adc_data/5;
		//LEDs = level[adc_data];
		
		//_delay_ms(0.1);
//////////////////////
}


 

/*------------------------------------------------*/
/* Capture waveform                               */

void capture_wave ( int16_t *buffer,  uint16_t count)
{
	ADMUX = _BV(REFS0)|_BV(ADLAR)|_BV(MUX2)|_BV(MUX1)|_BV(MUX0);	// channel

	do {
		ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADIF)|_BV(ADPS2)|_BV(ADPS1);
		while(bit_is_clear(ADCSRA, ADIF));
		*buffer++ = ADC - 32768;
	} while(--count);

	ADCSRA = 0;
}