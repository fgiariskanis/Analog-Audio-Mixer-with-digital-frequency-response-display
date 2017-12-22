unsigned int addat[8];
//--------------------
void adc_init(void){
	//ADC initialize

	ADCSR = 0x00; //disable adc
	ADMUX = 0x00; //select adc input 0
	ADCSR = 0x87;
}

//---------------------
unsigned int readadchan(char ch){
	//read channel ch of internal 10 bit a/d

	ADMUX= ch;        //channel 0
	ADCSRA |= 0x40;   //init conversion
	while((ADCSRA & 0x40) !=0){}; //wait for conv complete
	return ADC;
}

//--------------------
void readadloop(void){
	//read internal 10 bit a/d  5mv per bit
	char c,i;

	printf("a/ds \n");
	c=0;
	while(c != 'q'){
		if(kbhit()){
			c=getchar();
		}
		for(i=0; i<8; i++){
			addat[i]=readadchan(i);
			printf("%4d ",addat[i]);
		}//for i
		crlf();
	}//while
}