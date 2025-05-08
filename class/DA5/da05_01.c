// nicolas Evangelista

#include <avr/io.h>

#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

// init adc
void adc_init(void){
	// Prescalar = 64
	ADCSRA |= ((1 << ADPS2) | (1 << ADPS1));
	
	// Voltage reference from Avcc (5v)
	ADMUX |= (1<<REFS0);
	
	// init adc
	ADCSRA |= (1<<ADEN);
}

// Function to read an arbitrary analogic channel/pin
// init adc
uint16_t read_adc(uint8_t channel){

	ADCSRA |= (1 << ADSC);

	while (ADCSRA & (1 << ADSC));
	return ADC;
}


// Function to initialize and configure the USART/serial
void uart_init(void){
	UBRR0H = (uint8_t)(BAUD_PRESCALLER >> 8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (3 << UCSZ00); 
}

// Function that sends a char over the serial port
void uart_send( unsigned char data){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

// send string
void uart_putstring(char* StringPtr){
	while(*StringPtr != 0x00){
		UART_send(*StringPtr);
	StringPtr++;}
}

void pwm_init(void) {
	// OC0A as output
	DDRD |= (1 << PD6);

	// init pwm
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);

	// Prescaler 64
	TCCR0B = (1 << CS01) | (1 << CS00);
}

// init motor
void motor_init(void) {
	// Determines spin direction
	DDRD |= (1 << PD4) | (1 << PD5); 

	// set direction
	PORTD |= (1 << PD4);
	PORTD &= ~(1 << PD5);
}

int main(void){
	adc_init();
	uart_init();
	pwm_init();
	motor_init();

	while(1) {
		uint16_t adc_value = read_adc(0);
		uint8_t pwm_value = (adc_value * 255UL) / 1023;

		// init motor
		OCR0A = pwm_value;
	}
}