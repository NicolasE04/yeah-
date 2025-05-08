// da5_02
// nicolas evangelista

// Task 02 Objective:
// Connect the encoder pins to Timer1 to calculate the
// init motor
#define F_CPU 16000000UL
#define BAUDRATE 57600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define PULSES_PER_REV 1000

#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>

// Global variables
char buffer[5]; //Output of the itoa function
volatile uint16_t pulse_count = 0;
volatile uint16_t pulses_per_second = 0;

// init adc
void adc_init(void){
	// Prescalar = 64
	ADCSRA |= ((1 << ADPS2) | (1 << ADPS1));
	
	// Voltage reference from Avcc (5v)
	ADMUX |= (1<<REFS0);
	
	// init adc
	ADCSRA |= (1<<ADEN);
}

// Function to read a channel
// init adc
uint16_t read_adc(uint8_t channel){
	ADCSRA |= (1 << ADSC);

	while (ADCSRA & (1 << ADSC));
	return ADC;
}

unsigned char usart_receive(void) {
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void usart_transmit(char c) {

	while (! (UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void usart_getline(char* buffer, uint8_t n) {
	uint8_t i = 0;	
	char c;			

	do {
		c = usart_receive();
		buffer[i++] = c;
	}
	while((i < n) && (c != '\r'));
	buffer[i] = 0;
}

void usart_sendstring(char *word) {
	
	while(*word != '\0') {
		usart_transmit(*word);	
		word++;					
	}
}

void usart_init() {
	// Set baud rate
	UBRR0H = (BAUD_PRESCALER >> 8);
	UBRR0L = BAUD_PRESCALER;

	// transmit
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);

	// Asynchronous (00), no parity (00), 1 stop bit (0), 8-bit data (011)
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// pwm waveform
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


void input_capture_init() {
	// Set PB0 input
	DDRB &= ~(1 << DDB0);

	// Normal mode
	TCCR1A = 0;

	// No prescaler (1x speed), rising edge initially
	TCCR1B = (1 << ICES1) | (1 << CS10);

	// Enable Input Capture interrupt
	TIMSK1 = (1 << ICIE1);


	// Initialize Timer1 counter
	TCNT1 = 0;

	// Enable global interrupts
	sei();
}

ISR(TIMER1_CAPT_vect) {
	pulse_count++;
}

// count pulses
void update_pulse_rate() {
	pulses_per_second = pulse_count;
	pulse_count = 0;
}

int main(void){
	adc_init();
	usart_init();
	pwm_init();
	motor_init();
	input_capture_init();

	while(1) {
		uint16_t adc_value = read_adc(0);
		uint8_t pwm_value = (adc_value * 255UL) / 1023;

		// init motor
		OCR0A = pwm_value;

		_delay_ms(1000);
		update_pulse_rate();
		
		usart_sendstring("RPM: ");

		double rpm = pulses_per_second / 2.5;
		
		itoa(rpm, buffer, 10);

		usart_sendstring(buffer);
		usart_transmit('\n');
	}
}