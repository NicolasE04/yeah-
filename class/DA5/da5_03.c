// nicolas evangelista

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
volatile uint8_t override_rpm = 0;

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

void usart_sendstring(char *word) {
	while(*word != '\0') {
		usart_transmit(*word);	
		word++;					
	}
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

	// No prescaler
	TCCR1B = (1 << ICES1) | (1 << CS10);

	// Enable Input Capture interrupt
	TIMSK1 = (1 << ICIE1);


	// Initialize Timer1 counter
	TCNT1 = 0;

	// Enable global interrupts
	sei();
}

// count pulses
void update_pulse_rate() {
	pulses_per_second = pulse_count;
	pulse_count = 0;
}

ISR(TIMER1_CAPT_vect) {
	// Each time an edge is captured, increment the count
	pulse_count++;
}

void onesec_init(void) {
	// Set CTC mode (WGM32 = 1, WGM33 = 0)
	TCCR3A = 0;				// Normal port operation
	TCCR3B = (1 << WGM32);	// CTC mode
	TCCR3B |= (1 << CS32) | (1 << CS30);  // Prescaler = 1024

	// compare match
	OCR3A = 15624;

	// compare match
	TIMSK3 |= (1 << OCIE3A);
}

// print rpm
ISR(TIMER3_COMPA_vect) {
	char print_rpm[10];
	update_pulse_rate();
		
	usart_sendstring("RPM: ");

	double rpm = pulses_per_second / 2.33;
	
	itoa(rpm, print_rpm, 10);

	//Send the converted value to the terminal
	usart_sendstring(print_rpm);
	usart_transmit('\n');
}

void timer4_init(void) {
	// Set CTC mode (WGM42 = 1, WGM43 = 0)
	TCCR4A = 0;                    // Normal port operation
	TCCR4B = (1 << WGM42);         // CTC mode, TOP = OCR4A
	TCCR4B |= (1 << CS41) | (1 << CS40);  // Prescaler = 64

	OCR4A = 12499;                 // compare match

	// compare match
	TIMSK4 |= (1 << OCIE4A);
}

// init adc
ISR(TIMER4_COMPA_vect) {
	if(override_rpm == 0) {
		uint16_t adc_value = read_adc(0);
		uint8_t pwm_value = (adc_value * 255UL) / 1023;

		// init motor
		OCR0A = pwm_value;
	}
}

int main(void){
	adc_init();
	usart_init();
	pwm_init();
	motor_init();
	input_capture_init();
	onesec_init();
	timer4_init();

	while(1) {
		usart_getline(buffer, 5);	// Get user input in terminal
		override_rpm = atoi(buffer);

		// Check for value in range
		if ((override_rpm < 0) || (override_rpm >= 150)) {
			override_rpm = 0;
			usart_sendstring("Invalid RPM. Range is 0-150\n");
		}
		else {
			override_rpm = override_rpm * 1.96;
			OCR0A = override_rpm;
		}
	}
}