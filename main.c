/*
 * main.c
 *
 * Created: 1/5/2025 6:06:19 PM
 *  Author: smilexs4
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdio.h>

#define setPoint 800.0f

// Define the PI controller variables
float Kp;         // Proportional gain
float Ki;         // Integral gain
//float setPoint;   // Desired target value
float currentValue; // Current measured value
float error;      // Error between setPoint and currentValue
float integral;   // Integral of the error
float controlSignal; // Output control signal
float dt;         // Time step (in seconds)

#define SAMPLE_COUNT 100
volatile uint16_t samples[SAMPLE_COUNT], sampleIndex = 0;

void USART_Init(unsigned int ubrr) {
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;

	// Enable transmitter
	UCSR0B = (1 << TXEN0);

	// Set frame format: 8 data bits, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_Transmit(char data) {
	// Wait for empty transmit buffer
	while (!(UCSR0A & (1 << UDRE0)));

	// Put data into buffer, sends the data
	UDR0 = data;
}

void USART_Print(const char *str) {
	while (*str) {
		USART_Transmit(*str++);
	}
}

void ADC_Init(void) {
	// Configure ADC
	ADMUX |= (1 << REFS0);  // Set reference voltage to AVcc
	ADMUX &= ~(1 << ADLAR); // Right adjust the ADC result
	ADMUX |= (0 << MUX0);   // Select ADC0 as input channel

	ADCSRA |= (1 << ADEN);  // Enable the ADC
	ADCSRA |= (1 << ADIE);  // Enable ADC interrupt
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set prescaler to 128

	ADCSRA |= (1 << ADSC);  // Start the first conversion
}


void pwm_init() {
	DDRB |= (1 << PB3); //pin 11 COM2A output

	// Timer 2 activation
	PRR &= ~(1 << PRTIM2);

	// Timer/Counter2 is clocked from the I/O clock
	ASSR &= ~(1 << AS2);

	//Set Initial Timer value
	TCNT2 = 0;

	// Output compare register A value
	OCR2A = 255;

	//Set on compare match, clear on bottom inverting mode
	TCCR2A |= (1 << COM2A1);
	TCCR2A |= (1 << COM2A0);

	//Set fast PWM mode 3 TOP = 255
	TCCR2A |= (1 << WGM21) | (1 << WGM20); //fast pwm to 255

	// Set prescaller 1024 and start timer
	TCCR2B |= (1 << CS22);
	TCCR2B |= (1 << CS21);
	TCCR2B |= (1 << CS20);
}

void Timer1_Init_10ms() {
    // Set Timer1 to CTC mode
    TCCR1B |= (1 << WGM12); // WGM12: CTC mode
    
    // Set prescaler to 64
    TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64
    
    // Calculate and set OCR1A for 10 ms
    // Timer frequency = F_CPU / Prescaler
    // OCR1A = (Timer frequency * Desired Time) - 1
    // OCR1A = ((16000000 / 64) * 0.01) - 1 = 2499
    OCR1A = 2499;

    // Enable Timer1 Compare Match A Interrupt
    TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
	char buffer[50];
	
	if (sampleIndex >= SAMPLE_COUNT) {
		
		// Compute mean value of samples
		unsigned long sum = 0;
		for (int i = 0; i < SAMPLE_COUNT; i++) {
			sum += samples[i];
		}
        uint16_t currentValue = sum / SAMPLE_COUNT;
        
        // Compute error
        float error = setPoint - currentValue;

        // Compute integral term
        integral += error * dt;

        // Compute control signal
        controlSignal = Kp * error + Ki * integral;
		
		// Adjust and limit timer value on 8-bits
		float timerValue = 255 - controlSignal / 4;
		if (timerValue < 0) {
			timerValue = 0;
		}
		if (timerValue > 255) {
			timerValue = 255;
		}
		
		// Update timer register
		OCR2A = (uint8_t) timerValue;
		
		snprintf(buffer, sizeof(buffer), "S=%.2f; I=%d; O=%.2f;\r\n", setPoint, currentValue, controlSignal);
		USART_Print(buffer);   // Send ADC value to serial
		
		sampleIndex = 0;
		ADCSRA |= (1 << ADSC); // Start the next conversion
	}
}


int main(void) {
	// Ports
	DDRB |= (1 << PB3);
	// DDRB |= (1 << 5);
	
	// UART
	USART_Init(103); // Initialize USART with 9600 baud rate (assuming 16 MHz clock)
	
	// ADC
	ADC_Init();
	
	// PWM
	pwm_init();
	
	// PID Loop
	Timer1_Init_10ms();
	
    Kp = 0.01;       // Placeholder value for proportional gain
    Ki = 5.0;       // Placeholder value for integral gain
    dt = 0.01;      // Time step (10 ms as an example)
    integral = 0;   // Initialize the integral term
	
	sei();
	
	ADCSRA |= (1 << ADSC); // Start the next conversion
	
    while(1) {
		
    }
}


ISR(ADC_vect) {
	samples[sampleIndex] = ADCW;
	sampleIndex++;
	if (sampleIndex < SAMPLE_COUNT) {
		ADCSRA |= (1 << ADSC); // Start the next conversion
	}
}
