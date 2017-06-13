/* */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define PIN_UART_TX		PB0

static void init_hardware(void)
{
	// enable UART output pin
	DDRB = (1 << PIN_UART_TX);

	// set sleep-mode to idle so TIMER0 can still wake us up
	SMCR = 0;

	sei();
}

#define UART_LOGIC_HIGH()	PORTB &= ~(1 << PIN_UART_TX)
#define UART_LOGIC_LOW()	PORTB |= (1 << PIN_UART_TX)
#define UART_BAUD		(1220ULL)
#define UART_BIT_TIME		((F_CPU / 1ULL) / (UART_BAUD))

static void uart_tx(char c)
{
	char tmp;

	// start transmitter timer0 (OCR0A)
	OCR0A = UART_BIT_TIME;
	TCCR0A = 0;
	TCCR0C = 0;
	TCNT0 = 0;
	TIMSK0 = (1 << OCIE0A);// enable OCR0A interrupt
	TCCR0B = (1 << WGM02) | 0b001;
			// Clear timer on compare (OCR0A),
			// TimerCLK is IoCLK

	// start bit
	UART_LOGIC_LOW();

	// data bits
	for(tmp = 0x01; tmp != 0; tmp <<= 1) {
		sleep_mode();
		if(c & tmp)
			UART_LOGIC_HIGH();
		else
			UART_LOGIC_LOW();
	}

	// even parity bit
	tmp = c;
	tmp ^= tmp >> 4;
	tmp ^= tmp >> 2;
	tmp ^= tmp >> 1;
	sleep_mode();
	if(tmp & 1)
		UART_LOGIC_HIGH();
	else
		UART_LOGIC_LOW();

	// stop bit 1, stop bit 2, and 4 more tail
	for(tmp = 6; tmp != 0; --tmp) {
		sleep_mode();
		UART_LOGIC_HIGH();
	}

	// disable transmitter timer0
	TCCR0B = 0;
}

int main(void)
{
	init_hardware();

	char i = '0';

	uart_tx('S');
	uart_tx('\n');

	while(1) {

		uart_tx('i');
		uart_tx(i);
		uart_tx('\n');
		++i;
		if(i > '9')
			i = '0';
	}
}

ISR(TIM0_COMPA_vect)
{
	return;
}

