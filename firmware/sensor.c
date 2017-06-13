/* */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define PIN_UART_TX		PB0

#define UART_LOGIC_HIGH()	PORTB &= ~(1 << PIN_UART_TX)
#define UART_LOGIC_LOW()	PORTB |= (1 << PIN_UART_TX)
#define UART_BAUD		(1220ULL)
#define UART_BIT_TIME		((F_CPU / 1ULL) / (UART_BAUD))

static void start_timer0(void)
{
	TCCR0A = 0;
	TCCR0C = 0;
	TCNT0 = 0;
	// enable OCR0A interrupt
	TIMSK0 = (1 << OCIE0A);

	// set mode to Clear timer on compare (OCR0A)
	// set prescaler flags
	TCCR0B = (1 << WGM02) | 0b001;
}

static inline void stop_timer0(void)
{
	TCCR0B = 0;
}

static void uart_tx(char c)
{
	char tmp;

	// enable timer for specified baudrate
	OCR0A = UART_BIT_TIME;
	start_timer0();

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

	stop_timer0();
}

#define PIN_I2C_SCL		PB1
#define PIN_I2C_SDA		PB2

#define I2C_BAUD		(10000ULL)
#define I2C_BIT_TIME		((F_CPU / 1ULL) / (I2C_BAUD))

/*
static uint8_t i2c_XXXXXX(uint8_t address)
{
	// enable timer for i2c timings
	OCR0A = I2C_BIT_TIME;
	start_timer0();
}
*/

int main(void)
{
	// enable UART output pin
	DDRB = (1 << PIN_UART_TX) | (1 << PIN_I2C_SCL) | (1 << PIN_I2C_SDA);

	// set sleep-mode to idle so TIMER0 can still wake us up
	SMCR = 0;

	sei();

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

