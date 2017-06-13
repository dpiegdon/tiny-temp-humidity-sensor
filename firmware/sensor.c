/* */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// Software UART transmitting at 1200 BAUD 8E2 (8 bits even parity 2 stop bits)
#define UART_BUF_SIZE	4
static char uart_buffer[UART_BUF_SIZE];
static uint8_t uart_current = 0;
enum {
	UART_TX_IDLE		= 0,
	UART_TX_PREPARE		= 1,
	UART_TX_START_BIT	= 2,
	UART_TX_BIT0		= 3,
	UART_TX_BIT1		= 4,
	UART_TX_BIT2		= 5,
	UART_TX_BIT3		= 6,
	UART_TX_BIT4		= 7,
	UART_TX_BIT5		= 8,
	UART_TX_BIT6		= 9,
	UART_TX_BIT7		= 10,
	UART_TX_PARITY		= 11,
	UART_TX_STOP1		= 12,
	UART_TX_STOP2		= 13,
	UART_TX_TAIL1		= 14,
	UART_TX_TAIL2		= 15,
	UART_TX_TAIL3		= 16,
	UART_TX_TAIL4		= 17,
} uart_tx_state = UART_TX_IDLE;
static uint8_t uart_end = 0;
#define UART_IS_EMPTY()		(uart_current == uart_end)
#define UART_IS_FULL()		(((uart_current + 1) % UART_BUF_SIZE) == uart_end)
/* XXX NB the baud-rate may require slight tuning
 * due to internal oscillator precision.
 * start with 1200 when re-tuning for new hardware.
 * */
#define UART_BAUD		(1220ULL)
#define UART_BIT_TIME		((F_CPU / 1ULL) / (UART_BAUD))
#define PIN_UART		PB0
#define UART_LOGIC_HIGH()	PORTB &= ~(1 << PIN_UART)
#define UART_LOGIC_LOW()	PORTB |= (1 << PIN_UART)


static void init_hardware(void)
{
	CCP = 0xD8;		// allow writes to CLKPSR
	CLKPSR = 0b0011U;	// set prescaler to /8
	CCP = 0xD8;		// allow writes to CLKPSR
	CLKMSR = 0b00U;		// select internal 8MHz oscillator

	// enable TIMER0
	PRR &= ~(1 << PRTIM0);

	// enable UART output pin
	DDRB |= (1 << PIN_UART);
	DIDR0 |= (1 << PIN_UART);
	PUEB &= ~(1 << PIN_UART);

	// set sleep-mode to idle so TIMER0 can still wake us up
	SMCR = 0;

	sei();
}

static void uart_tx(char c)
{
	while(UART_IS_FULL())
		sleep_mode(); // block until space in ringbuffer

	uart_buffer[uart_end] = c;
	uart_end = (uart_end + 1) % UART_BUF_SIZE;

	if(UART_TX_IDLE == uart_tx_state) {
		// start transmitter timer0 (OCR0A)
		OCR0A = UART_BIT_TIME;
		TCNT0 = 0;
		TCCR0A = 0;
		TCCR0C = 0;
		TIMSK0 = (1 << OCIE0A);// enable OCR0A interrupt
		TCCR0B = (1 << WGM02) | 0b001;
				// Clear timer on compare (OCR0A),
				// TimerCLK is IoCLK
		++uart_tx_state;
	}
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
	register uint8_t val = uart_buffer[uart_current];
	if(!UART_IS_EMPTY()) {
		++uart_tx_state;
		switch(uart_tx_state) {
			case UART_TX_START_BIT:
				UART_LOGIC_LOW();
				break;
			case UART_TX_PARITY:
				{
					val ^= val >> 4;
					val ^= val >> 2;
					val ^= val >> 1;
					if(val & 1)
						UART_LOGIC_HIGH();
					else
						UART_LOGIC_LOW();
				}
				break;
			case UART_TX_STOP1:
				UART_LOGIC_HIGH();
				break;
			case UART_TX_TAIL1:
				/* fall through */
			case UART_TX_TAIL2:
			case UART_TX_TAIL3:
				/* fall through */
			case UART_TX_STOP2:
				break;
			case UART_TX_TAIL4:
				uart_tx_state = UART_TX_PREPARE;
				uart_current = (uart_current + 1) % UART_BUF_SIZE;
				break;
			default:
				/* transmit current bit */
				val >>= (uart_tx_state - UART_TX_BIT0);
				if(val & 1)
					UART_LOGIC_HIGH();
				else
					UART_LOGIC_LOW();
				break;
		}
	} else {
		TCCR0B = 0; // disable transmitter timer0
		uart_tx_state = UART_TX_IDLE;
	}
}

