/* */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

static inline void init_timer0(void)
{
	// enable OCR0A interrupt
	TIMSK0 = (1 << OCIE0A);
}

static void start_timer0(uint16_t bit_time)
{
	OCR0A = bit_time;
	TCNT0 = 0;

	// set mode to Clear timer on compare (OCR0A)
	// set prescaler flags
	TCCR0B = (1 << WGM02) | 0b001;
}

static inline void stop_timer0(void)
{
	TCCR0B = 0;
}

ISR(TIM0_COMPA_vect)
{
	return;
}

#define PIN_UART_TX		PB0

#define UART_LOGIC_HIGH()	PORTB &= ~(1 << PIN_UART_TX)
#define UART_LOGIC_LOW()	PORTB |= (1 << PIN_UART_TX)

#define UART_BAUD		(1220ULL)
#define UART_BIT_TIME		((F_CPU / 1ULL) / (UART_BAUD))

static void uart_tx(char c)
{
	char tmp;

	// enable timer for specified baudrate
	start_timer0(UART_BIT_TIME);

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

#define I2C_SCL_LOW_W()		(DDRB |= (1 << PIN_I2C_SCL))
#define I2C_SCL_HIGH_R()	(DDRB &= ~(1 << PIN_I2C_SCL))
#define I2C_SCL_IS_HIGH()	(PINB & (1 << PIN_I2C_SCL))

#define I2C_SDA_LOW_W()		(DDRB |= (1 << PIN_I2C_SDA))
#define I2C_SDA_HIGH_R()	(DDRB &= ~(1 << PIN_I2C_SDA))
#define I2C_SDA_IS_HIGH()	(PINB & (1 << PIN_I2C_SDA))

#define I2C_BAUD		(2000ULL)
#define I2C_BIT_TIME		((F_CPU / 4ULL) / (I2C_BAUD))

// requirement: SCL and SDA are high.
// sends an I2C start condition.
// afterwards: SDA is low, SCL is low, bus is ours.
static void i2c_start(void)
{
	start_timer0(I2C_BIT_TIME);

	I2C_SDA_HIGH_R();
	sleep_mode();

	I2C_SDA_LOW_W();
	sleep_mode();

	I2C_SCL_LOW_W();
	sleep_mode();
}

uint8_t i2c_sda_last_bit = 0;

// requirement: SCL is low.
// will then do a full clk cycle.
// (release CLK, wait for clk-stretchers until clk is up, pull clk down again)
// afterwards: SCL is low.
static void i2c_clk_wait_high_low(void)
{
	I2C_SCL_HIGH_R();
	do {
		sleep_mode();
	} while( !I2C_SCL_IS_HIGH() );
	sleep_mode();
	i2c_sda_last_bit = I2C_SDA_IS_HIGH();
	I2C_SCL_LOW_W();
	sleep_mode();
}

static void i2c_write_byte(uint8_t byte)
{
	for(uint8_t tmp = 0x80; tmp != 0; tmp >>= 1) {
		if(byte & tmp)
			I2C_SDA_HIGH_R();
		else
			I2C_SDA_LOW_W();
		sleep_mode();
		i2c_clk_wait_high_low();
	}
	I2C_SDA_HIGH_R();
	sleep_mode();
	// sample acknowledge bit
	i2c_clk_wait_high_low();
}

static uint8_t i2c_read_byte(void)
{
	uint8_t ret = 0;
	for(uint8_t tmp = 0x80; tmp != 0; tmp >>= 1) {
		sleep_mode();
		i2c_clk_wait_high_low();
		ret <<= 1;
		if(i2c_sda_last_bit)
			ret |= 1;
	}
	sleep_mode();
	I2C_SDA_LOW_W();
	i2c_clk_wait_high_low();
	I2C_SDA_HIGH_R();
	return ret;
}

// requirement: SCL is low
// sends an I2C stop condition
// afterwards: SCL is high, SDA is high, bus is released.
static void i2c_stop(void)
{
	I2C_SDA_LOW_W();
	sleep_mode();

	I2C_SCL_HIGH_R();
	do {
		sleep_mode();
	} while( !I2C_SCL_IS_HIGH() );

	I2C_SDA_HIGH_R();
	sleep_mode();

	stop_timer0();
}

char hexdigit(uint8_t lower_nibble)
{
	if(lower_nibble <= 9)
		lower_nibble += '0';
	else
		lower_nibble = lower_nibble - 10 + 'A';
	return lower_nibble;
}

int main(void)
{
	char temp=0;
	char v;

	// setup UART and I2C pins
	DDRB = (1 << PIN_UART_TX);
	PUEB = (1 << PIN_I2C_SCL) | (1 << PIN_I2C_SDA);

	init_timer0();

	sei();

	uart_tx('P');

	while(1) {
		// sample value
		i2c_start();
		i2c_write_byte(0x55);
		if(!i2c_sda_last_bit)
			temp = i2c_read_byte();
		i2c_stop();

		// send out value via uart (or error message)
		if(!i2c_sda_last_bit) {
			uart_tx('T');
			v = hexdigit(temp >> 4);
			uart_tx(v);
			v = hexdigit(temp & 0x0f);
			uart_tx(v);
		} else {
			uart_tx('N');
		}
		uart_tx('\n');
	}
}

