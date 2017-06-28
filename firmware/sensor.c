
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/******************************************************************************/
/* the timer0 is used as bit-timer for both software UART and software i2c.
 * it is configured to count TCNT0 up from 0 to OCR0A, trigger a TIM0_COMPA
 * interrupt, and reset TCNT0 to 0.
 * the interrupt itself is ignored, but takes the AVR cpu out of a sleep mode
 * so code execution continues at the exact time when a signal on an output or
 * input pin needs to be changed or sampled.
 * that way the using subsystem (uart or i2c) can simply
 *   * set a bit
 *   * calculate the next bit
 *   * set sleep mode
 * in a simple, procedural way. this heavily reduced code complexity and size
 * by trading in performance. it is rather power efficient though, as the
 * cpu spends most time in sleep mode.
 */
static inline void init_timer0(void)
{
	// enable OCR0A interrupt
	TIMSK0 = (1 << OCIE0A);
}

/* when starting a timing-critical transfer like uart or i2c, the timer needs
 * to be configured for the correct baud rate. the parameter baud_time
 * indicates the time between two symbol changes, in CPU cycles.
 */
static void start_timer0(uint16_t baud_time)
{
	OCR0A = baud_time;
	TCNT0 = 0;

	// set mode to Clear timer on compare (OCR0A)
	// set prescaler flags
	TCCR0B = (1 << WGM02) | 0b001;
}

/* when a transfer is finished it is best to turn off the timer again. */
static inline void stop_timer0(void)
{
	TCCR0B = 0;
}

/* timer that only returns, as the actions after leaving the sleep mode
 * are relevant, not the timer ISR itself.
 */
ISR(TIM0_COMPA_vect)
{
	return;
}

/******************************************************************************/
/* tiny software implementation of UART that can transfer a single character
 * at a time. having a pull-up resistor on the UART line is a good idea.
 * or a pull down if you use inverted UART level.
 * the current implementation uses a configurable baud rate, even parity and
 * two (actually even more) stop bits.
 * NOTE this implementation requires the timer0 setup as above.
 */
#define PIN_UART_TX		PB0
// normal UART level
#define UART_LOGIC_HIGH()	PORTB |= (1 << PIN_UART_TX)
#define UART_LOGIC_LOW()	PORTB &= ~(1 << PIN_UART_TX)
// inverted UART level (for RF transmission)
//#define UART_LOGIC_HIGH()	PORTB &= ~(1 << PIN_UART_TX)
//#define UART_LOGIC_LOW()	PORTB |= (1 << PIN_UART_TX)
// 1200 Baud UART, with CLOCK SKEW calibrated out. clock skew is oscillator-
// and temperature dependend, so one better works with lower baudrates here
// for univeral results.
// (actually, up to 57600 Baud works fine for me, via 59700ULL for the same chip)
#define UART_BAUD		(1243ULL)
#define UART_BAUD_TIME		((F_CPU / 1ULL) / (UART_BAUD))
/* transfer single char via software UART */
static void uart_tx(char c)
{
	char tmp;

	// enable timer for specified baudrate
	start_timer0(UART_BAUD_TIME);

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


/******************************************************************************/
/* converts the lower 4 bit of the parameter into a hexadecimal digit
 * representing its value.
 */
static char hexdigit(uint8_t lower_nibble)
{
	if(lower_nibble <= 9)
		lower_nibble += '0';
	else
		lower_nibble = lower_nibble - 10 + 'A';
	return lower_nibble;
}

/******************************************************************************/
/* the following functions represent a software i2c implementation
 * on PIN_I2C_SCL and PIN_I2C_SDA. it fully supports clock stretching
 * and can be used with cpu-internal pull ups (via PUEB register) or external
 * pull up resistors. it does *NOT* support multi-master busses.
 *
 * NOTE this implementation requires the timer0 setup as above.
 *
 * it is split into four functions and one variable:
 *	i2c_start()		send i2c start symbol
 *	i2c_write_byte()	send 8 bit
 *	i2c_read_byte()		read 8 bit
 *	i2c_stop()		send i2c stop symbol
 *	i2c_sda_last_bit	value of last bit that was sent or read.
 *				when i2c_write_byte() or i2c_read_byte() return
 *				this contains the value of the ACK bit.
 */

#define PIN_I2C_SCL		PB1
#define PIN_I2C_SDA		PB2

#define I2C_SCL_LOW_W()		(DDRB |= (1 << PIN_I2C_SCL))
#define I2C_SCL_HIGH_R()	(DDRB &= ~(1 << PIN_I2C_SCL))
#define I2C_SCL_IS_HIGH()	(PINB & (1 << PIN_I2C_SCL))

#define I2C_SDA_LOW_W()		(DDRB |= (1 << PIN_I2C_SDA))
#define I2C_SDA_HIGH_R()	(DDRB &= ~(1 << PIN_I2C_SDA))
#define I2C_SDA_IS_HIGH()	(PINB & (1 << PIN_I2C_SDA))

/* 20k bitrate (i.e. 80k baudrate, see below...) works fine with this
 * implementation, but probably is the upper limit. anything much higher
 * will continue to work, but bit timings won't be accurate and may be
 * much slower than 20k.
 */
#define I2C_BAUD		(20000ULL)
/* i2c needs an SDA change and an SDA sample point and additionally two
 * SCL changes per bit, which means it needs 4 symbol changes
 * per transferred bit. strictly this could be reduced to 3, as SDA only
 * needs to be sampled OR changed, but then SCL would be used arythmically
 * which looks stupid on an oscilloscope. as I love watching my oscilloscope,
 * I prefer the accurate, frequency-correct clk.
 */
#define I2C_BAUD_TIME		((F_CPU / 4ULL) / (I2C_BAUD))

/* sends an I2C start condition
 * requirement: SCL and SDA are high.
 * afterwards: SDA is low, SCL is low, bus is reserved for us.
 */
static void i2c_start(void)
{
	start_timer0(I2C_BAUD_TIME);

	I2C_SDA_HIGH_R();
	sleep_mode();

	I2C_SDA_LOW_W();
	sleep_mode();

	I2C_SCL_LOW_W();
	sleep_mode();
}

static uint8_t i2c_sda_last_bit = 0;

/* will do a full i2c clk cycle.
 * (release CLK, wait for clk-stretchers until clk is up, pull clk down again)
 * requirement: SCL is low.
 * afterwards: SCL is low.
 */
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

/* transfer a single byte via i2c, once i2c_start was called
 * requirement: SCL is low.
 * afterwards: SCL is low.
 */
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

/* read a single byte via i2c, once i2c_start was called
 * requirement: SCL is low.
 * afterwards: SCL is low.
 */
static uint8_t i2c_read_byte(uint8_t send_ack)
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

	if(send_ack) // send acknowledge bit
		I2C_SDA_LOW_W();
	i2c_clk_wait_high_low();
	I2C_SDA_HIGH_R();
	return ret;
}

/* send an i2c stop condition
 * requirement: SCL is low
 * afterwards: SCL is high, SDA is high (via pull up), bus is released.
 */
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

/******************************************************************************/
/* this will create two i2c transfers which are:
 *   * send an 8 bit command
 *   * read a 16 bit response
 * then the response value is passed along via UART
 * in hex with the least significant nibble first (!).
 *
 * address is the 7 bit i2c address of the target device.
 * command is the i2c command to send, i.e. the first byte, usually
 * the register to read.
 */
static void i2c_command8_resp16(uint8_t address, uint8_t command)
{
	uint16_t ret = 0;

	address <<= 1;

	// select i2c command
	i2c_start();
	i2c_write_byte(address | 0);
	if(i2c_sda_last_bit)
		goto fail;
	i2c_write_byte(command);
	if(i2c_sda_last_bit)
		goto fail;
	i2c_stop();

	// read higher 8 bit of 16 bit response
	i2c_start();
	i2c_write_byte(address | 1);
	if(i2c_sda_last_bit)
		goto fail;
	ret = i2c_read_byte(1) << 8;

	// read the lower 8 bit as well
	ret |= i2c_read_byte(0);

	// last transfer was successful, even though ACK bit was not low.
	// so clear i2c_sda_last_bit to indicate acknowledgement,
	// i.e. non-failure. strictly this is not required here.
	i2c_sda_last_bit = 0;
	i2c_stop();

	// i2c transfer finished and successfull.
	// now send the value out via uart:
	for(uint8_t i = 0; i < 4; ++i) {
		uart_tx(hexdigit(ret & 0xf));
		ret >>= 4;
	}

newline:
	uart_tx('\n');
	uart_tx('\r');
	return;

fail:
	// in case the device did not respond properly (ACK flags failed),
	// send an error.
	i2c_stop();
	uart_tx('?');
	goto newline;
}

/******************************************************************************/
/* firmware will read out connected temperature and humidity sensors on i2c
 * bus and write out their values via UART.
 * the used sensors are an ADT7410 and an Si7021-A20.
 * the firmware will then suspend.
 * after 8 seconds the cpu is reset via watchdog.
 * that way, temperature and humidity are sampled and printed every 8 seconds.
 */

#define SI7_ADDRESS 0b1000000
#define ADT_ADDRESS 0b1001000
int main(void)
{
	// setup UART and I2C pins
	UART_LOGIC_HIGH();
	DDRB = (1 << PIN_UART_TX);
	PUEB = (1 << PIN_I2C_SCL) | (1 << PIN_I2C_SDA);

	// set up WDT to restart us every 8 seconds.
	wdt_enable(0b1001);
	init_timer0();

	// set system clock to 8MHz by disabling prescaler
	CCP = 0xD8;
	CLKPSR = 0;

	sei();

	// from an ADT7410, read out temperatur register
	// and pass it along via uart
	uart_tx('T');
	i2c_command8_resp16(ADT_ADDRESS, 0x00);

	// same for humidity from an Si7021-A20
	uart_tx('h');
	i2c_command8_resp16(SI7_ADDRESS, 0xe5);

	// and temperature from an Si7021-A20
	uart_tx('t');
	i2c_command8_resp16(SI7_ADDRESS, 0xe0);

	// power down until watchdog wakes us up again
	SMCR = 0b010;
	sleep_mode();

	while(1) {}; // this is never reached but saves ~20 bytes...
}

