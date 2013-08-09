/*
 * transmitter.c
 *
 *  Created on: 07.07.2013
 *      Author: Galiaf
 */


/*
 * main.c
 *
 *  Created on: 30.06.2013
 *      Author: Galiaf
 */
/*
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nRF24L01.h"

#define BIT(x) (1 << (x))
#define SETBITS(x, y) ((x) |= (y))
#define CLEARBITS(x, y) ((x) &= (~(y)))
#define SETBIT(x, y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x, y) CLEARBITS((x), (BIT((y))))

#define W 1
#define R 0

//#include <avr/iotn2313a.h>
#define XCK_DDR   DDRD
#define XCK_PORT  PORTD
#define XCK_PIN   PIND
#define XCK_BIT   2

#define BITRATE 9600
#define BAUD ((F_CPU / (2 * BITRATE)) - 1)
#define SET_REGISTER_DELAY 100

/*
 * USART - SPI

void initMSPI() {
	UBRRH = 0;
	UBRRL = 0;
	//Setting the XCK port pin as output, enables master mode.
	XCK_DDR |= (1<<XCK_BIT);
	//Set MSPI mode of operation and SPI data mode 0.
    UCSRC = (3<<UMSEL) | (0<<UCSZ1) | (0<<UCSZ0) | (0<<UCPOL); // M_SPI enabled by 3<<UMSEL
	//Enable receiver and transmitter.154
	UCSRB = (1<<RXEN)|(1<<TXEN);
	//Set baud rate.
	//IMPORTANT: The Baud Rate must be set after the transmitter is enabled
    UBRRH = (BAUD >> 8) & 0xff;
    UBRRL = (BAUD & 0xff);
}

uint8_t writeMSPI(uint8_t data) {
	// Wait for empty transmit buffer
	// Data Register Empty (UDRE) Flag
	//while ( !( UCSRA & (1<<UDRE)) );
	// Put data into buffer, sends the data
	UDR = data;
	// Wait for data to be received
	// Receive Complete (RXC) Flag
	while (!(UCSRA & (1<<RXC))) {}
	// Get and return received data from buffer
	return UDR;
}
/*
uint8_t getReg(uint8_t data) {
	_delay_us(10);
	CLEARBIT(PORTA, 1);//CSN low
	_delay_us(10);
	USART_Receive(R_REGISTER + data);
	_delay_us(10);
	data = USART_Receive(NOP);
	_delay_us(10);
	SETBIT(PORTA, 1);//CSN high
	return data;
}
*
void setNrfRegister(uint8_t reg, uint8_t value) {
	CLEARBIT(PORTA, 1);
	_delay_us(SET_REGISTER_DELAY);
	writeMSPI(W_REGISTER + reg);
	_delay_us(SET_REGISTER_DELAY);
	writeMSPI(value);
	_delay_us(SET_REGISTER_DELAY);
	SETBIT(PORTA, 1);
}

uint8_t swriteToNrf(uint8_t readWrite, uint8_t reg, uint8_t *val, uint8_t count) {
	if(readWrite == W) {
		reg = W_REGISTER + reg;
	}

	//static uint8_t result[32];

	_delay_us(100);
	CLEARBIT(PORTA, 1);
	_delay_us(100);
	writeMSPI(reg);
	_delay_us(100);

	int i;
	for(i = 0; i < count; i++) {
		if(readWrite == R && reg != W_TX_PAYLOAD) {
			val[i] = writeMSPI(NOP);
			_delay_us(100);
		} else {
			writeMSPI(val[i]);
			_delay_us(100);
		}
	}

	SETBIT(PORTA, 1);
	return *val;
}

void nrfInit() {
	//PA1 - CSN; PA0 - CE
	DDRA |= (1 << PA1) | (1 << PA0);
	SETBIT(PORTA, 1);//CSN to high to disable nrf
	CLEARBIT(PORTA, 0);//CE to low to nothing to send/receive

	_delay_ms(100);

	// auto-acknowledgements
	setNrfRegister(EN_AA, 0x01);

	// Number of retries and delay
	// "2": 750us delay; "F": 15 retries
	setNrfRegister(SETUP_RETR, 0x2F);

	// data pipe 0
	setNrfRegister(EN_RXADDR, 0x01);

	// 5 bytes RF_Address length
	setNrfRegister(SETUP_AW, 0x03);

	// 2.401GHz
	setNrfRegister(RF_CH, 0x01);

	// power and data speed
	//00000111 bit[3]=0 1Mbps - longer rage; bit[2-1] power mode (11=-0dB; 00=-8dB)
	setNrfRegister(RF_SETUP, 0x07);

	// Receiver address
	//uint8_t address[] = {0x12, 0x12, 0x12, 0x12, 0x12};
	//swriteToNrf(W, RX_ADDR_P0, address, 5);

	setNrfRegister(RX_ADDR_P0, 0x12);
	// Transmitter address
	//swriteToNrf(W, TX_ADDR, address, 5);
	setNrfRegister(TX_ADDR, 0x12);
	// Payload length setup
	// 5 bytes per package
	setNrfRegister(RX_PW_P0, 5);

	// Boot the nrf and set it as transmitter: 0x1E or receiver: 0x1F
	//0b0001 1110 - bit[0]=0: transmitter; bit[0]=1: receiver; bit[1]=1: power up; bit[4]=1: mask_Max_RT (disable IRQ)
	setNrfRegister(CONFIG, 0x1F);

	_delay_ms(100);
}

void transmit(uint8_t *data) {
	swriteToNrf(R, FLUSH_TX, data, 0);// sends 0xE1 to flush the registry from old data
	swriteToNrf(R, W_TX_PAYLOAD, data, 5);// send the data

	sei();

	_delay_ms(10);
	SETBIT(PORTA, 0);
	_delay_us(20);
	CLEARBIT(PORTA, 0);
	_delay_ms(10);
}

void receive() {
	sei();

	SETBIT(PORTA, 0);
	_delay_ms(100);
	CLEARBIT(PORTA, 0);

	cli();
}

void resetIRQ() {
	_delay_us(10);
	CLEARBIT(PORTA, 1);//SCN low
	_delay_us(10);
	writeMSPI(W_REGISTER + STATUS);
	_delay_us(10);
	writeMSPI(0x70); //Reset all IRQ in STATUS registry
	_delay_us(10);
	SETBIT(PORTA, 1);//SCN high
}

void initInterrupts() {
	DDRD &= ~(1 << DDD3);
	GIMSK = (1 << INT1);
	MCUCR = (1 << ISC00) | (1 << ISC01);
}

int main(void) {
	initMSPI();
	initInterrupts();
	nrfInit();
	DDRD |= (1 << PD5);
	DDRB = 0xFF;

/*
	if(getReg(STATUS) == 0x0E) {
		SETBIT(PORTD, 5);
		_delay_ms(100);
		CLEARBIT(PORTD, 5);
		_delay_ms(10);
	}
*
/*
	uint8_t val[5];
	val[0] = 0x02;*
	//_delay_ms(100);
	//writeToNrf(W, EN_RXADDR, val, 1);
	//_delay_ms(100);
    while (1) {
		//uint8_t data[5];// = {0x93,0x93,0x93,0x93,0x93};
		/*for(int i = 0; i < 5; i++) {
			data[i] = 0x93;
		}*
		//transmit(data);
		receive();
		//_delay_ms(100);
		/*
		if((getReg(STATUS) & (1 << 6)) != 0) {
			uint8_t *dat = writeToNrf(R, R_RX_PAYLOAD, data, 5);
			data[0] = dat[4];
		}
		PORTB = data[0];
		//PORTB = getReg(STATUS);0147 10010011
		_delay_ms(100);
		PORTB = 0;
		_delay_ms(100);*/
		/*
		if((getReg(STATUS) & (1 << 5)) != 0) {
			CLEARBIT(PORTD, 5);
			_delay_ms(100);
		} else {
			SETBIT(PORTD, 5);
		}
		*/
		//resetIRQ();
/*
		SETBIT(PORTD, 5);
		_delay_ms(1000);
		CLEARBIT(PORTD, 5);
  * }

    return 1;
}
/*
ISR(INT1_vect) {
	cli();
	CLEARBIT(PORTA, 0);

	//TODO
	uint8_t data[5];
	swriteToNrf(R, R_RX_PAYLOAD, data, 5);
	SETBIT(PORTD, 5);
	PORTB = data[0];
	resetIRQ();
	sei();
	//_delay_ms(100);
	CLEARBIT(PORTD, 5);
}*/
