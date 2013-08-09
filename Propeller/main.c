/*
 * main.c
 *
 *  Created on: 30.06.2013
 *      Author: Galiaf
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nRF24L01.h"

//#include <avr/iotn2313a.h>
#define XCK_DDR   DDRD
#define XCK_PORT  PORTD
#define XCK_PIN   PIND
#define XCK_BIT   2
#define PCMSK2 _SFR_IO8(0x005)
#define PCINT15 4
#define PCIE2 4

#define CSN_PORT PORTA
#define CSN_PIN PORTA1
#define CE_PORT PORTA
#define CE_PIN PORTA0

#define BITRATE 9600
#define BAUD ((F_CPU / (2 * BITRATE)) - 1)
#define NRF_DATA_LENGTH 5
#define SET_REGISTER_DELAY 100
#define NRF_LISTEN_DELAY 100

//#define BIT(x) (1 << (x))
//#define SETBITS(x, y) ((x) |= (y))
//#define CLEARBITS(x, y) ((x) &= (~(y)))
//#define SETBIT(x, y) SETBITS((x), (BIT((y))))
//#define CLEARBIT(x, y) CLEARBITS((x), (BIT((y))))
#define SETBIT(x, y) x |= (1 << y)
#define CLEARBIT(x, y) x &= (~(1 << y))

#define CSN_LOW() CLEARBIT(CSN_PORT, CSN_PIN)
#define CSN_HIGH() SETBIT(CSN_PORT, CSN_PIN)
#define CE_LOW() CLEARBIT(CE_PORT, CE_PIN)
#define CE_HIGH() SETBIT(CE_PORT, CE_PIN)

#define charWidth 8
uint8_t messageLength = 6;
//7E-41-41-41-42 1-61-96-98-91-FF-41 41-FF-89-88-88-70 7E-41-41-41-41-7E 41-41-66-18-FF-18-66-41-41 1-3-3D-C8-C8-3D-3-1
/*
 *
const unsigned char message[47] = {
		0x7E, 0x41, 0x41, 0x41, 0x42, 0,
		0x1, 0x61, 0x96, 0x98, 0x91, 0xFF, 0x41, 0,
		0x41, 0xFF, 0x89, 0x88, 0x88, 0x70, 0,
		0x7E, 0x41, 0x41, 0x41, 0x41, 0x7E, 0,
		0x41, 0x41, 0x66, 0x18, 0xFf, 0x18, 0x66, 0x41, 0x41, 0,
		0x1, 0x3, 0x3D, 0xC8, 0xC8, 0x3D, 0x3, 0x1, 0
};
 * */
const unsigned char font[6][charWidth] = {
	//{0x00, 0x00, 0xFE, 0x81, 0x81, 0xFE, 0x00, 0x00},  // 0
	//{0x00, 0x00, 0x04, 0x02, 0xFF, 0x00, 0x00, 0x00},  // 1
	//{0x80, 0x40, 0xA2, 0x11, 0x89, 0x06, 0x80, 0x00},  // 2
	//{0xC0, 0x00, 0x82, 0x01, 0x89, 0x76, 0x00, 0x00},  // 3
	//{0x00, 0x10, 0x18, 0x14, 0x12, 0xFF, 0x10, 0x00},  // 4
	//{0x40, 0x8F, 0x09, 0x89, 0x09, 0x90, 0x60, 0x00},  // 5
	//{0x00, 0xF0, 0x0C, 0x8A, 0x09, 0x90, 0x60, 0x00},  // 6
	//{0x00, 0x00, 0x01, 0xF1, 0x09, 0x05, 0x03, 0x00},  // 7
	//{0x60, 0x96, 0x09, 0x89, 0x09, 0x96, 0x60, 0x00},  // 8
	//{0x00, 0x00, 0x0E, 0x91, 0x51, 0x3E, 0x00, 0x00},  // 9
	//{0x00, 0x00, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00},  // :
	{0x7E, 0x81, 0x81, 0x81, 0x42, 0x00, 0x00, 0x00},
	{0x01, 0x61, 0x96, 0x98, 0x91, 0xFF, 0x81, 0x00},
	{0x41, 0xFF, 0x89, 0x88, 0x88, 0x70, 0x00, 0x00},
	{0x7E, 0x81, 0x81, 0x81, 0x81, 0x7E, 0x00, 0x00},
	{0x81, 0x66, 0x18, 0xFf, 0x18, 0x66, 0x81, 0x00},
	{0x03, 0x3C, 0xC8, 0xC8, 0x3C, 0x03, 0x00, 0x00}
};

uint8_t message[] = {0, 1, 2, 3, 4, 5};

/*
 * USART - SPI
 */
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

void initNrfRegister(uint8_t reg) {
	CSN_LOW();
	_delay_us(SET_REGISTER_DELAY);
	writeMSPI(reg);
	_delay_us(SET_REGISTER_DELAY);
}

void getNrfReceivedData(uint8_t *data) {
	initNrfRegister(R_RX_PAYLOAD);

	for(int i = 0; i < NRF_DATA_LENGTH; i++) {
		data[i] = writeMSPI(NOP);
		_delay_us(SET_REGISTER_DELAY);
	}

	CSN_HIGH();
}

void setNrfRegister(uint8_t reg, uint8_t val, uint8_t count) {
	initNrfRegister(W_REGISTER + reg);

	for(int i = 0; i < count; i++) {
		writeMSPI(val);
		_delay_us(SET_REGISTER_DELAY);
	}

	CSN_HIGH();
}

void initNrf() {
	//PA1 - CSN; PA0 - CE
	DDRA |= (1 << PA1) | (1 << PA0);
	CSN_HIGH();//CSN to high to disable nrf
	CE_LOW();//CE to low to nothing to send/receive

	_delay_ms(100);

	// auto-acknowledgements
	setNrfRegister(EN_AA, 0x01, 1);

	// Number of retries and delay
	// "2": 750us delay; "F": 15 retries
	setNrfRegister(SETUP_RETR, 0x2F, 1);

	// data pipe 0
	setNrfRegister(EN_RXADDR, 0x01, 1);

	// 5 bytes RF_Address length
	setNrfRegister(SETUP_AW, 0x03, 1);

	// 2.401GHz
	setNrfRegister(RF_CH, 0x01, 1);

	// power and data speed
	// 00000111 bit[3]=0 1Mbps - longer rage; bit[2-1] power mode (11=-0dB; 00=-8dB)
	setNrfRegister(RF_SETUP, 0x07, 1);

	// Receiver address
	setNrfRegister(RX_ADDR_P0, 0x12, 5);
	// Transmitter address
	setNrfRegister(TX_ADDR, 0x12, 5);

	// Payload length setup
	// 5 bytes per package
	setNrfRegister(RX_PW_P0, NRF_DATA_LENGTH, 1);

	// Boot the nrf and set it as transmitter: 0x1E or receiver: 0x1F
	//0b0001 1110 - bit[0]=0: transmitter; bit[0]=1: receiver; bit[1]=1: power up; bit[4]=1: mask_Max_RT (disable IRQ)
	setNrfRegister(CONFIG, 0x1F, 1);

	_delay_ms(100);
}

void listenNrf() {
	CE_HIGH();
	_delay_ms(NRF_LISTEN_DELAY);
	CE_LOW();
}

void initInterrupts() {
	DDRD &= (~(1 << DDD3)) | (~(1 << DDD4));
	GIMSK = (1 << INT1) | (1 << PCIE2);
	PCMSK2 = (1 << PCINT15);
	//TODO:
	//MCUCR = (1 << ISC10) | (1 << ISC11);
}

int main(void) {
	cli();
	initMSPI();
	initInterrupts();
	initNrf();
	sei();
	//debug
	DDRD |= (1 << PD5);
	DDRB = 0xFF;

	while(1) {
		listenNrf();
	}

    return 1;
}

ISR(INT1_vect) {
	cli();
	CE_LOW();

	//TODO
	//uint8_t data[NRF_DATA_LENGTH];
	getNrfReceivedData(message);

	// reset IRQ bits
	setNrfRegister(STATUS, 0x70, 1);
	sei();
	//debug
	SETBIT(PORTD, 5);
	//PORTB = data[0];
	_delay_ms(100);
	CLEARBIT(PORTD, 5);
}

ISR(PCINT_D_vect) {
	for(int i = 0; i < messageLength; i++) {
		for(int j = 0; j < charWidth; j++) {
			PORTB = font[message[i]][j];
			_delay_us(500);
		}
	}
}
