/*

 */

#ifndef RS485NET_H_
#define RS485NET_H_

#include <Arduino.h>

#ifndef RS485_SERIAL
#define RS485_SERIAL Serial
#endif

#ifndef RS485_TX_ENABLE_PIN
#define RS485_TX_ENABLE_PIN 3
#endif

#ifndef RS485_BAUD
#define RS485_BAUD	115200
#endif

#define MAX_NODES				32

/* Frame
Preamble		FRAME_PREAMBLE_LEN x FRAME_PREAMBLE bytes
Dest. Address	1 byte 0xFF is broadcast, 0xA0-0xFE is multicast
Source Address	1 byte
Flags			1 byte
Payload size	1 byte
Payload			1-64 bytes
CRC				1 byte
Postamble		FRAME_POSTAMBLE_LEN x FRAME_POSTAMBLE bytes
*/

#define	FRAME_PREAMBLE			0xAA
#define	FRAME_PREAMBLE_LEN		3
#define	FRAME_POSTAMBLE			0x55
#define	FRAME_POSTAMBLE_LEN		3
#define	FRAME_MAXPAYLOAD		64

// Byte Stuffing
#define BS_ESC_CHAR				0x7D
#define BS_FIRST_ESC			0x40
#define BS_ESC_CHARS			{ 0x7D, 0xAA, 0x55 }

#define CRC_SIZE			2

#define TIMEOUT_BYTES		10


#define	TOKEN_SIZEGHT		3
#define BYTE_TIME			1					// Time in milliseconds required to transmit a byte at slower baud rate
#define TOKEN_TIME		(TOKEN_SIZEGHT*BYTE_TIME)
#define	MAX_TIME			(MAXPAYLOAD*BYTE_TIME)
#define	BUSBUSY			0x03
#define	BUSRECV			0x05
#define	BUSFREE			0x00
#define	STARTDELAY		300				// Delay in miliseconds before node startup





#define BUS_IDLE				0x00
#define BUS_PREAMBLE			0x01
#define BUS_IN_FRAME			0x02
#define BUS_POSTAMBLE			0x03


class RS485Net {

	// Our node address
	uint8_t nodeAddress_;

	// where we save incoming stuff
	byte * buffer_;

	// how much data is in the buffer
	const int bufferSize_;

	unsigned long startTime_;

	// helper private functions
	byte crc16(const byte *addr, byte len);

public:

	// constructor
	RS485Net(uint8_t nodeAddress, const byte bufferSize);

	// destructor - frees memory used
	~RS485Net();

	// handle incoming data, return true if packet ready
	bool update();

	// send frame, returns error code or 0
	int sendFrame(uint8_t dest, const byte * data, const byte length);

	// get received frame, returns error code or 0
	int getFrame(const byte * data, const byte length);

private:

	int read();

	int available();

	size_t write(const byte what);

};	// RS485Net

#endif // RS485NET_H_
