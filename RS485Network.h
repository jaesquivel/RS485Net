/*
 RS485 protocol library - non-blocking.

 Devised and written by Nick Gammon.
 Date: 4 December 2012
 Version: 1.0

 Licence: Released for public use.

 */

#include <Arduino.h>

#ifndef RS485_SERIAL
#define RS485_SERIAL Serial;
#endif

#ifndef RS485_TX_ENABLE_PIN
#define RS485_TX_ENABLE_PIN 3;
#endif

#ifndef RS485_BAUD
#define RS485_BAUD	115200
#endif

class RS485Network {

	enum {
		STX = '\2',   // start of text
		ETX = '\3'    // end of text
	};  // end of enum

	AvailableCallback fAvailableCallback_;
	WriteCallback fWriteCallback_;

	// where we save incoming stuff
	byte * data_;

	// how much data is in the buffer
	const int bufferSize_;

	// this is true once we have valid data in buf
	bool available_;

	// an STX (start of text) signals a packet start
	bool haveSTX_;

	// count of errors
	unsigned long errorCount_;

	// variables below are set when we get an STX
	bool haveETX_;
	byte inputPos_;
	byte currentByte_;
	bool firstNibble_;
	unsigned long startTime_;

	// helper private functions
	byte crc8(const byte *addr, byte len);
	void sendComplemented(const byte what);

public:

	// constructor
	RS485Network(const byte bufferSize) :
			data_(NULL), bufferSize_(bufferSize) {
	}

	// destructor - frees memory used
	~RS485Network() {
		stop();
	}

	// allocate memory for buf_
	void begin();

	// free memory in buf_
	void stop();

	// handle incoming data, return true if packet ready
	bool update();

	// reset to no incoming data (eg. after a timeout)
	void reset();

	// send data
	void sendMsg(const byte * data, const byte length);

	// returns true if packet available
	bool available() const {
		return available_;
	}
	;

	// once available, returns the address of the current message
	byte * getData() const {
		return data_;
	}
	byte getLength() const {
		return inputPos_;
	}

	// return how many errors we have had
	unsigned long getErrorCount() const {
		return errorCount_;
	}

	// return when last packet started
	unsigned long getPacketStartTime() const {
		return startTime_;
	}

	// return true if a packet has started to be received
	bool isPacketStarted() const {
		return haveSTX_;
	}

};
// end of class RS485

