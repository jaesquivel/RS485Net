/*
 RS485 protocol library - non-blocking.
 
 Devised and written by Nick Gammon.
 Date: 4 December 2012
 Version: 1.0
 
 Licence: Released for public use.

 
 Can send from 1 to 255 bytes from one node to another with:
 
 * Packet start indicator (STX)
 * Each data byte is doubled and inverted to check validity
 * Packet end indicator (ETX)
 * Packet CRC (checksum)
 
 */

#include <RS485Net.h>

// allocate the requested buffer size
void RS485Net::begin() {
	data_ = (byte *) malloc(bufferSize_);
	reset();
	errorCount_ = 0;
	RS485_SERIAL.begin(RS485_BAUD);
	pinMode(RS485_TX_ENABLE_PIN, OUTPUT);
} // end of RS485Net::begin

// get rid of the buffer
void RS485Net::stop() {
	reset();
	free(data_);
	data_ = NULL;
} // end of RS485Net::stop


// called after an error to return to "not in a packet"
void RS485Net::reset() {
	haveSTX_ = false;
	available_ = false;
	inputPos_ = 0;
	startTime_ = 0;
} // end of RS485Net::reset

// calculate 8-bit CRC
byte RS485Net::crc8(const byte *addr, byte len) {
	byte crc = 0;
	while (len--) {
		byte inbyte = *addr++;
		for (byte i = 8; i; i--) {
			byte mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0x8C;
			inbyte >>= 1;
		}  // end of for
	}  // end of while
	return crc;
}  // end of RS485Net::crc8

// send a byte complemented, repeated
// only values sent would be (in hex): 
//   0F, 1E, 2D, 3C, 4B, 5A, 69, 78, 87, 96, A5, B4, C3, D2, E1, F0
void RS485Net::sendComplemented(const byte what) {
	byte c;

	// first nibble
	c = what >> 4;
	write((c << 4) | (c ^ 0x0F));

	// second nibble
	c = what & 0x0F;
	write((c << 4) | (c ^ 0x0F));

}  // end of RS485Net::sendComplemented

void RS485Net::sendFrame(const byte * data, const byte length) {
}  // end of RS485Net::sendMsg

// called periodically from main loop to process data and 
// assemble the finished packet in 'data_'

// returns true if packet received.

// You could implement a timeout by seeing if isPacketStarted() returns
// true, and if too much time has passed since getPacketStartTime() time.

bool RS485Net::update() {
	// no data? can't go ahead (eg. begin() not called)
	if (data_ == NULL)
		return false;

	while (available() > 0) {
		byte inByte = read();

		switch (inByte) {

		case STX:   // start of text
			haveSTX_ = true;
			haveETX_ = false;
			inputPos_ = 0;
			firstNibble_ = true;
			startTime_ = millis();
			break;

		case ETX:   // end of text (now expect the CRC check)
			haveETX_ = true;
			break;

		default:
			// wait until packet officially starts
			if (!haveSTX_)
				break;

			// check byte is in valid form (4 bits followed by 4 bits complemented)
			if ((inByte >> 4) != ((inByte & 0x0F) ^ 0x0F)) {
				reset();
				errorCount_++;
				break;  // bad character
			} // end if bad byte

			// convert back
			inByte >>= 4;

			// high-order nibble?
			if (firstNibble_) {
				currentByte_ = inByte;
				firstNibble_ = false;
				break;
			}  // end of first nibble

			// low-order nibble
			currentByte_ <<= 4;
			currentByte_ |= inByte;
			firstNibble_ = true;

			// if we have the ETX this must be the CRC
			if (haveETX_) {
				if (crc8(data_, inputPos_) != currentByte_) {
					reset();
					errorCount_++;
					break;  // bad crc
				} // end of bad CRC

				available_ = true;
				return true;  // show data ready
			}  // end if have ETX already

			// keep adding if not full
			if (inputPos_ < bufferSize_)
				data_[inputPos_++] = currentByte_;
			else {
				reset(); // overflow, start again
				errorCount_++;
			}

			break;

		}  // end of switch
	}  // end of while incoming data

	return false;  // not ready yet
} // end of RS485Net::update

int RS485Net::read() {
	return RS485_SERIAL.read();
}

int RS485Net::available() {
	return RS485_SERIAL.available();
}

size_t RS485Net::write(const byte what) {
	digitalWrite(RS485_TX_ENABLE_PIN, HIGH);
	return RS485_SERIAL.write(what);
	digitalWrite(RS485_TX_ENABLE_PIN, LOW);
}
