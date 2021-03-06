#ifndef PACKET_H
#define PACKET_H

#include <cstdint>
#include <sys/time.h>
#include <time.h>
#include <ctime>

constexpr static uint8_t FRAME_SIZE = 4;
constexpr static uint8_t HEADER_SIZE = 7;

//Contains a frame along with its information
struct Frame {

	uint8_t 	source;
	uint8_t 	var_address;
	short 		data;
	char 		checksum;
	char 		frame [FRAME_SIZE];

	//Constructor creates a frame with given information
	Frame (uint8_t src, uint8_t addr, short data);

	//Constructor copies a frame from a character buffer
	Frame (char* buf);

	//Calculates an 8-bit checksum of the frame
	static char CRC8 (const char* data, int length);
	
	//Checks if the checksum in the frame is equal
	//to a calculated checksum of the frame
	bool is_valid (void);

};

//The header of a packet
struct Header {

	uint8_t		 	num_frames;
	uint8_t 		destination;
	char 			header [HEADER_SIZE];

	//Constructor creates a packet header with given information
	Header (uint8_t dest, uint8_t length);
	Header (char* buf);

};

//returns realtime of the system in milliseconds
unsigned long millis();

#endif