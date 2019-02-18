#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include <queue>
#include <time.h>
#include <ctime>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <bitset>
//#include <pthread.h>
//#include <thread>



#include "serial/serial.h"
#include "comms/Packet.h"
#include "comms/DataTable.h"

using namespace std;

constexpr static uint8_t V5_ID = 1;
constexpr static uint8_t ODROID_ID = 2;
constexpr static uint8_t ARDUINO_ID = 3;
constexpr static uint8_t UPDATE_PERIOD = 100;
constexpr static uint8_t TIMEOUT = 50;

enum Read_Stream_States { W, H, O_1, O_2, P, H_Dest, H_Len, F_1, F_2, F_3, F_4 };

struct StreamInfo {
	uint8_t rem = 0;
	uint8_t dest = 0;
	uint8_t frame [FRAME_SIZE];
	Read_Stream_States read_state;
};

//Manages buffers for serial communication
class SerialManager {
	private:

		queue <Frame> v5_queue;
		queue <Frame> arduino_queue;
		queue <Frame> read_queue;

		int fd[2];
		struct termios tty[2];

		StreamInfo stream_0;
		StreamInfo stream_1;

		std::vector<uint8_t> write_string;

		//Checks if the first 5 characters of the read buffer
		//are "WHOOP"
		bool is_header(uint8_t src);
		int set_interface_attribs(int fd, int speed, int parity);
		int available(int fd);
	public:

		DataTable 		v5_table;
		DataTable		arduino_table;

		//Returns a pointer to the single instance of Serial Manager
		SerialManager ();

		~SerialManager ();

		//Puts variable data into a write queue
		// '1' -> v5
		// '2' -> odroid
		// '3' -> arduino
		// addr [0-63]
		// returns false if destination is not 1 or 2
		bool enqueue_write (uint8_t dest, uint8_t addr, short data);

		//Sends the write queue into the write
		//buffer in the form of a packet
		void send_buffer (uint8_t destination);

		//Reads a packet in the read buffer and updates data tables
		//returns false if packet is corrupted
		void receive_buffer (uint8_t);
		void sort_read_queue ();
		void handle_read();
};

//Task function that sends both buffers at a rate of 100 Hz


unsigned long millis();

extern SerialManager serial_manager;

#endif
