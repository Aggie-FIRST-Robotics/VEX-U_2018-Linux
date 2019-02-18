
#include "comms/SerialManager.h"

SerialManager serial_manager;

bool SerialManager::enqueue_write (uint8_t dest, uint8_t addr, short data) {
	//cout << "Enqueuing frame" << endl;
	Frame curr( ODROID_ID, addr, data );

	if( dest == V5_ID )
		v5_queue.push(curr);
	else if( dest == ARDUINO_ID )
		arduino_queue.push(curr);
	else
		return false;

	return true;
}

void SerialManager::send_buffer (uint8_t dest) {
	//cout << "Sending queue" << endl;

	if( dest == V5_ID ? v5_queue.empty() : arduino_queue.empty() )
		return;

	uint8_t str = (dest == V5_ID ? 0 : 1);
	write_string.clear();
	Header packet_header ( dest, ( dest == V5_ID ? v5_queue.size() : arduino_queue.size() ) );
	//cout << reinterpret_cast<uint8_t*>(packet_header.header) << endl;

	for(int i = 0; i < HEADER_SIZE; i++) {
		write_string.push_back(static_cast<uint8_t>(packet_header.header[i]));
	}

	char frame [FRAME_SIZE];
	if(!v5_queue.empty())
		//cout << "Start write" << endl;
	if ( dest == V5_ID ) {
		while(!v5_queue.empty()) {
			for(int i = 0; i < FRAME_SIZE; i++) {
				write_string.push_back(static_cast<uint8_t>(v5_queue.front().frame[i]));
			}
			v5_queue.pop();
		}
	}
	else {
		while(!arduino_queue.empty()) {
			// frame[0] = arduino_queue.front().frame[0];
			// frame[1] = arduino_queue.front().frame[1];
			// frame[2] = arduino_queue.front().frame[2];
			// frame[3] = arduino_queue.front().frame[3];
			// stream[str].write(reinterpret_cast<uint8_t*>(arduino_queue.front().frame), FRAME_SIZE);
			arduino_queue.pop();
		}
	}
	for(int i = 0; i < 8; i++)
		write_string.push_back(0);
	write(fd[str], write_string.data(), write_string.size());
	// stream[str].flush();
}

void SerialManager::receive_buffer (uint8_t src) {

	uint8_t u[1];

	uint8_t* dest;
	uint8_t* rem;
	uint8_t *frame;
	Read_Stream_States* read_state;

	if(src == 0) {
		dest = &stream_0.dest;
		rem = &stream_0.rem;
		frame = stream_0.frame;
		read_state = &stream_0.read_state;
	}
	else {
		dest = &stream_1.dest;
		rem = &stream_1.rem;
		frame = stream_1.frame;
		read_state = &stream_1.read_state;
	}

	if(available(fd[src]) > 0) {
		cout << "Start read" << endl;
		cout << "Destination :" << (int)*dest << endl;
		cout << "Remaining: " << (int)*rem << endl;
		cout << "Frame: " << bitset<32>(frame[0]) << endl;
		cout << "State: " << *read_state << endl;
	}
	while(available(fd[src]) > 0)
	{
		read(fd[src], u,1);
		std::cout << bitset<8>(u[0]) << " ";
		switch(*read_state)
		{
			case W :
			//cout << "W" << endl;
				if(u[0] == 'W') {
					*read_state = H;
				}
		    	break;
			case H :
			//cout << "H" << endl;
				if(u[0] == 'H')
					*read_state = O_1;
				else
					*read_state = W;
		    	break;
	    	case O_1 :
	    	//cout << "O_1" << endl;
	    		if(u[0] == 'O')
					*read_state = O_2;
				else
					*read_state = W;
		    	break;
	      	case O_2 :
	      	//cout << "O_2" << endl;
	      		if(u[0] == 'O')
					*read_state = P;
				else
					*read_state = W;
		    	break;
	    	case P :
	    	//cout << "P" << endl;
		    	if(u[0] == 'P')
					*read_state = H_Dest;
				else
					*read_state = W;
		    	break;
	    	case H_Dest :
	    	//cout << "H_Dest" << endl;
	    		//cout << static_cast<int>(u[0]) << endl;
	    		// if( src == 0 ) {
	    		// 	if( u[0] == 2 || u[0] == 3 ){
						*dest = u[0];
						*read_state = H_Len;
		    	// 	}
		    	// 	else{
		    	// 		//cout << "Failed destination check" << endl;
		    	// 		*read_state = W;
		    	// 	}
	    		// }
	    	// 	else {
	    	// 		if( u[0] == 1 || u[0] == 2 ){
						// *dest = u[0];
						// *read_state = H_Len;
		    // 		}
		    // 		else
		    // 			//cout << "Failed destination check" << endl;
		    // 			*read_state = W;
	    	// 	}
		    	break;
	    	case H_Len :
	    	cout << " End header" << endl;
	    		if( u > 0 ){
					*rem = *u;
					*read_state = F_1;
					cout << "Header dest: " << (int)*dest << endl;
					cout << "Header rem: " << (int)*rem << endl;

	    		}
				else{
					*read_state = W;
				}
				
		    	break;
	    	case F_1 :
	    	//cout << "F_1" << endl;
		      	frame[0] = *u;
		      	*read_state = F_2;
		    	break;
	    	case F_2 :
	    	//cout << "F_2" << endl;
		      	frame[1] = *u;
		      	*read_state = F_3;
		    	break;
	    	case F_3 :
	    	//cout << "F_3" << endl;
		      	frame[2] = *u;
		      	*read_state = F_4;
		    	break;
	    	case F_4 : {
	    		cout << " End frame, rem: " << (int)*rem << endl;
		      	frame[3] = *u;

		      	Frame next(reinterpret_cast<char*>(frame));

		      	if(next.is_valid()) {
		      		//cout << "Checksum Passed for addr:\t" << bitset<8>(frame[0]) << "\t" << bitset<8>(frame[1]) << "\t" << bitset<8>(frame[2]) << "\t" << bitset<8>(frame[3]) << endl;
			      	if( *dest == ODROID_ID )
			      		read_queue.push(next);
			      	else if( *dest == V5_ID )
			      		v5_queue.push(next);
			      	else
			      		arduino_queue.push(next);
			    }
			    else {
			    	//cout << "Checksum Failed for addr:\t" << bitset<8>(frame[0]) << "\t" << bitset<8>(frame[1]) << "\t" << bitset<8>(frame[2]) << "\t" << bitset<8>(frame[3]) << endl;
			    	//cout << "Checksum Failed for addr:\t" << bitset<8>(next.var_address) << "\t data: \t" << bitset<16>(next.data) << "\t calc checksum:\t" << bitset<8>(next.checksum) << "\t actual:\t" << bitset<8>(next.frame[3]) << endl;
			    }

		      	(*rem)--;
		      	//cout << static_cast<int>(rem) << endl;

		      	if(*rem > 0){
		      		*read_state = F_1;
		      	}
		      	else{
		      		cout << "End Packet" << endl;
		      		*read_state = W;

		      	}

		    	break;
		    }
		   	default :
		   		break;
		}
	}
	//cout << endl;
}

void SerialManager::sort_read_queue () {
	while(!read_queue.empty()) {
		//cout << static_cast<int>(read_queue.front().source) << endl;
		switch (read_queue.front().source){
			case V5_ID:
				v5_table.update(read_queue.front());
				read_queue.pop();
				break;
			case ARDUINO_ID:
				arduino_table.update(read_queue.front());
				read_queue.pop();
				break;
			default:
				cout << "Bad source flag" << endl;
				read_queue.pop();
		}

	}
}

void SerialManager::handle_read() {
	// stream_0.read_state = W;
	// stream_1.read_state = W;

	receive_buffer(0);
	//cout << "Probe 1" << endl;
	//receive_buffer(1);
	sort_read_queue();
	//cout << "Probe 2" << endl;
	// uint8_t u[1];
	// while (available(fd[0]) > 3){
	// 	for(int i = 0; i < 3; i++) {
	// 		read(fd[0], u,1);
	// 		// if((char)u[0] == 'E')
	// 		// 	cout << endl;
	// 		//else
	// 			cout << hex << (int)u[0] << " ";
	// 	}
	// 	cout << endl;
	// }
	//unsigned int start = millis();

	// while((stream_0.read_state != Exit || stream_1.read_state != Exit) && start + TIMEOUT > millis()){
	// 	if(stream_0.read_state != Exit)
	// 		receive_buffer(0);
	// 	if(stream_1.read_state != Exit)
	// 		//receive_buffer(1);
	// 	sort_read_queue();
	// }
	//cout << "Probe 3" << endl;
}

SerialManager::SerialManager () {

	fd[0] = open("/dev/ttyS10", O_RDWR | O_NOCTTY );

	if(fd[0] < 0) {
		cerr << "we're fucked" << endl;
	}

	set_interface_attribs(fd[0], B9600, 0);

	stream_0.read_state = W;
	stream_1.read_state = W;
	//stream[1] = serial::Serial("----", 115200);
	// pthread_t serial_task;
	// pthread_create(serial_task, NULL, handle_serial, (void *)this);

	// v5_table 			= DataTable();
	// arduino_table		= DataTable();
}

SerialManager::~SerialManager () {
}



// bool SerialManager::is_header(uint8_t src) {


// 	if ( stream[src].read() != "W") return false;
// 	if ( stream[src].read() != "H") return false;
// 	if ( stream[src].read() != "O") return false;
// 	if ( stream[src].read() != "O") return false;
// 	if ( stream[src].read() != "P") return false;
// 	return true;
// }

int SerialManager::set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                cerr << "error " << errno << " from tcgetattr" << endl;
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                cerr << "error " << errno << " from tcsetattr" << endl;
                return -1;
        }
        return 0;
}

int SerialManager::available(int fd) {
	int a;
	ioctl(fd, TIOCINQ, &a);
	return a;
}

unsigned long millis() {
	struct timespec a;
    clock_gettime(CLOCK_REALTIME, &a);
    //cout << a.tv_sec << "\t" << a.tv_nsec << "\t" << ((a.tv_sec%1000000)*1000 + a.tv_nsec/1000000) << endl;
    return (a.tv_sec%1000000)*1000 + a.tv_nsec/1000000;
}
