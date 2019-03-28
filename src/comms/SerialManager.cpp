
#include "comms/SerialManager.h"

namespace SerialManager{

	SerialManager* serial_manager = nullptr;

	void SerialManager::push_to_vector(char* array, int length) {
		for(int i = 0; i < length; i++) {
			write_string.push_back(static_cast<uint8_t>(array[i]));
			if(small_pp) {
				write_string.push_back(static_cast<uint8_t>(array[i]));
				small_pp = false;
			}
			if(array[i] == 0x70) {
				small_pp = true;
			}		
		}
	}

	bool SerialManager::enqueue_write (uint8_t dest, uint8_t addr, short data) {
		if(!serial_open) return false;

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

		if( dest == V5_ID ? v5_queue.empty() : arduino_queue.empty() || !serial_open)
			return;

		uint8_t str = (dest == V5_ID ? 0 : 1);
		write_string.clear();

		Header packet_header ( dest, ( dest == V5_ID ? v5_queue.size() : arduino_queue.size() ) );
		push_to_vector(packet_header.header, HEADER_SIZE);

		if ( dest == V5_ID ) {
			while(!v5_queue.empty()) {
				push_to_vector(v5_queue.front().frame, FRAME_SIZE);
				v5_queue.pop();
			}
		}
		else {
			while(!arduino_queue.empty()) {
				push_to_vector(arduino_queue.front().frame, FRAME_SIZE);
				arduino_queue.pop();
			}
		}
		//Include a stream of zeros 
		for(int i = 0; i < 8; i++)
			write_string.push_back(0);

		write(fd[str], write_string.data(), write_string.size());
	}

	void SerialManager::receive_buffer (uint8_t src) {
		if(!serial_open) return;

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

		while(available(fd[src]) > 0)
		{
			read(fd[src], u,1);
			switch(*read_state)
			{
				case W :
					if(u[0] == 'W') {
						*read_state = H;
					}
					else {
						std::cout << u[0] << std::flush;
					}
			    	break;
				case H :
					if(u[0] == 'H')
						*read_state = O_1;
					else
						*read_state = W;
			    	break;
		    	case O_1 :
		    		if(u[0] == 'O')
						*read_state = O_2;
					else
						*read_state = W;
			    	break;
		      	case O_2 :
		      		if(u[0] == 'O')
						*read_state = P;
					else
						*read_state = W;
			    	break;
		    	case P :
			    	if(u[0] == 'P')
						*read_state = H_Dest;
					else
						*read_state = W;
			    	break;
		    	case H_Dest :
		    		if( src == 0 ) {
		    			if( u[0] == 2 || u[0] == 3 ){
							*dest = u[0];
							*read_state = H_Len;
			    		}
			    		else{
			    			*read_state = W;
			    		}
		    		}
		    		else {
		    			if( u[0] == 1 || u[0] == 2 ){
							*dest = u[0];
							*read_state = H_Len;
			    		}
			    		else
			    			*read_state = W;
		    		}
			    	break;
		    	case H_Len :
		    		if( u > 0 ){
						*rem = *u;
						*read_state = F_1;
						//std::cout << std::endl;
		    		}
					else{
						*read_state = W;
					}
					
			    	break;
		    	case F_1 :
			      	frame[0] = *u;
			      	*read_state = F_2;
			    	break;
		    	case F_2 :
			      	frame[1] = *u;
			      	*read_state = F_3;
			    	break;
		    	case F_3 :
			      	frame[2] = *u;
			      	*read_state = F_4;
			    	break;
		    	case F_4 : {
			      	frame[3] = *u;
			      	//std::cout << std::endl;

			      	Frame next(reinterpret_cast<char*>(frame));

			      	// std::cout << "****************" << std::endl;

			      	// std::cout << next.source << std::endl;
			      	// std::cout << next.var_address << std::endl;
			      	// std::cout << next.data << std::endl;

			      	// std::cout << "****************" << std::endl;

			      	if(next.is_valid()) {
			      		counter++;
			      		std::cout << "Frames recieved: " << counter << std::endl;
 				      	if( *dest == ODROID_ID )
				      		read_queue.push(next);
				      	else if( *dest == V5_ID )
				      		v5_queue.push(next);
				      	else if( *dest == ARDUINO_ID )
				      		arduino_queue.push(next);
				    }
				    else {
				    	std::cout << "CHECKSUM FAIL. Addr: " << (int)next.var_address << std::endl;
				    }

			      	(*rem)--;

			      	if(*rem > 0){
			      		*read_state = F_1;
			      	}
			      	else{
			      		*read_state = W;

			      	}

			    	break;
			    }
			   	default :
			   		break;
			}
		}
	}

	void SerialManager::sort_read_queue () {
		while(!read_queue.empty()) {
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
					read_queue.pop();
			}

		}
	}

	void SerialManager::handle_read() {
		if(!serial_open) return;

		receive_buffer(0);
		sort_read_queue();
		receive_buffer(1);
		sort_read_queue();
	}

	SerialManager::SerialManager () {

		fd[0] = open("/dev/ttyACM1", O_RDWR | O_NOCTTY );
		//fd[1] = open("/dev/ttyACM1", O_RDWR | O_NOCTTY );
		connected = true;
		if(fd[0] < 0) {
			connected = false;
		}
		if(set_interface_attribs(fd[0], B115200, 0) != 0) {
			connected = false;
		}
		if(connected) {
			stream_0.read_state = W;
			stream_1.read_state = W;
			small_pp = false;
			counter = 0;
		}
	}

	SerialManager::~SerialManager () {
		close(fd[0]);
		//close(fd[1]);
	}

	int SerialManager::set_interface_attribs (int fd, int speed, int parity)
	{
		serial_open = false;
	        struct termios tty;
	        memset (&tty, 0, sizeof tty);
	        if (tcgetattr (fd, &tty) != 0)
	        {
	                std::cerr << "error " << errno << " from tcgetattr" << std::endl;
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
	                std::cerr << "error " << errno << " from tcsetattr" << std::endl;
	                return -1;
	        }
		serial_open = true;
	        return 0;
	}

	int SerialManager::available(int fd) {
		int a;
		ioctl(fd, TIOCINQ, &a);
		return a;
	}

	bool SerialManager::is_connected() {
		return connected;
	}

	void init() {
		serial_manager = new SerialManager();
	}

	void destroy() {
		delete serial_manager;
	}

}
