#include "bus.h"
#include "utils.h"
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>

#define BUFFER_SIZE 512


int configure_serial_port(char* serial_port_name)
{

	// Open the serial port
	int serial_port = open(serial_port_name, O_RDWR | O_NOCTTY);
	if(serial_port < 0) {
		perror("Failed to open serial port");
		return -1;
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));

	if (tcgetattr(serial_port, &tty) != 0) {
		perror("Failed to get tty attributes");
		return -1;
	}

	uint64_t speed;
	//speed = B38400;
	//speed= B115200;
	//speed = B230400;
	//speed = B460800;
	speed = B1000000;
	//speed = B2000000;
	

	// Set Baud Rate
	cfsetospeed(&tty, speed); // Output baud rate
	cfsetispeed(&tty, speed); // Input baud rate

	// 8N1 Mode (8 data bits, no parity, 1 stop bit)
	tty.c_cflag &= ~PARENB;     // Clear parity bit, disabling parity (no parity)
	tty.c_cflag &= ~CSTOPB;     // Clear stop field, only one stop bit
	tty.c_cflag &= ~CSIZE;      // Clear all bits that set the data size
	tty.c_cflag |= CS8;         // 8 bits per byte

	tty.c_cflag &= ~CRTSCTS;    // Disable RTS/CTS hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	// Disable canonical mode, echo, and signal characters
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Disable software flow control
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

	// Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST;

	// Set timeout (optional)
	tty.c_cc[VMIN]  = 0;    // Minimum number of characters to read
	tty.c_cc[VTIME] = 0;   // Timeout in deciseconds for reading

	// Apply the settings to the serial port
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		perror("Failed to set tty attributes");
		return -1;
	}
	
	return serial_port;
}



Bus::Bus(char* _serial_port_name)
{
	serial_port_name = strdup(_serial_port_name);
	serial_port = configure_serial_port(serial_port_name);
	debug = false;
}

Bus::~Bus()
{
	//close serial port
	dbg_printf("~Bus\n");
	free(serial_port_name);

}



void Bus::dbg_printf(const char* format, ...) 
{
	if(debug)
	{
		// First print the instance name
		fprintf(stdout, "[bus:%s] ", serial_port_name);
		
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	}        
}

void Bus::ser_write(std::vector<uint8_t> msg)
{
	int write_result = write(serial_port, msg.data(), msg.size());
	if (write_result < 0) {
		perror("Failed to write data to the serial port");
		close(serial_port);
		exit(1);
	}
}


std::vector<uint8_t> Bus::ser_read()
{
	
	unsigned char read_buffer[BUFFER_SIZE];
	memset(read_buffer, 0, BUFFER_SIZE);

	std::vector<uint8_t> r;

	// Read data from the serial port
	int num_bytes = read(serial_port, read_buffer, BUFFER_SIZE);
	if (num_bytes < 0) {
		if (errno == EAGAIN) {
			dbg_printf("No data available to read at the moment.\n");
		} else {
			perror("Failed to read data from the serial port");
			close(serial_port);
			exit(1);
		}
	}
	else
	{
		if(num_bytes>0)
		{
			r.insert(r.end(), read_buffer, &read_buffer[num_bytes]);
		}
	}
	return r;
}





std::vector<uint8_t> Bus::exchangebytes(std::vector<uint8_t> msg, uint8_t chars, uint64_t patience_ns)
{
	dbg_printf("Sending : ");
	if(debug)
	{
		print_msg(msg);
	}
	std::vector<uint8_t> r;
	{
		std::lock_guard<std::mutex> lock(mutex);
		
		ser_write(msg);
		
		
		uint64_t ns_start = get_timestamp_ns();
		while(1)
		{
			std::vector<uint8_t> r2 = ser_read();
			for(size_t i=0;i<r2.size();i++)
			{
				r.push_back(r2[i]);
			}
			
			uint64_t ns_now = get_timestamp_ns();
			uint64_t dt = ns_now - ns_start;

			if(dt >= patience_ns || r.size() >= chars)
			{
				break;	
			}
			//sleep a bit
			usleep(1);
		}
	}
	
	//usleep(10000);
	dbg_printf("Recieved : ");
	if(debug)
	{
		print_msg(r);
	}
	return r;
}

std::vector<uint8_t> Bus::exchangebytes(std::vector<uint8_t> msg, uint8_t chars)
{
	return exchangebytes(msg, chars, 50 * 1000000); //Allow 3 milliseconds for answer
}
    


void Bus::set_debug(bool dbg)
{
	debug = dbg;
	if(debug)
	{
		dbg_printf("bus:debug on\n");
	}
}
