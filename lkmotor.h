#ifndef LKMOTOR_H
#define LKMOTOR_H


#include <iostream>
#include <string>
#include <assert.h>
#include <mutex>

#include <vector>
#include <deque>
#include <fcntl.h>
#include <stdint.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <sys/time.h>

#include <pthread.h>

// *********************************************
// *
// *   Access to time
// *
// *********************************************


uint64_t get_timestamp_ns() {
	struct timespec ts;
	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		printf("time does not work\n");
		assert(0);
	}
	return (uint64_t)(ts.tv_sec) * 1000000000ULL + (uint64_t)(ts.tv_nsec);
}

uint64_t get_timestamp_ms() 
{
	return get_timestamp_ns()/1000000;
}


// *********************************************
// *
// *   RS485 Bus
// *
// *********************************************

void print_msg(std::vector<uint8_t> msg)
{
	for(size_t i=0;i<msg.size();i++)
	{
		printf(" %02x", msg[i]);
	}
	printf("\n");
}


static int bus_instance_counter=0;
class Bus
{
private:
    int serial_port;
    //pthread_mutex_t mutex;
	mutable std::mutex mutex;
	bool debug;
	
	int id;

	void dbg_printf(const char* format, ...) 
	{
		if(debug)
		{
			// First print the instance name
			fprintf(stdout, "[bus:%d] ", id);
			
			va_list args;
			va_start(args, format);
			vprintf(format, args);
			va_end(args);
		}        
	}

    void ser_write(std::vector<uint8_t> msg)
	{
		int write_result = write(serial_port, msg.data(), msg.size());
		if (write_result < 0) {
			perror("Failed to write data to the serial port");
			close(serial_port);
			exit(1);
		}
	}

    std::vector<uint8_t> ser_read()
	{
		unsigned char read_buffer[256];
		memset(read_buffer, 0, sizeof(read_buffer));

		std::vector<uint8_t> r;

		// Read data from the serial port
		int num_bytes = read(serial_port, read_buffer, sizeof(read_buffer));
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

		//uint64_t speed = B38400;
		//uint64_t speed = B115200;
		//uint64_t speed = B230400;
		//uint64_t speed = B460800;
		uint64_t speed = B1000000;

		//uint64_t speed = B2000000;
		

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

public:
    Bus(char* serial_port_name)
    {
        //mutex = PTHREAD_MUTEX_INITIALIZER;   
		id = bus_instance_counter++; 
        serial_port = configure_serial_port(serial_port_name);
		debug = false;
    }

    ~Bus()
    {
		dbg_printf("~Bus\n");
    }


	std::vector<uint8_t> exchangebytes(std::vector<uint8_t> msg, uint8_t chars, uint64_t patience_ns)
    {
		
		dbg_printf("Sending : ");
		if(debug)
		{
			print_msg(msg);
		}
		std::vector<uint8_t> r;
		{
			std::lock_guard<std::mutex> lock(mutex);
			//pthread_mutex_lock(&mutex);

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
		
        //pthread_mutex_unlock(&mutex);
		//usleep(10000);
		dbg_printf("Recieved : ");
		if(debug)
		{
			print_msg(r);
		}
		return r;
    }

	std::vector<uint8_t> exchangebytes(std::vector<uint8_t> msg, uint8_t chars)
	{
		return exchangebytes(msg, chars, 30 * 1000000); //Allow 3 milliseconds for answer
	}
    


	void set_debug(bool dbg)
	{
		debug = dbg;
		if(debug)
		{
			dbg_printf("bus:debug on\n");
		}
	}

};



// *********************************************
// *
// *   Motor
// *
// *********************************************

class Motor
{
public:
    
	typedef enum {
		PARAM_ANGULAR_LOOP_PID=0x96,
		PARAM_SPEED_LOOP_PID=0x97,
		
		PARAM_CURRENT_LOOP_PID=0x98,
		PARAM_MAX_TORQUE_CURRENT=0x99,
		PARAM_MAX_SPEED=0x9A,
		PARAM_MAX_ANGLE_LOW_4_BYTES=0x9B,
		PARAM_MAX_ANGLE_HIGH_4_BYTES=0x9C,
		PARAM_CURRENT_RAMP=0x9D,
		PARAM_SPEED_RAMP=0x9E,
		PARAM_MAX_POWER=0x99
	} motor_param_t;
	
	struct productinfo
	{
		uint8_t drivername[20];
		uint8_t motorname[20];
		uint8_t motorid[12];
		uint16_t hardware_version;
		uint16_t motor_version;
		uint16_t firmware_version;
	};
	
private:
    Bus* bus;
	bool debug;

	typedef enum {
		MEMORY_TYPE_RAM=0x42,
		MEMORY_TYPE_ROM=0x44,		
	} motor_memorytype_t;


	void dbg_printf(const char* format, ...) 
	{
		if(debug)
		{
			// First print the instance name
			fprintf(stdout, "[bus:%d] ", id);
			
			va_list args;
			va_start(args, format);
			vprintf(format, args);
			va_end(args);
		}        
	}

	//Checksum of bytearray
    uint8_t checksum(std::vector<uint8_t> bytes)
    {
        uint8_t cs = 0;
        for(size_t i=0;i<bytes.size();i++)
        {
            cs += bytes[i];
        }
        return cs;
    }

	//Convert encoder to centidegrees
	int64_t	raw_encoder2centidegrees(uint16_t encoder)
	{
		assert(encoder < (1<<bit_depth));
		int64_t encoder_centidegrees = floor((36000.0 * encoder)/((1<<bit_depth)));
		assert(encoder_centidegrees >= 0);
		assert(encoder_centidegrees < 360000);
		return encoder_centidegrees;
	}

	//Write parameter to RAM or ROM
	int write_parameter(motor_param_t paramID, uint16_t A, uint16_t B, uint16_t C, motor_memorytype_t mt)
	{
		std::vector<uint8_t> cmd;
    
		// Command header
		cmd.push_back(0x3E);  // Header
		cmd.push_back(mt);
		cmd.push_back(id);    // Motor ID
		cmd.push_back(0x07);  // Data length
		cmd.push_back(checksum(std::vector<uint8_t>(cmd.begin(), cmd.end())));

		// Parameter data
		cmd.push_back(paramID);
		cmd.push_back(A & 0xFF);
		cmd.push_back((A >> 8) & 0xFF);
		cmd.push_back(B & 0xFF); 
		cmd.push_back((B >> 8) & 0xFF);
		cmd.push_back(C & 0xFF);
		cmd.push_back((C >> 8) & 0xFF);
		cmd.push_back(checksum(std::vector<uint8_t>(cmd.begin()+5, cmd.end())));

		std::vector<uint8_t> r = bus->exchangebytes(cmd, 8, 100*1000000);

		// Validate response
		if(r.size() != 8) 
		{
			assert(0);
			return 1;
		}

		// Validate header checksum
		std::vector<uint8_t> header(r.begin(), r.begin()+4);
		if(r[4] != checksum(header)) 
		{
			assert(0);
			return 1;
		}

		// Validate response format
		if(r[0] != 0x3E || r[1] != 0x42 || r[2] != id || r[3] != 0x02) 
		{
			assert(0);
			return 1;
		}

		// Validate parameter ID and data checksum
		if(r[5] != paramID || r[7] != checksum(std::vector<uint8_t>{r[5], r[6]})) 
		{
			assert(0);
			return 1;
		}

		return r[6]; // Success = 0, Failure = 1
	}


	int read_parameter(motor_param_t paramID, uint16_t* A, uint16_t* B, uint16_t* C)
	{
		std::vector<uint8_t> cmd;
		
		// Command header
		cmd.push_back(0x3E);  // Header
		cmd.push_back(0x40);    // Read command from RAM
		cmd.push_back(id);    // Motor ID
		cmd.push_back(0x02);  // Data length
		cmd.push_back(checksum(std::vector<uint8_t>(cmd.begin(), cmd.end())));

		// Parameter data
		cmd.push_back(0x00);  // Reserved byte
		cmd.push_back(paramID);
		cmd.push_back(checksum(std::vector<uint8_t>{0x00, (uint8_t)paramID}));

		std::vector<uint8_t> r = bus->exchangebytes(cmd, 13);

		// Validate response length
		if(r.size() != 13) {
			printf("motor(%d):fail response length\n",id);
			return 1;
		}

		// Validate header checksum
		std::vector<uint8_t> header(r.begin(), r.begin()+4);
		if(r[4] != checksum(header)) {
			printf("motor(%d):fail header checksum\n",id);
			return 1;
		}

		// Validate response format
		if(r[0] != 0x3E || r[1] != 0x40 || r[2] != id || r[3] != 0x07) {
			printf("motor(%d):fail format\n",id);
			return 1;
		}

		// Validate data checksum
		std::vector<uint8_t> data(r.begin()+5, r.begin()+12);
		if(r[12] != checksum(data)) {
			if(debug)
			{
				printf("motor(%d):fail payload checksum\n",id);
			}
			return 1;
		}

		// Extract parameter values
		if(A) *A = r[6]  | (r[7]  << 8);
		if(B) *B = r[8]  | (r[9]  << 8);
		if(C) *C = r[10] | (r[11] << 8);

		return 0;
	}

public:


	
	int id;
    uint8_t bit_depth;
	char* name;
	uint64_t cnt_angle2_error_streak = 0;
	uint64_t cnt_angle2_good = 0;
	uint64_t cnt_angle2_bad = 0;
    
	//******************************
	//*  Constructor / Destructor
	//******************************
	
	Motor(int _id, uint8_t _bit_depth, Bus* _bus, const char* _name)
    {
        id = _id;
        bit_depth = _bit_depth;
        bus = _bus;
		debug = false;
		name = strdup(_name);
    }

    ~Motor()
    {
		printf("Destroying %s\n", name);
        free(name);
    }

	//******************************
	//*  Debug
	//******************************
	void set_debug(bool dbg)
	{
		debug = dbg;
		if(debug)
		{
			printf("motor(%d):debug on\n",id);
		}
	}
    

	//**********************************
	//*  Read motor state 2
	//**********************************
	int read_motor_state_2(uint8_t* motor_temperature_celcius, int16_t* output_power, uint16_t* motor_speed, uint16_t* encoder_position)
	{
		// Command header
		std::vector<uint8_t> cmd;
		cmd.push_back(0x3E);
		cmd.push_back(0x9C);
		cmd.push_back(id);
		cmd.push_back(0x00);
		cmd.push_back(checksum(std::vector<uint8_t>(cmd.begin(), cmd.end())));
		
		std::vector<uint8_t> r = bus->exchangebytes(cmd, 13, 1000);

		if(r.size() != 5+8) {
			printf("motor(%d): expected 13 bytes (5 cmd + 8 data), got %zu\n", id, r.size());
			return 1;
		}

		// Validate command header
		if(r[0] != 0x3E || r[1] != 0x9C || r[2] != id || r[3] != 0x07) {
			printf("motor(%d): invalid response header\n", id);
			return 1;
		}

		// Header checksum validation
		std::vector<uint8_t> header(r.begin(), r.begin()+4);
		if(r[4] != checksum(header)) {
			printf("motor(%d): header checksum mismatch\n", id);
			return 1;
		}

		// data checksum validation
		std::vector<uint8_t> data(r.begin()+5, r.begin()+5+7);
		//print_msg(data);
		if(r[12] != checksum(data)) {
			printf("motor(%d): data checksum mismatch\n", id);
			return 1;
		}

		if(motor_temperature_celcius)
		{
			*motor_temperature_celcius = data[0];
		}
		
		if(output_power)
		{
			*output_power = static_cast<int16_t>(data[1] + (data[2]<<8));			
		} 		
		if(motor_speed)
		{
			*motor_speed = data[3] + (data[4]<<8);
		} 
		if(encoder_position)
		{
			*encoder_position = data[5] + (data[6]<<8);
		} 
		return 0;
	} 
	
	//******************************
	//*  Turn on / off
	//******************************

private:
    int set_power(bool power_on)
    {
        std::vector<uint8_t> cmd;
		cmd.push_back(0x3e);
		if(power_on)
        {
            printf("motor_on\n");
			cmd.push_back(0x88);
        }
        else
        {
			printf("motor_off\n");
            cmd.push_back(0x80);
        }        
		cmd.push_back(id);
		cmd.push_back(0);
		cmd.push_back(checksum(cmd  ));
		std::vector<uint8_t> r = bus->exchangebytes(cmd, cmd.size(), 200*1000000);
        if(r.size() != cmd.size())
		{
			return 1;
		}
		for(size_t i=0;i<r.size(); i++)
		{
			if(r[i] != cmd[i])
			{
				return 1;
			}
		}
		return 0;
    }

public:

	int power_on()
	{
		return set_power(true);
	}

	int power_off()
	{
		return set_power(false);
	}


	//******************************
	//*  Motor stop
	//******************************
	int stop()
    {
        std::vector<uint8_t> cmd;
		cmd.push_back(0x3e);
		cmd.push_back(0x81);
        cmd.push_back(id);
		cmd.push_back(0);
		cmd.push_back(checksum(cmd  ));
		std::vector<uint8_t> r = bus->exchangebytes(cmd, cmd.size());
        assert(r.size() == cmd.size());
		for(size_t i=0;i<r.size(); i++)
		{
			if(r[i] != cmd[i])
			{
				return 1;
			}
		}
		return 0;
    }


	

	//******************************
	//*  Read encoder
	//******************************

	//This should be private
	int read_encoder(uint16_t* encoder_data, uint16_t* encoder_raw, uint16_t* encoder_zero_offset) 
	{
		std::vector<uint8_t> cmd;
		
		// Command header
		cmd.push_back(0x3E);
		cmd.push_back(0x90);
		cmd.push_back(id);
		cmd.push_back(0x00);
		cmd.push_back(checksum(std::vector<uint8_t>(cmd.begin(), cmd.end())));
		
		std::vector<uint8_t> r = bus->exchangebytes(cmd, 12, 100*1000000);

		if(r.size() != 5+7) {
			printf("motor(%d): expected 12 bytes (5 cmd + 7 data), got %zu\n", id, r.size());
			return 1;
		}

		// Validate command header
		if(r[0] != 0x3E || r[1] != 0x90 || r[2] != id || r[3] != 0x06) {
			printf("motor(%d): invalid response header\n", id);
			return 1;
		}

		// Header checksum validation
		std::vector<uint8_t> header(r.begin(), r.begin()+4);
		if(r[4] != checksum(header)) {
			printf("motor(%d): header checksum mismatch\n", id);
			return 1;
		}

		std::vector<uint8_t> data(r.begin()+5, r.begin()+5+6);
		/*printf("We look at %x\n", *(r.begin()+11) );
		print_msg(data);
		printf("Checksum of data is %x\n", checksum(data));*/
		if(*(r.begin()+11) != checksum(data)) {
			printf("motor(%d): data checksum mismatch\n", id);
			return 1;
		}

		//Parse encoder values
		if(encoder_data)
		{
			//This is raw encoder minus encoder_zero_offset
			*encoder_data = data[0] + (data[1] << 8);
		}

		if(encoder_raw)
		{
			//This is raw encoder
			*encoder_raw = data[2] + (data[3] << 8);
		}

		if(encoder_zero_offset)
		{
			//This is the zero offset
			*encoder_zero_offset = data[4] + (data[5] << 8);
		}
		return 0;
	}

	
	


	//******************************
	//*  AnglePID
	//******************************

	int set_angle_pid(uint16_t Kp, uint16_t Ki)
	{
		return write_parameter(PARAM_ANGULAR_LOOP_PID, Kp, Ki, 0, MEMORY_TYPE_RAM);
	}

	int get_angle_pid(uint16_t* pKp, uint16_t* pKi)
	{		
		return read_parameter(PARAM_ANGULAR_LOOP_PID, pKp, pKi, NULL);
	}
	
	//******************************
	//*  SpeedPID
	//******************************

	int set_speed_pid(uint16_t Kp, uint16_t Ki)
	{
		return write_parameter(PARAM_SPEED_LOOP_PID, Kp, Ki, 0, MEMORY_TYPE_RAM);
	}

	int get_speed_pid(uint16_t* pKp, uint16_t* pKi)
	{		
		return read_parameter(PARAM_SPEED_LOOP_PID, pKp, pKi, NULL);
	}
	
	
	int get_speed_ramp(int32_t* speed_ramp)
	{		
		uint16_t low;
		uint16_t high;
		int r = read_parameter(PARAM_SPEED_RAMP, &low, &high, NULL);
		if(r == 0)
		{
			*speed_ramp = (high << 16) + low;
		}

		return r;
	}
	
	int reboot()
	{
		std::vector<uint8_t> cmd;
		cmd.push_back(0x3E);
		cmd.push_back(0x07);
		cmd.push_back(id);
		cmd.push_back(0x00);
		uint8_t chk = checksum(cmd);
		cmd.push_back(chk);
		std::vector<uint8_t> r = bus->exchangebytes(cmd, 0);
		//reboot has no answer
		return 0;
	}

	

	int multi_loop_angle_control_command_2(int64_t target_centidegrees, uint32_t max_speed, uint8_t* temp, int16_t* power, int16_t* speed, int64_t* encoder_centidegrees)
    {
        
		
		//printf("multi_loop_angle_control_command_2, %ld, %u\n", target_centidegrees, max_speed);
		std::vector<uint8_t> cmd;
		cmd.push_back(0x3E);
		cmd.push_back(0xA4);
		cmd.push_back(id);
		cmd.push_back(0x0c);
		cmd.push_back(checksum(cmd));

		//printf("target_centidegrees_64: %ld\n", target_centidegrees_64);
		for(int i=0;i<8;i++)
		{
			cmd.push_back(((uint8_t*)&target_centidegrees)[i]);
		}
		for(int i=0;i<4;i++)
		{
			cmd.push_back(((uint8_t*)&max_speed)[i]);
		}
		
		cmd.push_back(checksum(std::vector<uint8_t>(cmd.begin()+5, cmd.begin()+5+8+4)));

		std::vector<uint8_t> r = bus->exchangebytes(cmd, 13);

		if(r.size() != 13)
		{
			//Motor does not send reply
			//Communication must have failed
			return 1;
		}
		

		std::vector<uint8_t> r0(r.begin(), r.begin()+4);
		if(r[4] != checksum(r0))
		{
			//printf("Checksum error in multi_loop_angle_control_command_2 !!!\n");
			//As motor is sending reply but the reply fails
			//Forward message must be good..
			return 2;
		}

		assert(r[0] == 0x3e);
		assert(r[1] == 0xA4);
		assert(r[2] == id);
		assert(r[3] == 0x07);

		std::vector<uint8_t> r1(r.begin()+5, r.begin()+5+7);

		if(temp)
		{
			*temp = *((int8_t*)&r1[0]);
		}
		if(power)
		{
			*power = *((int16_t*)&r1[1]);
		}
		if(speed)
		{
			*speed = *((int16_t*)&r1[3]);
		}
		
		uint16_t encoder_raw = *((uint16_t*)&r1[5]);            
		
		if(encoder_centidegrees)
		{
			*encoder_centidegrees = raw_encoder2centidegrees(encoder_raw);
		}
		
		//printf("multi_loop_angle_control_command_2 %lu\n", target_centidegrees);
		//print_msg(cmd);
		//print_msg(r);

		uint8_t cs = checksum(r1);
        if(r[12] == cs)
		{
            cnt_angle2_good ++;
			cnt_angle2_error_streak = 0;
		}
		else
		{
			cnt_angle2_bad ++;
			cnt_angle2_error_streak++;
			//printf("%02x should be %02x\n", r[12],cs);
			//float pct = 0;
			//if(cnt_angle2_good > 0)
			//{
			//	pct = cnt_angle2_bad * 1.0 / cnt_angle2_good;
			//}
			//printf("Stats %lu %lu %f\n", cnt_angle2_bad,cnt_angle2_good, pct);
			//printf("\n\nError %s\n", name);
			//printf("Checksum error in multi_loop_angle_control_command_2 %lu\n", target_centidegrees);
			//print_msg(cmd);
			//print_msg(r);

			return 3;
		}

		return 0;
    }

	



	
	//******************************
	//*  Read drive and motor type
	//******************************

	int read_drive_and_motor_type(productinfo* ppi) 
	{
		std::vector<uint8_t> cmd;
		
		// Command header
		cmd.push_back(0x3E);
		cmd.push_back(0x12);
		cmd.push_back(id);
		cmd.push_back(0x00);
		cmd.push_back(checksum(std::vector<uint8_t>(cmd.begin(), cmd.end())));
		

		std::vector<uint8_t> r = bus->exchangebytes(cmd, 64, 100*1000000);  //allow 100ms
		
		if(r.size() != 64) {
			dbg_printf("read_drive_and_motor_type: expected 64 bytes (5 cmd + 59 data), got %zu\n", r.size());
			return 1;
		}

		// Validate command header
		if(r[0] != 0x3E || r[1] != 0x12 || r[2] != id || r[3] != 0x3A) {
			dbg_printf("read_drive_and_motor_type: invalid response header\n");
			return 1;
		}

		// Header checksum validation
		std::vector<uint8_t> header(r.begin(), r.begin()+4);
		if(r[4] != checksum(header)) {
			dbg_printf("read_drive_and_motor_type: header checksum mismatch\n");
			return 1;
		}
		
		//Todo data checksum validation!!!

		if(ppi)
		{
			memcpy(ppi, &r[5], sizeof(productinfo));
		}
		

		return 0;
	}

	void print_productinfo(productinfo pi)
	{
		
		printf("Motor:             %s\n", this->name);
		printf("Driver Name:       %s\n", pi.drivername);
		printf("Motor Name:        %s\n", pi.motorname);
		char buf[13];
		buf[12] = 0;
		memcpy(buf, pi.motorid, 12);
		printf("Motor ID:          %s\n", buf);
		printf("Hardware Version:  %0.2f\n", pi.hardware_version/100.0f);
		printf("Motor Version:     %0.2f\n", pi.motor_version/100.0f);
		printf("Firmware Version:  %0.2f\n", pi.firmware_version/100.0f);
	}

	int print_drive_and_motor_type()
	{
		productinfo pi;
		int r = read_drive_and_motor_type(&pi);
		if(r == 0)
		{
			print_productinfo(pi);
		}
		return r;	
	}

};

#endif //LK_MOTOR