#include <iostream>
#include <string>

#include <assert.h>
#include <mutex>

#include <vector>
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

#include "motor.h"


Motor::Motor(int _id, Bus* _bus, char* _name)
{
	id = _id;
	bus = _bus;
	debug = false;
	initialized = false;
	name = strdup(_name);

	assert(_id >= 1 && _id <= 32);
	multi_loop_angle_control_command_2_attempt = 0;
    multi_loop_angle_control_command_2_success = 0;

}

Motor::~Motor()
{
	power_off();
	printf("Destroying %s\n", name);
	free(name);
}

void Motor::set_debug(bool dbg)
{
	debug = dbg;
	if(debug)
	{
		printf("motor(%d):debug on\n",id);
	}
}

void Motor::dbg_printf(const char* format, ...) 
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
uint8_t Motor::checksum(std::vector<uint8_t> bytes)
{
	uint8_t cs = 0;
	for(size_t i=0;i<bytes.size();i++)
	{
		cs += bytes[i];
	}
	return cs;
}

//Convert encoder to centidegrees
int64_t	Motor::raw_encoder2centidegrees(uint16_t encoder)
{
	int64_t encoder_centidegrees = floor((36000.0 * encoder)/(1<<16));
	//assert(encoder_centidegrees >= 0);
	//assert(encoder_centidegrees < 360000);
	return encoder_centidegrees;
}

//Write parameter to RAM or ROM
int Motor::write_parameter(motor_param_t paramID, uint16_t A, uint16_t B, uint16_t C, motor_memorytype_t mt)
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

	std::vector<uint8_t> r = bus->exchangebytes(cmd, 8, 1000*1000000);

	// Validate response
	if(r.size() != 8) 
	{
		return 1;
	}

	// Validate header checksum
	std::vector<uint8_t> header(r.begin(), r.begin()+4);
	if(r[4] != checksum(header)) 
	{
		return 1;
	}

	// Validate response format
	if(r[0] != 0x3E || r[1] != 0x42 || r[2] != id || r[3] != 0x02) 
	{
		return 1;
	}

	// Validate parameter ID and data checksum
	if(r[5] != paramID || r[7] != checksum(std::vector<uint8_t>{r[5], r[6]})) 
	{
		return 1;
	}

	return r[6]; // Success = 0, Failure = 1
}


int Motor::read_parameter(motor_param_t paramID, uint16_t* A, uint16_t* B, uint16_t* C)
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

int Motor::set_power(bool power_on)
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


int Motor::multi_loop_angle_control_command_2(int64_t target_centidegrees, uint32_t max_speed, uint8_t* temp, int16_t* power, int16_t* speed, int64_t* encoder_centidegrees)
{
	multi_loop_angle_control_command_2_attempt += 1;
	
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

	if(r[0] != 0x3e) return 3;
	if(r[1] != 0xA4) return 3;
	if(r[2] != id) return 3;
	if(r[3] != 0x07) return 3;

	std::vector<uint8_t> r1(r.begin()+5, r.begin()+5+7);

	//Quite often the checksum fails - we suspect a firmware problem 
	if(r[12] == checksum(r1))
	{
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
		multi_loop_angle_control_command_2_success += 1;	
	}
	else
	{
		//There seem to be an error in the firmware failing this part often. 
		//We  just discard the return values when it happen
		//return 4;
	}
	
	
	
    return 0;
}

//******************************
//*  Read drive and motor type
//******************************

int Motor::read_drive_and_motor_type(productinfo* ppi) 
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

void Motor::print_productinfo(productinfo pi)
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

int Motor::print_drive_and_motor_type()
{
	productinfo pi;
	int r = read_drive_and_motor_type(&pi);
	if(r == 0)
	{
		print_productinfo(pi);
	}
	return r;	
}


    
int Motor::power_on()
{
	return set_power(true);
}

int Motor::power_off()
{
	return set_power(false);
}
	
	
//**********************************
//*  Read motor state 2
//**********************************
int Motor::read_motor_state_2(uint8_t* motor_temperature_celcius, int16_t* output_power, uint16_t* motor_speed, uint16_t* encoder_position)
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
//*  Motor stop
//******************************
/*
int Motor::stop()
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
*/


	

//******************************
//*  Read encoder
//******************************

//This should be private
int Motor::read_encoder(uint16_t* encoder_data, uint16_t* encoder_raw, uint16_t* encoder_zero_offset) 
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

int Motor::set_angle_pid(uint16_t Kp, uint16_t Ki)
{
	return write_parameter(PARAM_ANGULAR_LOOP_PID, Kp, Ki, 0, MEMORY_TYPE_RAM);
}

int Motor::get_angle_pid(uint16_t* pKp, uint16_t* pKi)
{		
	return read_parameter(PARAM_ANGULAR_LOOP_PID, pKp, pKi, NULL);
}

//******************************
//*  SpeedPID
//******************************

int Motor::set_speed_pid(uint16_t Kp, uint16_t Ki)
{
	return write_parameter(PARAM_SPEED_LOOP_PID, Kp, Ki, 0, MEMORY_TYPE_RAM);
}

int Motor::get_speed_pid(uint16_t* pKp, uint16_t* pKi)
{		
	return read_parameter(PARAM_SPEED_LOOP_PID, pKp, pKi, NULL);
}
	
int Motor::reboot()
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



	
int Motor::init_motor(uint16_t speed_kP, uint16_t speed_kI, uint16_t angle_kP, uint16_t angle_kI)
{
	printf("********** Motor - ID: %d (on bus)**********\n", id );
		
	printf("print_drive_and_motor_type\n");
	if(print_drive_and_motor_type() != 0)	
	{
		printf("Unable to find 	motor %d\n", id);
		return 1;
	}
	
	
	printf("power_on\n");
	if(power_on() != 0)
	{
		printf("Unable to turn motor %d on\n", id);
		return 1;
	}
	
	/*
	uint16_t encoder, raw, offset;            
	printf("read_encoder\n");
	if(read_encoder(&encoder, &raw, &offset) != 0)
	{
		printf("Unable to read encoder\n");
		return 1;
	}
	printf("Encoder: %d Raw:%d offset:%d\n", encoder, raw, offset);
	*/

	
	//printf("write_rom_zero\n");
	//if(m->write_rom_zero(&offset) != 0)
	//{
	//	printf("Unable to write rom encoder\n");
	//	return 1;
	//}
	//printf("Offset:%d\n", encoder, raw, offset);
	//
	//
	//printf("read_encoder2\n");
	//if(m->read_encoder(&encoder, &raw, &offset) != 0)
	//{
	//	printf("Unable to read encoder\n");
	//	return 1;
	//}
	//printf("Encoder: %d Raw:%d offset:%d\n", encoder, raw, offset);    
	//running = false;
	
	
	printf("set_speed_pid\n");
	if(set_speed_pid(speed_kP, speed_kI) != 0)
	{
		printf("Unable to set speed PID parameters\n");
		//return 1;
	}
		
	printf("set_angle_pid\n");
	if(set_angle_pid(angle_kP, angle_kI) != 0)
	{
		printf("Unable to set angle PID parameters\n");
		//return 1;
	}
	
	
	return 0;
}



int Motor::update(int64_t target_centidegrees)
{
	if(!initialized)
	{
		if(init_motor(100, 20, 100, 20) == 0)
		{
			initialized = true;
		}
	}
	
	if(!initialized)
	{
		return 1;
	}
	

	
	uint32_t max_speed  = 0; 
	if(multi_loop_angle_control_command_2(target_centidegrees, max_speed, &temp, &power, &speed, &encoder_centidegrees) != 0)
	{
		initialized = false;
		return 1;
	}
	return 0;
}
