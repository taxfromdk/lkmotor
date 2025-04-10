#pragma once

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
#include "bus.h"



// *********************************************
// *
// *   Motor
// *
// *********************************************

class Motor
{

private:    
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
    
	typedef enum {
		MEMORY_TYPE_RAM=0x42,
		MEMORY_TYPE_ROM=0x44,		
	} motor_memorytype_t;


	Bus* bus;				//Bus we are attached to
	int id; 				//Id of motor 1 to 32 (1 indexed)
    char* name;				//Human friendly name
	
	bool initialized;
	
	bool debug;
	
	
	
	

	uint32_t max_speed = 0;


	void dbg_printf(const char* format, ...);

	//Checksum of bytearray
    uint8_t checksum(std::vector<uint8_t> bytes);

	//Convert encoder to centidegrees
	int64_t	raw_encoder2centidegrees(uint16_t encoder);

	//Write parameter to RAM or ROM
	int write_parameter(motor_param_t paramID, uint16_t A, uint16_t B, uint16_t C, motor_memorytype_t mt);

	int read_parameter(motor_param_t paramID, uint16_t* A, uint16_t* B, uint16_t* C);
	int set_power(bool power_on);
	int multi_loop_angle_control_command_2(int64_t target_centidegrees, uint32_t max_speed, uint8_t* temp, int16_t* power, int16_t* speed, int64_t* encoder_centidegrees);
	int read_drive_and_motor_type(productinfo* ppi) ;
	void print_productinfo(productinfo pi);
	int print_drive_and_motor_type();

	int init_motor(uint16_t speed_kP, uint16_t speed_kI, uint16_t angle_kP, uint16_t angle_kI);

	//This should be private
	int write_rom_zero(uint16_t* encoder_zero_offset);
	
	int read_motor_state_2(uint8_t* motor_temperature_celcius, int16_t* output_power, uint16_t* motor_speed, uint16_t* encoder_position);
	int read_encoder(uint16_t* encoder_data, uint16_t* encoder_raw, uint16_t* encoder_zero_offset);

	int power_on();

	int power_off();
	

public:
	uint8_t temp;
	int16_t power;
	int16_t speed;
	int64_t encoder_centidegrees;

	uint64_t multi_loop_angle_control_command_2_attempt;
    uint64_t multi_loop_angle_control_command_2_success;

	Motor(int _id, Bus* _bus, char* _name);
    ~Motor();
	void set_debug(bool dbg);
	
	
	//******************************
	//*  AnglePID
	//******************************

	int set_angle_pid(uint16_t Kp, uint16_t Ki);
	int get_angle_pid(uint16_t* pKp, uint16_t* pKi);
	
	//******************************
	//*  SpeedPID
	//******************************
	int set_speed_pid(uint16_t Kp, uint16_t Ki);
	int get_speed_pid(uint16_t* pKp, uint16_t* pKi);
	

	//******************************
	//*  Reboot
	//******************************
	int reboot();
	
	//******************************
	//*  Update target
	//******************************
	int update(int64_t target_centidegrees);
};
