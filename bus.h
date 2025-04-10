#pragma once

#include <stdint.h>
#include <vector>
#include <mutex>
#include <cstdarg>



class Bus
{
private:
    char* serial_port_name;
	int serial_port;
	mutable std::mutex mutex;
	bool debug;
	void dbg_printf(const char* format, ...);
    void ser_write(std::vector<uint8_t> msg);
    std::vector<uint8_t> ser_read();    
public:
    Bus(char* _serial_port_name);
    ~Bus();
	void set_debug(bool dbg);
	std::vector<uint8_t> exchangebytes(std::vector<uint8_t> msg, uint8_t chars, uint64_t patience_ns);
	std::vector<uint8_t> exchangebytes(std::vector<uint8_t> msg, uint8_t chars);		
};

