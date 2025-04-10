#include "utils.h"

#include <stdio.h>
#include <ctime>
#include <assert.h>

/*

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

*/


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

//Print binary message
void print_msg(const std::vector<uint8_t> msg)
{
	for(size_t i=0;i<msg.size();i++)
	{
		printf(" %02x", msg[i]);
	}
	printf("\n");
}
