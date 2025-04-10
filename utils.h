#pragma once

#include <stdint.h>
#include <vector>

uint64_t get_timestamp_ns();
uint64_t get_timestamp_ms();
void print_msg(const std::vector<uint8_t> msg);
