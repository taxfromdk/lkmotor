# Compiler to use
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -std=c++11 -Wfatal-errors

# Target executable name
TARGET = main

# Source files - main.cpp contains the main() function, others are support files
SRCS = main.cpp bus.cpp motor.cpp utils.cpp

# Default target
all: $(TARGET)

# Build the executable
# main.cpp includes the main() function and uses functionality from other files
$(TARGET): $(SRCS) motor.h motor.h bus.h 
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)


test0: $(TARGET)
	./$(TARGET) /dev/ttyCH9344USB0

test1: $(TARGET)
	./$(TARGET) /dev/ttyCH9344USB1

# Clean built files
clean:
	rm -f $(TARGET)

# Phony targets
.PHONY: clean test0 test1 all