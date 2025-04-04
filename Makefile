test: main 
	./main /dev/ttyCH341USB0

main: main.cpp lkmotor.h
	g++ -o main main.cpp

# Clean built files
clean:
	rm -rf main

.PHONY: clean test