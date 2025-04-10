#include <stdio.h>
#include <math.h>
#include <signal.h>

#include "utils.h"
#include "bus.h"
#include "motor.h"


int running = 1;
std::vector<Motor*> motors;

void sigint_handler(int sig) {
    
    printf("signal\n");
    running = false;
}

void usage()
{
    printf("usage: ./main [serial_port]\n");
    printf("usage: ./main /dev/ttyUSB0\n");
}

int main(int argc, char* argv[])
{
    if(argc != 2 )
    {
        usage();    
        exit(0);
    }
    
    printf("Using: serial port: %s\n", argv[1]);

    signal(SIGINT, sigint_handler);
    
    bool dbg = false;

    Bus* bus = new Bus(argv[1]);
    
    for(int i=0;i<2;i++)
    {
        char buf[128];
        sprintf(buf, "motor_%d", i);
        motors.push_back(new Motor(i+1, bus, buf));    
        motors[i]->set_debug(dbg);
    }
    
    bus->set_debug(dbg);

    uint64_t start_time = get_timestamp_ns();
    uint64_t last_time = start_time;
    while(running)
    {
        uint64_t now = get_timestamp_ns();

        float t = (now - start_time)/1000000000.0;
        
        int64_t target_centidegrees =  5000 * sin(t*3);
        
        for(Motor* motor : motors) {
            motor->update(target_centidegrees);
        
        }
        
        float fps = 0;
        uint64_t dt = now - last_time;
        last_time = now;
        if(dt > 0)
        {
            fps = 1000000000.0/dt;
        }
        printf("fps:%0.1f Target:%+05ld", fps, target_centidegrees);        
        for(Motor* motor : motors) {
            printf(" (m2: %lu/%lu %lu)", motor->multi_loop_angle_control_command_2_attempt - motor->multi_loop_angle_control_command_2_success, motor->multi_loop_angle_control_command_2_attempt, motor->encoder_centidegrees);        
        }
        //usleep(10000);
        
        printf("\n");
    }

    for(Motor* motor : motors) {
        delete motor;
    }
    motors.clear();
    delete bus;
    



    printf("fin\n");    
    return 0;
}