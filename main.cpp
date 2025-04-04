#include <stdio.h>
#include <math.h>
#include <signal.h>
#include "lkmotor.h"

int running = 1;


void sigint_handler(int sig) {
    printf("signal\n");
    running = false;
}

int init_motor(Motor* m, uint16_t speed_kP, uint16_t speed_kI, uint16_t angle_kP, uint16_t angle_kI, uint16_t raw_encoder_zero)
{
    printf("********** Motor %s **********\n", m->name);
        
    if(m->print_drive_and_motor_type() == 0)
    {
        if(m->power_on() == 0)
        {
            uint16_t encoder, raw, offset;            
            if(m->read_encoder(&encoder, &raw, &offset) == 0)
            {

                //Todo raw_encoder_zero

                printf("Encoder:%d Raw:%d Offset:%d\n", encoder, raw, offset);
                if(m->set_speed_pid(speed_kP, speed_kI) == 0)
                {
                    if(m->set_angle_pid(angle_kP, angle_kI) == 0)
                    {
                        return 0;
                    }
                    else
                    {
                        printf("Unable to set angle PID parameters\n");    
                    }
                }
                else
                {
                    printf("Unable to set speed PID parameters\n");    
                }
            }
            else
            {
                printf("Unable to read encoder\n");
            }
        }
        else
        {
            printf("Unable to turn %s motor on\n", m->name);
        }
    }
    else
    {
        printf("Unable to find %s motor\n", m->name);
    }    
    return 1;
}

int main(int argc, char* argv[])
{
    printf("Hello lkmotor\n");
    if(argc != 2)
    {
        printf("usage: ./main /dev/ttyUSB0\n");
        exit(0);
    }
    printf("Using serial port: %s\n", argv[1]);
    Bus bus(argv[1]);
    bus.set_debug(true);
    const char* n = "A";
    Motor motor(1, 16, &bus, n);
    motor.set_debug(true);
    bool initialized = false;
    float x = 0;
    uint64_t good_transmissions = 0;
    uint64_t bad_transmissions = 0;

    uint64_t bad_checksum_in_second_part_of_response = 0;
    while(running)
    {
        if(!initialized)
        {
            if(init_motor(&motor, 50,20,50,20, 0) == 0)
            {
                initialized = true;
            }
        }
        else
        {
            int64_t target_centidegrees = 18000 + 18000 * sin(x / 100.0);
            uint32_t max_speed = 0;
            uint8_t temp;
            int16_t power;
            int16_t speed;
            int64_t encoder_centidegrees;

            int r = motor.multi_loop_angle_control_command_2(target_centidegrees, max_speed, &temp, &power, &speed, &encoder_centidegrees);
            if(r == 0)
            {
                good_transmissions++;
                printf("Target: % 8ld Encoder: % 8ld errors:%ld/%ld  (errors in checksum of second part of response: %ld)\n", target_centidegrees, encoder_centidegrees, bad_transmissions,(good_transmissions+bad_transmissions), bad_checksum_in_second_part_of_response);
            }
            else
            {
                printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
                if(r == 3)
                {
                    bad_checksum_in_second_part_of_response ++;                    
                }
                bad_transmissions++;
                if(bad_checksum_in_second_part_of_response >= 10)
                {
                    running = false;
                }
                printf("Communication error\n");
                initialized = false;
            }
            x += 1;
        }        
    }
    printf("fin\n");    
    return 0;
}