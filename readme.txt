The intention with this repo is to highlight a problem with the checksum in the second part of the response of "13. Multi loop angle control command 2"

Of 754 calls to "Multi loop angle control command 2" all of the communication errors happen in the checksum of the second part of the response. 
I have not seen the error elsewhere. 

Therefore I am led to believe it is a problem in the firmware. 

Target:    35063 Encoder:    23954 errors:9/754  (errors in checksum of second part of response: 9)

My supply voltage is 19V and firmware details are in the log.

Motor and firmware
Driver Name:       DS40R7
Motor Name:        MS4015

You can test it yourself if you checkout the code and run "make clean test"

