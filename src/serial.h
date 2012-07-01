//  serial.h -

/*
*   Copyright (c) 2012 Illinois Tech Robotics.
*   All rights reserved.
*
*   Redistribution and use in source and binary forms are permitted
*   provided that the above copyright notice and this paragraph are
*   duplicated in all such forms and that any documentation,
*   advertising materials, and other materials related to such
*   distribution and use acknowledge that the software was developed
*   by Illinois Tech Robotics.  The name of the
*   organization may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*   THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
*   IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
*   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOS
*/

#ifndef SERIAL_H
#define SERIAL_H

#include <fcntl.h>
#include "ros/ros.h"
#include <termios.h>

const int MAX_BUFFER_SIZE = 512; //this number can increase (at 400hz can support 1.6 mega baud)

class Serial
{

public:      
    Serial();
    ~Serial();
    bool close_serial_port();
    bool open_serial_port(std::string s, int baud);
    int write_serial_data(std::string s);
    int get_fd();
    std::string read_serial_data();
    std::string get_port_name();

private:
    int fd;
    std::string port;

};

#endif //SERIAL_H
