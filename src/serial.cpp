/**
 * @file serial.cpp
 * @author  Adrian Birylo <abirylo@iit.com>
 * @version 1.0
 *
 * @section LICENSE
 * Copyright (c) 2012, Illinois Tech Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by Illinois Tech Robotics.
 * 4. Neither the name of Illinois Tech Robotics nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ILLINOIS TECH ROBOTICS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ILLINOIS TECH ROBOTICS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORTIMPLIED
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * A serial library for linux
 *
*/

#include "serial.h"

Serial::Serial(){

}

Serial::~Serial(){

}

bool Serial::open_serial_port(std::string s, int baudrate){

    port = s;
    fd = open(s.data(), O_RDWR | O_NDELAY | O_ASYNC | O_NOCTTY | O_NONBLOCK, 0);
    if (fd < 0)
    {
        ROS_ERROR("Unable to open serial port %s.",  s.data());
        return false;
    }

    struct termios settings;
    memset(&settings, 0, sizeof(settings));
    settings.c_cflag = CS8 | CLOCAL | CREAD;
    settings.c_ispeed = baudrate;
    settings.c_ospeed = baudrate;
    int error = tcsetattr(fd, TCSANOW, &settings);
    if(error==-1)
    {
        close_serial_port();
        ROS_ERROR("Unable to adjust portsettings, Port: %s ", s.data());
        return false;
    }

    ROS_INFO("Serial port %s open successfully, File Descriptor: %i, Baudrate: %i", s.data(), fd, baudrate);
    return true;

}

std::string Serial::read_serial_data(){

    char buf[MAX_BUFFER_SIZE];
    std::string str = "";    
    int count = read(fd, buf, MAX_BUFFER_SIZE);

    if (count < 0){
        ROS_ERROR("Unable to read serial port: %s", port.data());
        return str;   
    }
    else{
        ROS_DEBUG("Read %i bytes, Serial Port: %s", count, port.data());
        return str.append(buf, count);
    }
}

int Serial::write_serial_data(std::string str){

    int count = write(fd, str.data(), str.length());

    if (count < (int)str.length()){
        ROS_ERROR("Only able to write %i of %i bytes to serial port, Serial Port: %s", count, str.length(), port.data());
    }
    else{
        ROS_DEBUG("Wrote %i of %i bytes, Serial Port: %s", count, str.length(), port.data());
    }
    return count;
}


bool Serial::close_serial_port(){
    int error = close(fd);
    if (error < 0){
        ROS_ERROR("Unable to close serial port, Serial Port: %s", port.data());
    }
    else{
        ROS_DEBUG("Serial port closed successfully, Serial Port: %s", port.data());
    }
    return error;
}

int Serial::get_fd(){
    return fd;
}

std::string Serial::get_port_name(){
    return port;
}
