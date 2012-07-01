/**
 * @file arduino.h
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
*/

#ifndef ARDUNIO_H
#define ARDUNIO_H

#include <fcntl.h>

#include "ros/ros.h"
#include "topics.h"
#include "serial.h"

#include "std_msgs/Bool.h"
#include "ghost/imu.h"

const float ARDUINO_WATCHDOG_DURATION = 1.0; //second
const float ARDUINO_HEARTBEAT_DURATION = .1; //second
const float ARDUINO_LOOP_RATE = 400.0; //hz
const int ARDUINO_BAUDRATE_DEFAULT = 250000;
const std::string ARDUINO_SERIAL_PORT_DEFAULT = "/dev/ttyUSB0";

ros::Publisher tIsAduinoConnected;
ros::Publisher tIMUData;

bool bIsConnected;
int nmeaChecksumFailedCount;

void arduinoWatchdog(const ros::TimerEvent &event);
void arduinoHeartBeat(const ros::TimerEvent &event);
void parse_nmea_message(std::string str);
bool is_nmea_checksum_valid(std::string message);
int base64_to_12bit(std::string substr);
float hextofloat(std::string data);
unsigned char hextobyte(std::string data);

#endif //ARDUNIO_H
