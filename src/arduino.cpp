/**
 * @file arduino.cpp
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
 * Reads and Writes Serial data to/from the arduino.
 *
 */

#include "arduino.h"
#include "serial.h"

/**
 * The start the Arduino node
 *
 * This read command line arguments if applicable the opens the serial port.
 * Then reads the serial port at 400 Hz.
 *
 */
int main(int argc, char **argv){

    std::string usb_port_name = ARDUINO_SERIAL_PORT_DEFAULT;
    int baudrate = ARDUINO_BAUDRATE_DEFAULT;
    int opt;

    //to initialize ros with command line arguments and name of the node
    ros::init(argc, argv, "arduino_handler");

    ros::NodeHandle n;

    //get any params/ set any params
    n.param<std::string>("dev", usb_port_name, usb_port_name);
    n.param<int>("baud_rate", baudrate, baudrate);

    //commandline arguments take precedance to params
    while(( opt = getopt(argc, argv, "u:b:")) != -1){
        switch (opt){
            case 'u':
                usb_port_name.assign(optarg);
                n.setParam("dev",usb_port_name);
                break;
            case 'b':
                baudrate = atoi(optarg);
                n.setParam("baud_rate", baudrate);
                break;
            default:
                break;
        }
    }

    bIsConnected = false;
    nmeaChecksumFailedCount = 0;

    //advertise(topic_name, queue_size, latch)
    tIsAduinoConnected = n.advertise<std_msgs::Bool>(IsArduinoConnected_T, 1, true);
    std_msgs::Bool msg_isArduinoConnected;
    msg_isArduinoConnected.data = bIsConnected;
    tIsAduinoConnected.publish(msg_isArduinoConnected);

    tIMUData = n.advertise<ghost::imu>(IMU_T, 1, true);

    Serial serial;

    if (serial.open_serial_port(usb_port_name, baudrate) == false){
        ros::shutdown();
    }

    ros::Timer watchdogTimer = n.createTimer(ros::Duration(ARDUINO_WATCHDOG_DURATION), arduinoWatchdog);
    ros::Timer heartbeatTimer = n.createTimer(ros::Duration(ARDUINO_HEARTBEAT_DURATION), arduinoHeartBeat);

    //specify the speed of the loop
    ros::Rate loop(ARDUINO_LOOP_RATE);

    std::string inBuf;

    //loop until program is closed (Ctrl-C or ros::shutdown())
    while (ros::ok()){

        //lastConnectedTime.sec

        inBuf += serial.read_serial_data();
        parse_nmea_message(inBuf);

        //to check if subscribed topics have updated
        ros::spinOnce();

        //sleep until next loop time
        loop.sleep();
    }

    watchdogTimer.stop();
    heartbeatTimer.start();

    serial.close_serial_port();

    return 0;
}

void arduinoWatchdog(const ros::TimerEvent &event){
    //to only publish the data when state changes
    static bool last_state = false;
    if (bIsConnected == true && last_state == false){
        std_msgs::Bool msg_isArduinoConnected;
        msg_isArduinoConnected.data = true;
        tIsAduinoConnected.publish(msg_isArduinoConnected);
        ROS_DEBUG("Arduino reconnected.");
    }
    else if (bIsConnected == false && last_state == true){
        std_msgs::Bool msg_isArduinoConnected;
        msg_isArduinoConnected.data = false;
        tIsAduinoConnected.publish(msg_isArduinoConnected);
        ROS_WARN("Arduino is not connected.");
    }
    bIsConnected = false; //reset the flag
}

void arduinoHeartBeat(const ros::TimerEvent &event){

}

void parse_nmea_message(std::string str){
    int start;
    int end;
    if ((start = str.find('$')) >= 0 ){
        if ((end = str.find(0x0A)) > start){
            std::string message = str.substr(start,end-start+1);
            str.erase(0, end+1);
            if (is_nmea_checksum_valid(message) == true){
                bIsConnected = true;
                std::string header = message.substr(0, 7);
                std::string remainder = message.substr(6, message.length()-6);
                if(header == "$PITRI,"){
                    //read all of the accel, gyro, mag, angles values
                    ghost::imu imu;
                    std::string substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.accelX = base64_to_12bit(substr);
                    substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.accelY = base64_to_12bit(substr);
                    substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.accelZ = base64_to_12bit(substr);
                    substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.gyroX = base64_to_12bit(substr);
                    substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.gyroY = base64_to_12bit(substr);
                    substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.gyroZ = base64_to_12bit(substr);
                    substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.magX = base64_to_12bit(substr);
                    substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.magY = base64_to_12bit(substr);
                    substr = remainder.substr(0, 2);
                    remainder.erase(0, 3);
                    imu.magZ = base64_to_12bit(substr);
                    substr = remainder.substr(0, 4);
                    remainder.erase(0, 5);
                    imu.pitch = hextofloat(substr);
                    substr = remainder.substr(0, 4);
                    remainder.erase(0, 5);
                    imu.roll = hextofloat(substr);
                    substr = remainder.substr(0, 4);
                    remainder.erase(0, 5);
                    imu.yaw = hextofloat(substr);
                    substr = remainder.substr(0,8);
                    remainder.erase(0, 9);
                    imu.time = base64_to_12bit(substr);
                    imu.header.stamp = ros::Time().now();
                    tIMUData.publish(imu);
                }
            }
            else{            
                nmeaChecksumFailedCount++;
                ROS_WARN("NMEA checksum failed, Failed Count: %i", nmeaChecksumFailedCount);
            }
        }
        else{
            str.erase(0,end+1); //remove everything in front of the end pt.
        }
    }

}

bool is_nmea_checksum_valid(std::string message){
    int checksum = 0;
    for (int i=1; i < ((int)message.length()-5); i++){
        checksum ^= message.at(i);
    }

    std::string strchecksum = message.substr(message.length()-4,message.length()-2);
    int meschecksum;
    std::stringstream ss;
    ss << std::hex << strchecksum;
    ss >> meschecksum;

    if (checksum == meschecksum){
        return true;
    }
    else{
        ROS_WARN("NMEA checksum failed, Expected: %i Recived: %i", checksum, meschecksum);
        return false;
    }
}

int base64_to_12bit(std::string substr){
    char high = substr.at(0);
    char low = substr.at(1);

    if(high >= 0x30 && high <= 0x39)
      high = high - 0x30;
    else if(high >= 0x41 && high <= 0x5A)
      high = high - 0x41 + 10;
    else if(high >= 0x61 && high <= 0x7A)
      high = high - 0x61 + 36;
    else if(high == 0x5B)
      high = 62;
    else if( high == 0x5D)
      high = 63;
    else
      high = -1;

    if(low >= 0x30 && low <= 0x39)
      low = low - 0x30;
    else if(low >= 0x41 && low <= 0x5A)
      low = low - 0x41 + 10;
    else if(low >= 0x61 && low <= 0x7A)
      low = low - 0x61 + 36;
    else if(low == 0x5B)
      low = 62;
    else if( low == 0x5D)
      low = 63;
    else
      low = -1;

    if(high == -1 or low == -1) //error
        return -1;
    else
        return (high << 6) + low;
}

float hextofloat(std::string data){
    if (data.find(',') < 8 || data.length() != 8){
        return NAN;
    }

    unsigned char chdata[4];
    for (int i=0; i<4; i++){
        std::string substr = data.substr(i*2, (i*2)+2);
        chdata[i] = hextobyte(substr);
    }

    float output = *(float *)((void *)chdata); //floating point bit level hack
    return output;
}

unsigned char hextobyte(std::string data){
    unsigned char output;
    std::stringstream ss;
    ss << std::hex << data;
    ss >> output;
    return output;
}
