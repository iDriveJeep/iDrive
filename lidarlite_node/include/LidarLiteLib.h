// lidarlite.h
// Interface for Lidar-Lite V2 (Blue Label) with NVIDIA Jetson TK1
// The MIT License (MIT)
//
// Copyright (c) 2015 Jetsonhacks
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef LIDARLITE_H
#define LIDARLITE_H

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <vector>
#include <thread>
#include <pthread.h>
#include <chrono>
//thread safe types
#include <mutex>
typedef std::chrono::high_resolution_clock Clock;

#define bits_in_byte 8
// Information taken from PulsedLight knowledge base 5-4-15
// Internal Control Registers
// http://kb.pulsedlight3d.com/support/solutions/articles/5000549552-detailed-register-descriptions-internal
// External Control Registers
// http://kb.pulsedlight3d.com/support/solutions/articles/5000549565-detailed-register-descriptions-external

// I2C Slave Address
#define kLidarLiteI2CAddress                    0x62
// I2C NXP MCU
#define NXPS32K148_I2CAddress                   0x44
// I2C IMU
#define ACCELEROMETER_I2CAddress                0x45

// Internal Control Registers
#define kLidarLiteCommandControlRegister        0x00    // Command Control Register
#define kLidarLiteVelocityMeasurementOutput     0x09    // Velocity [Read Only]: in .1 meters/sec (8 bit signed value)
// High byte set means read two bytes
#define kLidarLiteCalculateDistanceMSB          0x8f    // Calculated distance in cm (difference between signal and reference delay)
                                                        // High byte of calculated delay of signal [Read Only]: reference – calculated after correlation record processing
                                                        // If the returned MSB is 1 then the reading is not considered valid.

#define kLidarLiteCalculateDistanceLSB          0x10    // Low byte of calculated delay of signal [Read Only]: reference – calculated after correlation record processing
#define kLidarLitePreviousMeasuredDistanceMSB   0x94    // Previous high byte of calculated delay of signal
#define kLidarLitePreviousMeasuredDistanceLSB   0x15    // Previous low byte of calculated delay of signal

// External Control Registers
#define kLidarLiteHardwareVersion               0x41    // Hardware Version: revisions begin with 0x01

#define kLidarLiteSoftwareVersion               0x4f    // Software Version: Revisions begin with 0x01

// Register Command
#define kLidarLiteMeasure                       0x04    // Take acquisition & correlation processing with DC correction


class I2C_Device
{
public:
    I2C_Device(unsigned char _kI2CBus, char _I2CDevice_Address);
    ~I2C_Device() ;
    char          I2CDevice_Address;
    unsigned char kI2CBus;
    int           I2C_FileDescriptor;
    int           error;

    int   write_I2CDevice (int writeRegister, int writeValue);
    int   write_I2CDevice_block_of_u8(std::vector<std::uint8_t> bloques);
    int   read_I2CDevice  (int readRegister);
private:

    bool  open_I2CDevice  ();
    void  close_I2CDevice ();
};

class LidarLite
{
public:
    //unsigned char kI2CBus ;         // I2C bus of the Lidar-Lite
    //int kI2CFileDescriptor ;        // File Descriptor to the Lidar-Lite
    int error;
    LidarLite(I2C_Device* Lidar);
    LidarLite();
    ~LidarLite();

    //bool openLidarLite() ;                   // Open the I2C bus to the Lidar-Lite
    //void closeLidarLite();                   // Close the I2C bus to the Lidar-Lite
    //int writeLidarLite(int writeRegister,int writeValue) ;
    //int readLidarLite(int readRegister) ;
    int getDistance         () ;
    int getPreviousDistance () ;
    int getVelocity         () ;
    int getHardwareVersion  () ;
    int getSoftwareVersion  () ;
    int getError            () ;
  private:
    I2C_Device* Lidar_;

};
union float_to_hex {
  float flotante;
  std::uint32_t hex;
};

class NXPs32k148
{
public:
  int error;
  NXPs32k148(I2C_Device* NXP);
  ~NXPs32k148();
  /**Manda la información cada 10ms*/

  void set_reference_points(float acc, float dir, float brk);
  void send_acceleration_breaking_direction_one_time();
private:
  std::mutex mtx;
  std::uint8_t get_n_byte(std::uint32_t un, int pos);
  Clock::time_point time_count, time_begin;
  std::thread sending_;
  void send_acceleration_breaking_direction();
  I2C_Device* NXP_;
  //Se usa la union para estar checando constantemente el numero ingresado al float pero en hexadecimal.
  float_to_hex acceleration_  = {0.0};
  float_to_hex direction_     = {0.0};
  float_to_hex break_         = {0.0};
  //Mete los 3 floats dentro del arreglo
  std::array<float_to_hex*,3> data_to_send = {{{&direction_},{&break_},{&acceleration_}}};
  bool kill_i2c_thread = 0;
  double count_cycles = 0;
  std::chrono::microseconds step = std::chrono::microseconds(10003);
};

#endif // LIDARLITE_H
