#include "I2C_DeviceLib.h"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdexcept>
#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

I2C_Device::I2C_Device(unsigned char _kI2CBus, char _I2CDevice_Address){
    kI2CBus = _kI2CBus;                         // Desired I2C bus on Jetson TX2
    I2CDevice_Address = _I2CDevice_Address;     // Desired I2C Address on Device
    error = 0 ;
    error = this->open_I2CDevice();
    if(error < 0){
      //STOPS EXECUTION
      std::string errorMessage = std::string("Error: on opening device address")+I2CDevice_Address;
      throw std::runtime_error(errorMessage);
    }
}
I2C_Device::~I2C_Device(){
    close_I2CDevice();
}
// Returns true if device file descriptor opens correctly, false otherwise
bool I2C_Device::open_I2CDevice(){
    char fileNameBuffer[32];
    sprintf(fileNameBuffer,"/dev/i2c-%d", kI2CBus);
    I2C_FileDescriptor = open(fileNameBuffer, O_RDWR);
    if (I2C_FileDescriptor < 0) {
        // Could not open the file
        error = errno ;
        return false ;
    }
    if (ioctl(I2C_FileDescriptor, I2C_SLAVE, I2CDevice_Address) < 0) {
        // Could not open the LIDAR on the bus
        error = errno ;
        return false ;
    }
    return true ;
}
void I2C_Device::close_I2CDevice(){
    if (I2C_FileDescriptor > 0) {
        close(I2C_FileDescriptor);
        // WARNING - This is not quite right, need to check for error first
        I2C_FileDescriptor = -1 ;
    }
}
// Read the given register on the Lidar-Lite
int I2C_Device::read_I2CDevice(int readRegister){
    // Do not use i2c_smbus_read_byte_data here ; LidarLite V2 needs STOP between write and read
    int toReturn ;
    toReturn = i2c_smbus_write_byte(I2C_FileDescriptor, readRegister) ;
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    toReturn = i2c_smbus_read_byte(I2C_FileDescriptor) ;
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}
// Write the the given value to the given register on the Lidar-Lite
int I2C_Device::write_I2CDevice(int writeRegister, int writeValue){
    int toReturn = i2c_smbus_write_byte_data(I2C_FileDescriptor, writeRegister, writeValue);
    //i2c_smbus_write_byte() to write single byte.
    // Wait a little bit to make sure it settles
    //usleep(10000);
    if (toReturn < 0) {
        error = errno ;
        toReturn = -1 ;
    }
    return toReturn ;
}
int I2C_Device::write_I2CDevice_block_of_u8(std::vector<std::uint8_t> bloques ){
    //int i2c_master_send               (const struct i2c_client *client, const char *buf, int count)
    //s32 i2c_smbus_write_block_data    (const struct i2c_client *client, u8 command, u8 length, const u8 *values)
    //s32 i2c_smbus_write_i2c_block_data(const struct i2c_client *client, u8 command, u8 length, const u8 *values)
    //el primer byte mandado va a ser 0

    std::cout << "se enviaran " << bloques.size() << " datos \n";
    int toReturn = i2c_smbus_write_block_data(I2C_FileDescriptor, 0x23, bloques.size(), &bloques[0]);
    if(toReturn < 0){
      error = errno;
      toReturn = -1;
    }
    return toReturn;
}

