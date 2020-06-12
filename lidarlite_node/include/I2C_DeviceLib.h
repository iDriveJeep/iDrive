#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H

#include <vector>
#include <iostream>
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

#endif //
