#include "LidarLiteV3Lib.h"

// Interface for Lidar-Lite V2 (Blue Label) with NVIDIA Jetson TK1
//2.000000


LidarLite::LidarLite(){
	error = 0;
}
LidarLite::~LidarLite(){
  delete Lidar_;
}
LidarLite::LidarLite(I2C_Device* Lidar){
	error = 0;
  Lidar_ = Lidar;

}
// Return the current calculated distance in centimeters
int LidarLite::getDistance(){
    int ioResult ;
    int msb, lsb ;
    ioResult = Lidar_->write_I2CDevice(kLidarLiteCommandControlRegister,kLidarLiteMeasure);
    if (ioResult < 0) {
        return ioResult ;
    }
    ioResult = Lidar_->read_I2CDevice(kLidarLiteCalculateDistanceMSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        msb = ioResult ;
    }
    ioResult = Lidar_->read_I2CDevice(kLidarLiteCalculateDistanceLSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        lsb = ioResult ;
    }

    int distance = (msb << 8) + lsb ;

    return distance ;
}
// Return the previous measurement in centimeters
int LidarLite::getPreviousDistance() {

    int ioResult ;
    int msb, lsb ;
    ioResult = Lidar_->read_I2CDevice(kLidarLitePreviousMeasuredDistanceMSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        msb = ioResult ;
    }
    ioResult = Lidar_->read_I2CDevice(kLidarLitePreviousMeasuredDistanceLSB);
    if (ioResult < 0) {
        return ioResult ;
    } else {
        lsb = ioResult ;
    }

    int distance = (msb << 8) + lsb ;

    return distance ;
}
// Return the velocity (rate of change) in centimeters; +/-
// Velocity is returned from the Lidar-Lite as an 8-bit 2's complement number
// The returned value is converted to a signed integer
int LidarLite::getVelocity(){
    int ioResult = Lidar_->read_I2CDevice(kLidarLiteVelocityMeasurementOutput);
    if (ioResult == 255) {
        return 0 ;
    }
    if (ioResult > 127) {

        return  ioResult - 256 ;
    }
    return ioResult ;
}
// Return the Lidar-Lite hardware version
int LidarLite::getHardwareVersion(){
    return Lidar_->read_I2CDevice(kLidarLiteHardwareVersion) ;
}
// Return the Lidar-Lite software version
int LidarLite::getSoftwareVersion() {
    return Lidar_->read_I2CDevice(kLidarLiteSoftwareVersion) ;
}
// Return the last i/o error
int LidarLite::getError(){
    return Lidar_->error ;
}


