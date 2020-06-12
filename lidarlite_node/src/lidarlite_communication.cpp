#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <LidarLiteV3Lib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <math.h>


int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

int main ( int argc, char **argv)
{

	ros::init(argc, argv, "I2C_LidarLite_Communication_node");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Int64>("/I2C/LidarLite_data",10);

	ros::Rate rate(10);
    	// Objects belong to I2C Class
    	I2C_Device *Lidar = new I2C_Device(0, kLidarLiteI2CAddress);
    	I2C_Device *IMU   = new I2C_Device(1, ACCELEROMETER_I2CAddress);
    	I2C_Device *NXP   = new I2C_Device(0, NXPS32K148_I2CAddress);

    	std::vector<I2C_Device*> I2C_BUS1;
    	I2C_BUS1.push_back(Lidar);
    	I2C_BUS1.push_back(IMU);
    	I2C_BUS1.push_back(NXP);

    	LidarLite *lidarLite 	= new LidarLite(Lidar);
    	//NXPs32k148 *board		= new NXPs32k148(NXP);

    	int hardwareVersion = lidarLite->getHardwareVersion() ;
    	int softwareVersion = lidarLite->getSoftwareVersion() ;
    	printf("Hardware Version: %d\n",hardwareVersion) ;
    	printf("Software Version: %d\n",softwareVersion) ;
    	float counter = 250;
    	time_t timer1,timer2;
    	time(&timer1);
    	//board->set_reference_points(counter,counter+2,counter+4);
    	//board->send_acceleration_breaking_direction_one_time();

    	// 27 is the ESC key


    while (ros::ok()){
        int distance = lidarLite->getDistance();
        if (distance < 0) {
            int llError ;
            llError = lidarLite->getError() ;
            ROS_INFO("Lidar-Lite error: %d\n",llError) ;
        } else {
            int previousDistance = lidarLite->getPreviousDistance();
            int velocity = lidarLite->getVelocity();
            ROS_INFO("Distance: %5d cm  | Previous Distance: %5d cm  | Velocity: % 8d \n",distance,previousDistance,velocity);
        }
        /*
        time(&timer2);
        if(difftime(timer2,timer1) > 2){//seconds
			counter+=20;
			time(&timer1);//restart timer
			board->set_reference_points(counter,counter+1,counter+2);
			//break;
		}
        */

		
	std_msgs::Int64 msg;
	msg.data = distance;
	pub.publish(msg);
	rate.sleep();
	}
	
    delete lidarLite;
    delete IMU;
    //delete board;
}

