#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <NXPS32k148Lib.h>
#include <I2C_DeviceLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <math.h>


ros::Publisher pub;
 	
I2C_Device *NXP     = new I2C_Device(0, NXPS32K148_I2CAddress);
NXPs32k148 *board   = new NXPs32k148(NXP,0);

void callback_receive_references(const std_msgs::Float32MultiArray& msg){
	
	

    	board->set_reference_points(msg.data.at(0),msg.data.at(1),msg.data.at(2));
    	board->send_acceleration_breaking_direction_one_time();
	pub.publish(msg);	
	
}


int main ( int argc, char **argv)
{

	ros::init(argc, argv, "nxp_communication_node");
	ros::NodeHandle nh;

	pub = nh.advertise<std_msgs::Float32MultiArray>("/I2C/receive",1000);
	ros::Subscriber sub = nh.subscribe("/I2C/nxp_communication",1000,callback_receive_references);

	ros::Rate rate(10);

    	

    	std::vector<I2C_Device*> I2C_BUS1;

    	I2C_BUS1.push_back(NXP);

 
   

 
    	ros::spin();
	
	//float counter = 250;
    	//time_t timer1,timer2;
    	//time(&timer1);


    	// 27 is the ESC key

/*
    while (ros::ok()){
        

        time(&timer2);
        if(difftime(timer2,timer1) > 2){//seconds
			counter+=20;
			time(&timer1);//restart timer
			board->set_reference_points(counter,counter+1,counter+2);
			//break;
		}
        

		
	//std_msgs::Int64 msg;
	//msg.data = 125;
	//pub.publish(msg);
	//rate.sleep();
	}
	*/
	
    delete board;

}

