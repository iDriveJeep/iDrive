#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <std_msgs/String.h>
#include <jeep_msgs/yolov3_msg.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <array>
#include <cstdlib>
#define stop 80

//Valores de Sliding mode controller para reference tracking
#define C1 0.8
#define C2 0.35
#define b 0.2

#define Ts 0.01 //Tiempo de muestreo
#define K 0.0035 //Relación de pixeles a metros de la cámara
#define rad_to_deg 57.2956

static float sigma = 0;
static float out = 0;
static float integrator = 0;
static float error_anterior = 0;
static float error_lat = 0;
static int sign = 0 ;


//filtro de mediana



static ros::Publisher pub;
static std_msgs::Float32MultiArray array;
static std::array<float,8> data;

static float median_filter(){
	std::array<float,8> data_copy = data;
	std::sort(data_copy.begin(),data_copy.end());
	return ((data_copy.at(3)+data_copy.at(4))*0.5);
}

void push_n_shift_data(float new_data){
	for(int i = 6; i >= 0; i--){
		data.at(i+1) = data.at(i);
	}
	data.at(0) = new_data;
}

void callback_receive_yolov3(const jeep_msgs::yolov3_msg& msg){
	
	//array.data.clear();		
	//if (msg.name == "person"){			
		//array.data.push_back()
		//array.data[0] = 0;
		push_n_shift_data(msg.depth);
		array.data[1] = 43*exp(-(pow(0.12*median_filter(),2))); //y = 80*e^(-(0.1*x)^2) GAUSSIAN FUNCTION
		array.data[2] = array.data[1];
		pub.publish(array);
	//}
			
}
void callback_receive_steer_angle(const std_msgs::Float32& msg){
	if (abs(msg.data) < 1){
	//	array.data[0] = 0;
		
	}else {
		
		//array.data[0] = msg.data;
		}
	
//pub.publish(array);
	}

void callbackErrorLat(const std_msgs::Float32& msg)
{
	error_lat = K*msg.data*(-1);

	sigma = C1*error_lat + (error_lat-error_anterior)/Ts;
	
	if (sigma>0){
		sign = 1;
	}
	else if (sigma<0){
			sign = -1;
	}
	else{
		sign = 0;
	}
	
	integrator = integrator + sign*b*Ts;
	error_anterior = error_lat;
	out = C2*sqrt(abs(sigma))*sign + integrator;
	out = out*rad_to_deg; // Conversión de radianes a grados de las llantas
	array.data[0] = out;
	
	pub.publish(array);
	std::cout << "This is the reference controller output: " << out << std::endl;
	std::cout << "This is the lateral error: " << error_lat << std::endl;
}

int main ( int argc, char **argv)
{
		array.data = std::vector<float>(3,0);
	ros::init(argc, argv, "navigation_control_node");
	ros::NodeHandle nh;

	pub = nh.advertise<std_msgs::Float32MultiArray>("/I2C/nxp_communication",10);
	ros::Subscriber sub = nh.subscribe("/yolo_detections_topic",1000,callback_receive_yolov3);
	ros::Subscriber sub_steer = nh.subscribe("/lane_detector/steer_angle",1000,callback_receive_steer_angle);
	ros::Subscriber sub_error_lat = nh.subscribe("/lane_detector/error_lat",10,callbackErrorLat);


  	ros::spin();
	


}

