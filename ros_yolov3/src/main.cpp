#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <pthread.h>
#include <sched.h>
#include <atomic>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jeep_msgs/yolov3_msg.h>


// This is a modified version of https://github.com/AlexeyAB/darknet/blob/master/src/yolo_console_dll.cpp
// Basically simplified and using the ZED SDK

#define OPENCV
#define GPU

#include <sl_zed/Camera.hpp>

#include "yolo_v2_class.hpp"    // https://github.com/AlexeyAB/darknet/blob/master/src/yolo_v2_class.hpp
#include <opencv2/opencv.hpp>





//ros::Publisher
typedef struct threadrec_t{
		int available;
		pthread_t thread;
		pthread_attr_t attributes;
		struct sched_param param;
} threadrec_t;

//thread priority
class thread
{  private:
	pthread_attr_t 	attributes_;
    sched_param 	sch_params_;
    int 			top_priority_;
    int 			policy_;
    
  public:
    thread();
    void setScheduling(std::thread &th, int policy, int priority) {
        sch_params_.sched_priority = priority;
        if(pthread_setschedparam(th.native_handle(), policy, &sch_params_)) {
            std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        }
    };
    void setScheduling_top(std::thread &th) {
		pthread_getattr_np			(th.native_handle(),&attributes_);
		pthread_attr_getschedpolicy	(&attributes_, &policy_);
		top_priority_ 				= sched_get_priority_max(policy_);
        sch_params_.sched_priority 	= top_priority_;
        if(pthread_setschedparam(th.native_handle(), policy_, &sch_params_)) {
            std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        }
    };

};

class bbox_t_3d {
public:
    bbox_t bbox;
    sl::float3 coord;

    bbox_t_3d(bbox_t bbox_, sl::float3 coord_) {
        bbox = bbox_;
        coord = coord_;
    }
};

float getMedian(std::vector<float> &v) {
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

std::vector<bbox_t_3d> getObjectDepth(std::vector<bbox_t> &bbox_vect, sl::Mat &xyzrgba) {
    sl::float4 out(NAN, NAN, NAN, NAN);
    bool valid_measure;
    int i, j;
    const int R_max = 4;

    std::vector<bbox_t_3d> bbox3d_vect;

    for (auto &it : bbox_vect) {

        int center_i = it.x + it.w * 0.5f, center_j = it.y + it.h * 0.5f;

        std::vector<float> x_vect, y_vect, z_vect;
        for (int R = 0; R < R_max; R++) {
            for (int y = -R; y <= R; y++) {
                for (int x = -R; x <= R; x++) {
                    i = center_i + x;
                    j = center_j + y;
                    xyzrgba.getValue<sl::float4>(i, j, &out, sl::MEM_CPU);
                    valid_measure = std::isfinite(out.z);
                    if (valid_measure) {
                        x_vect.push_back(out.x);
                        y_vect.push_back(out.y);
                        z_vect.push_back(out.z);
                    }
                }
            }
        }

        if (x_vect.size() * y_vect.size() * z_vect.size() > 0) {
            float x_med = getMedian(x_vect);
            float y_med = getMedian(y_vect);
            float z_med = getMedian(z_vect);

            bbox3d_vect.emplace_back(it, sl::float3(x_med, y_med, z_med));
        }
    }

    return bbox3d_vect;
}

void draw_boxes(cv::Mat mat_img, std::vector<bbox_t_3d> result_vec, std::vector<std::string> obj_names) {
    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.bbox.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.bbox.x, i.bbox.y, i.bbox.w, i.bbox.h), color, 2);
        if (obj_names.size() > i.bbox.obj_id /*&& (obj_names[i.bbox.obj_id] == "person")*/ ) {
	    //cv::rectangle(mat_img, cv::Rect(i.bbox.x, i.bbox.y, i.bbox.w, i.bbox.h), color, 2); /* Changed*/

            std::string obj_name = obj_names[i.bbox.obj_id];
            std::stringstream stream, probability;
            probability <<std::fixed << std::setprecision(2) << i.bbox.prob*100;
            stream << std::fixed << std::setprecision(2) << sqrt(i.coord.x * i.coord.x + i.coord.y * i.coord.y + i.coord.z * i.coord.z);
            obj_name += " - " + probability.str() + "% - " + stream.str() + "m";
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int const max_width = (text_size.width > i.bbox.w + 2) ? text_size.width : (i.bbox.w + 2);
            cv::rectangle(mat_img, cv::Point2f(std::max((int) i.bbox.x - 1, 0), std::max((int) i.bbox.y - 30, 0)),
                    cv::Point2f(std::min((int) i.bbox.x + max_width, mat_img.cols - 1),
                    std::min((int) i.bbox.y, mat_img.rows - 1)),
                    color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.bbox.x, i.bbox.y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2,
                    cv::Scalar(0, 0, 0), 2);
        }
    }
}

void publish_with_ROS(std::vector<bbox_t_3d> result_vec, std::vector<std::string> obj_names, ros::Publisher &pub){
	jeep_msgs::yolov3_msg msg;
	msg.name  = "none";
	msg.depth = 10000.00;
	msg.prob  = 0.00;
	for(auto& i:result_vec) {
		if(obj_names.size() > i.bbox.obj_id){
			if(obj_names[i.bbox.obj_id] == "person" && msg.depth > sqrt(i.coord.x * i.coord.x + i.coord.y * i.coord.y + i.coord.z * i.coord.z)){
				msg.name  = obj_names[i.bbox.obj_id] ;
				msg.depth = sqrt(i.coord.x * i.coord.x + i.coord.y * i.coord.y + i.coord.z * i.coord.z);
				msg.prob  = i.bbox.prob*100;
				
			}
		}
	}
	
	pub.publish(msg);
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for (std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "Object names loaded \n";
    return file_lines;
}

cv::Mat slMat2cvMat(sl::Mat &input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1:
            cv_type = CV_32FC1;
            break;
        case sl::MAT_TYPE_32F_C2:
            cv_type = CV_32FC2;
            break;
        case sl::MAT_TYPE_32F_C3:
            cv_type = CV_32FC3;
            break;
        case sl::MAT_TYPE_32F_C4:
            cv_type = CV_32FC4;
            break;
        case sl::MAT_TYPE_8U_C1:
            cv_type = CV_8UC1;
            break;
        case sl::MAT_TYPE_8U_C2:
            cv_type = CV_8UC2;
            break;
        case sl::MAT_TYPE_8U_C3:
            cv_type = CV_8UC3;
            break;
        case sl::MAT_TYPE_8U_C4:
            cv_type = CV_8UC4;
            break;
        default:
            break;
    }
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}


std::mutex data_lock;
cv::Mat cur_frame;
std::vector<bbox_t> result_vect;
std::atomic<bool> exit_flag, new_data;

void detectorThread(std::string cfg_file, std::string weights_file, float thresh) {
    Detector detector(cfg_file, weights_file);
    std::shared_ptr<image_t> det_image;
    cv::Size const frame_size = cur_frame.size();
	float count = cv::getTickCount();
    while (!exit_flag) {
        if (new_data) {
            data_lock.lock();
            det_image = detector.mat_to_image_resize(cur_frame);
            result_vect = detector.detect_resized(*det_image, frame_size.width, frame_size.height, thresh, false); // true
            for(const auto& position : result_vect)
				std::cout << "X : " << (float)position.x/(float)frame_size.width << ",Y: " << (float)position.y/(float)frame_size.height << ",W: " << position.w << ",H: " << position.h << std::endl;
            std::cout << "FPS: " << cv::getTickFrequency()/(cv::getTickCount()-count) << std::endl << "\n\n\n\n\n\n\n\n";
            count = cv::getTickCount();
            data_lock.unlock();
            new_data = false;
        } else sl::sleep_ms(1);
    }
}

struct arg_files{
		std::string cfg_file;
		std::string weights_file;
		float thresh;
};

void* detectorThread_in_c(void * ptr) {
	
	arg_files* args = (arg_files*)ptr;
	
	std::string cfg_file = args->cfg_file;
	std::string weights_file = args->weights_file;
	float thresh = args->thresh;
	
    Detector detector(cfg_file, weights_file);
    std::shared_ptr<image_t> det_image;
    cv::Size const frame_size = cur_frame.size();
	float count = cv::getTickCount();
    while (!exit_flag) {
        if (new_data) {
            data_lock.lock();
            det_image = detector.mat_to_image_resize(cur_frame);
            result_vect = detector.detect_resized(*det_image, frame_size.width, frame_size.height, thresh, false); // true
            for(const auto& position : result_vect)
				std::cout << "X : " << (float)position.x/(float)frame_size.width << ",Y: " << (float)position.y/(float)frame_size.height << ",W: " << position.w << ",H: " << position.h << std::endl;
            std::cout << "FPS: " << cv::getTickFrequency()/(cv::getTickCount()-count) << std::endl << "\n\n\n\n\n\n\n\n";
            count = cv::getTickCount();
            data_lock.unlock();
            new_data = false;
        } else usleep(10);
    }
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv,"yoloV3_zed_node");
    ros::NodeHandle nh;
	ros::Publisher pub_main = nh.advertise<jeep_msgs::yolov3_msg>("/yolo_detections_topic",10);
	
	
	
	ROS_INFO("ZED YOLOV3 NODE");
	std::string names_file = "/home/nvidia/libdarknet/data/coco.names";
   	std::string weights_file = "/home/nvidia/libdarknet/yolov3-tiny.weights";
 	std::string cfg_file = "/home/nvidia/libdarknet/cfg/yolov3-tiny.cfg";
   	//std::string weights_file = "yolov3.weights";
 	//std::string cfg_file = "yolov3.cfg";
   	std::string filename;
	//ros::Rate rate (10);



    if (argc > 3) { //voc.names yolo-voc.cfg yolo-voc.weights svo_file.svo
        names_file = argv[1];
        cfg_file = argv[2];
        weights_file = argv[3];
        if (argc > 4)
            filename = argv[4];
    } else if (argc > 1) filename = argv[1];

    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_HD720;
    init_params.coordinate_units = sl::UNIT_METER;
    if (!filename.empty()) init_params.svo_input_filename.set(filename.c_str());

    std::cout << zed.open(init_params) << std::endl;
    zed.grab();

    float const thresh = (argc > 5) ? std::stof(argv[5]) : 0.38;
    auto obj_names = objects_names_from_file(names_file);

    sl::Mat left, cur_cloud;
    zed.retrieveImage(left);
    zed.retrieveMeasure(cur_cloud, sl::MEASURE_XYZ);
    slMat2cvMat(left).copyTo(cur_frame);
    exit_flag = false;
    new_data = false;

    //std::thread detect_thread(detectorThread, cfg_file, weights_file, thresh);
    
    //create thread as c code---------------------------------------------------
    int policy;
    threadrec_t detect_thread = {1};
    pthread_attr_init(&detect_thread.attributes);
    pthread_attr_getschedparam(&detect_thread.attributes,&detect_thread.param);
    //pthread_attr_setschedpolicy(&detect_thread.attributes,SCHED_FIFO);
    pthread_attr_getschedpolicy(&detect_thread.attributes,&policy);
    detect_thread.param.sched_priority = sched_get_priority_max(policy);
    pthread_attr_setschedparam(&detect_thread.attributes,&detect_thread.param);
    
    arg_files args = {cfg_file, weights_file, thresh};
    
    if(detect_thread.available)	{
			detect_thread.available = 0;
			if(pthread_create(&detect_thread.thread, &detect_thread.attributes, detectorThread_in_c, (void*)&args)) std::cerr << "Thread creation failed\n";
	}
	//--------------------------------------------------------------------------	
    

    while (!exit_flag) {

        if (zed.grab() == sl::SUCCESS) {
            zed.retrieveImage(left);
            data_lock.lock();
            cur_frame = slMat2cvMat(left);
            data_lock.unlock();
            new_data = true;

            zed.retrieveMeasure(cur_cloud, sl::MEASURE_XYZ);

            data_lock.lock();
            auto result_vec_draw = getObjectDepth(result_vect, cur_cloud);
            data_lock.unlock();
			
            publish_with_ROS(result_vec_draw, obj_names, pub_main);
            draw_boxes(cur_frame, result_vec_draw, obj_names);
            cv::imshow("ZED", cur_frame);
        }

        int key = cv::waitKey(1); // 3 or 16ms
        if (key == 'p') while (true) if (cv::waitKey(100) == 'p') break;
        if (key == 27 || key == 'q' || !(ros::ok())) exit_flag = true;
    }
    //join as c code ---------------------------------------------------------
	pthread_join(detect_thread.thread , 0);
	//------------------------------------------------------------------------
    //detect_thread.join();
    zed.close();
    return 0;
}
