#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "net.h"
#include "mat.h"
#include "chrono"
#include "tracking.h"
#include "letnet.id.h"

#define IMG_H 240
#define IMG_W 320

const float mean_vals[3] = {0, 0, 0};
const float norm_vals[3] = {1.0/255.0, 1.0/255.0, 1.0/255.0};
const float mean_vals_inv[3] = {0, 0, 0};
const float norm_vals_inv[3] = {255.f, 255.f, 255.f};

ncnn::Net net;

// 模型路径
std::string package_path = ros::package::getPath("letnet");
std::string model_path, model_param_path;


cv::Mat score(IMG_H, IMG_W, CV_8UC1);
cv::Mat desc(IMG_H, IMG_W, CV_8UC3);
cv::Mat frame;
ncnn::Mat in;
ncnn::Mat out1, out2;

corner_tracking tracker;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        ncnn::Extractor ex = net.create_extractor();
        ex.set_light_mode(true);

        cv::resize(frame, frame, cv::Size(IMG_W, IMG_H));

        //////////////////////////  opencv image to ncnn mat  //////////////////////////
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        in = ncnn::Mat::from_pixels(frame.data, ncnn::Mat::PIXEL_BGR, frame.cols, frame.rows);
        in.substract_mean_normalize(mean_vals, norm_vals);

        //////////////////////////  ncnn forward  //////////////////////////

        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();


        ex.input(letnet_param_id::BLOB_input_1, in);
        ex.extract(letnet_param_id::LAYER__Unsqueeze, out1); // score map
        ex.extract(letnet_param_id::BLOB_33, out2); // descriptor map

        // ex.input("input", in);
        // ex.extract("score", out1);
        // ex.extract("descriptor", out2);

        //////////////////////////  ncnn mat to opencv image  //////////////////////////

        std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
        out1.substract_mean_normalize(mean_vals_inv, norm_vals_inv);
        out2.substract_mean_normalize(mean_vals_inv, norm_vals_inv);

//        memcpy((uchar*)score.data, out1.data, sizeof(float) * out1.w * out1.h);

        out1.to_pixels(score.data, ncnn::Mat::PIXEL_GRAY);
        out2.to_pixels(desc.data, ncnn::Mat::PIXEL_BGR);

        std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();


        //////////////////////////  show times  //////////////////////////

        std::chrono::duration<double> time_used_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        std::chrono::duration<double> time_used_2 = std::chrono::duration_cast<std::chrono::duration<double>>(t3-t2);
        std::chrono::duration<double> time_used_3 = std::chrono::duration_cast<std::chrono::duration<double>>(t4-t3);

        std::cout<<"time_used 1 : "<<time_used_1.count()*1000<<"ms"<<std::endl;
        std::cout<<"time_used 2 : "<<time_used_2.count()*1000<<"ms"<<std::endl;
        std::cout<<"time_used 3 : "<<time_used_3.count()*1000<<"ms"<<std::endl;

        //////////////////////////  show result  //////////////////////////

        cv::imshow("desc", desc);
        cv::imshow("score", score);
        cv::imshow("LetNet Infer Result", frame);
        cv::waitKey(1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "letnet");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/infra1/image_rect_raw", 10, imageCallback);


    ros::param::get("~model_param_path", model_param_path);
    ros::param::get("~model_path", model_path);

    net.opt.num_threads=1;
    model_param_path = package_path+"/model/"+model_param_path;
    model_path = package_path+"/model/"+model_path;

    net.load_param(model_param_path.c_str());
    net.load_model(model_path.c_str());

    ros::spin();
    cv::destroyWindow("LetNet Infer Result");

    return 0;
}
