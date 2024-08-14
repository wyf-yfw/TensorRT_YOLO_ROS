//
// Created by wyf on 24-8-2.
//

#ifndef CAMERA_INFER_H
#define CAMERA_INFER_H
#include "infer.h"
#include <utility>
#include "camera_infer.h"
#include "config.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <infer_result.h>
#include <results.h>

class DCameraInfer:public YoloDetector{
public:
    DCameraInfer(ros::NodeHandle& nh);
    int run(cv::Mat& img);
    void draw_image(cv::Mat& img, tensorrt_yolo::infer_result& inferResult);
    void image_callback(const sensor_msgs::ImageConstPtr& rbg_msg, const sensor_msgs::ImageConstPtr& depth_msg);

protected:
    image_transport::Subscriber image_sub_;
    int frame_count_;
    std::chrono::high_resolution_clock::time_point start_time_;
    ros::NodeHandle& nh_;
    std::vector<Detection> result_;
    tensorrt_yolo::results results_msg_;
    ros::Publisher results_pub_ = nh_.advertise<tensorrt_yolo::results>("infer_results", 10);
};

#endif //CAMERA_INFER_H
