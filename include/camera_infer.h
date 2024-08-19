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
#include "BYTETracker.h"


class CameraInfer:public YoloDetector, public BYTETracker{
public:
    CameraInfer(ros::NodeHandle& nh,  bool track);
    int detect(cv::Mat& img);
    void bytetrack(cv::Mat& img);
    void cameraInferCallback(const sensor_msgs::ImageConstPtr& msg);
protected:
    image_transport::Subscriber image_sub_;
    int frame_count_;
    std::chrono::high_resolution_clock::time_point start_time_;
    image_transport::ImageTransport it_;
    ros::NodeHandle& nh_;
    tensorrt_yolo::results results_msg_;
    ros::Publisher detect_results_pub_ = nh_.advertise<tensorrt_yolo::results>("detect_results", 10);
    ros::Publisher track_results_pub_ = nh_.advertise<tensorrt_yolo::results>("track_results", 10);
    // 追踪开关，默认为false
    bool track_;

};
#endif //CAMERA_INFER_H
