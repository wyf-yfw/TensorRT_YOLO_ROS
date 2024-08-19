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

class DCameraInfer:public YoloDetector, public BYTETracker{
public:
    // 构造函数
    DCameraInfer(ros::NodeHandle& nh, bool track);
    // 目标检测
    int detect(cv::Mat& img, cv::Mat& depth_img);
    // 追踪
    void bytetrack(cv::Mat& img, cv::Mat& depth_img);
    // 回调函数
    void image_callback(const sensor_msgs::ImageConstPtr& rbg_msg, const sensor_msgs::ImageConstPtr& depth_msg);
protected:
    // 图像subscriber
    image_transport::Subscriber image_sub_;
    // 帧计数
    int frame_count_;
    // 起始时间
    std::chrono::high_resolution_clock::time_point start_time_;
    ros::NodeHandle& nh_;
    // 用于publish的results
    tensorrt_yolo::results results_msg_;
    // 目标检测的publisher
    ros::Publisher detect_results_pub_ = nh_.advertise<tensorrt_yolo::results>("detect_results", 10);
    // 目标追踪的publisher
    ros::Publisher track_results_pub_ = nh_.advertise<tensorrt_yolo::results>("track_results", 10);
    // 追踪开关，默认为false
    bool track_;
};

#endif //CAMERA_INFER_H
