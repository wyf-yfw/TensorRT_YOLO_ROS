//
// Created by wyf on 24-8-2.
//

//#ifndef CAMERA_INFER_H
//#define CAMERA_INFER_H
#include <ros/ros.h>
#include "utils.h"
#include "infer.h"
#include <utility>
#include "config.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <InferResult.h>
#include <Results.h>
#include "byte_tracker.h" //bytetrack
#include <chrono>


class CameraInfer:public YoloDetector, public BYTEtracker{
public:
    CameraInfer(ros::NodeHandle& nh);
    void draw_image(cv::Mat& img);
    void draw_image(cv::Mat& img, cv::Mat& depth_img);
    void bytetrack();
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void image_callback(const sensor_msgs::ImageConstPtr& rbg_msg, const sensor_msgs::ImageConstPtr& depth_msg);
protected:
    image_transport::Subscriber image_sub_;
    // topic的名字
    std::string rgbImageTopic_;
    std::string depthImageTopic_;

    int frame_count_;
    std::chrono::high_resolution_clock::time_point start_time_;
    image_transport::ImageTransport it_;
    tensorrt_yolo::Results results_msg_;
    ros::Publisher results_pub_ = nh_.advertise<tensorrt_yolo::Results>("infer_results", 10);
    std::vector<strack> output_stracks_;
    // 追踪开关，默认为false
    bool track_;
    bool depth_;
    bool pose_;
};
//#endif //CAMERA_INFER_H
