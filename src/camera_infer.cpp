//
// Created by wyf on 24-8-1.
//
#include "camera_infer.h"
#include "utils.h"
#include "infer.h"
#include <utility>
#include "config.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <infer_result.h>
#include <results.h>
#include "BYTETracker.h" //bytetrack


CameraInfer::CameraInfer(ros::NodeHandle& nh, bool track):
it_(nh),
frame_count_(0),
start_time_(std::chrono::high_resolution_clock::now()),
nh_(nh),
track_(track),
YoloDetector()
{
    //订阅图像话题
    image_sub_ = it_.subscribe(rgbImageTopic, 1, &CameraInfer::cameraInferCallback, this);
}
int CameraInfer::detect(cv::Mat& img){
    // 绘制图像
    draw_fps(img, frame_count_, start_time_);
    draw_detection_box(img,results_msg_);
    calculate_print_center_point(img, results_msg_);
    // publish目标检测的results
    detect_results_pub_.publish(results_msg_);
    // 显示图像
    cv::imshow("img_viewer", img);
    cv::waitKey(30);
    return 0;
}
void CameraInfer::bytetrack(cv::Mat& img)
{
    // 需要跟踪的目标类型
    std::vector<detect_result> objects;
    // 用于存储结果的转换
    std::vector<detect_result> results;
    // result格式转换
    yolo_detece2detect_result(results_msg_, results);
    // 判断需要跟踪的目标类型
    for (detect_result dr : results) {
        for (int tc: track_classes) {
            if (dr.classId == tc)
            {
                objects.push_back(dr);
            }
        }
    }
    // 目标跟踪
    std::vector<STrack> output_stracks = update(objects);
    // 清除用于原始的results_msg_
    results_msg_.results.clear();
    // 绘制图像
    draw_tracking_box(img, output_stracks, results_msg_);
    calculate_print_center_point(img, results_msg_);
    draw_fps(img, frame_count_, start_time_);
    draw_detection_box(img,results_msg_);
    // publish追踪的results
    track_results_pub_.publish(results_msg_);
    cv::imshow("track", img);
    cv::waitKey(30);
}
void CameraInfer::cameraInferCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 使用 cv_bridge 转换 ROS 图像消息到 OpenCV 图像
    cv::Mat img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    // 推理
    if (img.empty()) return;
    results_msg_ = inference(img);
    if (track_) {
        bytetrack(img);
    } else {
        // 目标检测
        detect(img);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_infer_node");
    ros::NodeHandle nh;
    bool track = false;
    nh.getParam("/yolo_node/track", track);
    CameraInfer c(nh, track);
    ros::spin();
}