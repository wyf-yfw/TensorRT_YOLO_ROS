//
// Created by wyf on 24-8-1.
//
#include "utils.h"
#include "infer.h"
#include <utility>
#include "d_camera_infer.h"
#include "config.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "/home/wyf/catkin_ws/devel/include/tensorrt_yolo/Pixel3dPoint.h"
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
        
DCameraInfer::DCameraInfer(ros::NodeHandle& nh):
frame_count_(0),
start_time_(std::chrono::high_resolution_clock::now()),
nh_(nh),
YoloDetector()
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> slamSyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image>* rimg_sub_ ;             // rgb图像输入
    message_filters::Subscriber<sensor_msgs::Image>* dimg_sub_;              // 深度图像输入
    message_filters::Synchronizer<slamSyncPolicy>* sync_;
    rimg_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, rgbImageTopic, 1);
    dimg_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, depthImageTopic, 1);

    sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(10), *rimg_sub_, *dimg_sub_);
    sync_->registerCallback(boost::bind(&DCameraInfer::image_callback, this, _1, _2));
}
int DCameraInfer::run(cv::Mat& img){
    // 推理
    if (img.empty()) return -1;
    auto start = std::chrono::system_clock::now();

    result_ = inference(img);
    auto end = std::chrono::system_clock::now();
    int cost = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // ROS_INFO("cost: %d ms.", cost);
    // std::cout << "Image: " << file_names[i] << " done." << std::endl

    return 0;
}
void DCameraInfer::image_callback(const sensor_msgs::ImageConstPtr& rbg_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    // 使用 cv_bridge 转换 ROS 图像消息到 OpenCV 图像
    std::string Fps = "FPS:";
    // cv::Mat img = cv_bridge::toCvShare(rbg_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    cv::Mat img = cv_bridge::toCvShare(rbg_msg, sensor_msgs::image_encodings::BGR8)->image;
    //推理图像
    run(img);
    for(int i = 0;i<result_.size();i++) {
        int pixel_x = round((result_[i].bbox[0] + result_[i].bbox[2]) / 2.0);
        int pixel_y = round((result_[i].bbox[1] + result_[i].bbox[3]) / 2.0);
        if (pixel_x < 0 || pixel_y < 0)
        {
            ROS_WARN("Pixel coordinates not set. Skipping depth image processing.");
            return;
        }

        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_image = cv_ptr->image;

            if (pixel_x >= depth_image.cols || pixel_y >= depth_image.rows)
            {
                ROS_WARN("Pixel coordinates out of bounds.");
                return;
            }

            uint16_t depth_value = depth_image.at<uint16_t>(pixel_y, pixel_x); // 深度值是 16 位无符号整数（单位：毫米）

            double depth_m = depth_value / 10.0; // 转换为cm

            double fx = K[0];
            double fy = K[4];
            double cx = K[2];
            double cy = K[5];

            double X = (pixel_x - cx) * depth_m / fx;
            double Y = -(pixel_y - cy) * depth_m / fy;
            double Z = depth_m;
            result_[i].coordinate[0] = X;
            result_[i].coordinate[1] = Y;
            result_[i].coordinate[2] = Z;
            // ROS_INFO("Depth at pixel (%d, %d): %f mm", pixel_x, pixel_y, depth_value);
            // ROS_INFO("Published 3D coordinates: X = %f m, Y = %f m, Z = %f m", X, Y, Z);
            draw_image(img,result_[i]);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    cv::Point textOrg(50, 50);

    // 设置字体类型、大小、颜色等
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 1;
    int thickness = 2;
    cv::Scalar color(0, 255, 0); // 绿色

    // 更新帧计数
    ++frame_count_;

    // 计算并输出帧率
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = now - start_time_;
    if (elapsed.count() >= 0.001)
    {
        int fps = frame_count_ / elapsed.count();
        // ROS_INFO("FPS: %d", fps);
        std::string f = Fps + std::to_string(fps);
        cv::putText(img, f, textOrg, fontFace, fontScale, color, thickness);
        // 重置计数器和时间
        frame_count_ = 0;
        start_time_ = now;
    }
    // 显示图像
    cv::imshow("img_viewer", img);
    cv::waitKey(30);
}



void DCameraInfer::draw_image(cv::Mat& img, Detection& inferResult){
    // 在图像上绘制检测结果

    cv::Scalar bboxColor((inferResult.classId * 30 + 123) % 255, (inferResult.classId * 20 + 78) % 255 , (inferResult.classId + 478) % 255); // 随机颜色

    cv::Rect r(
        round(inferResult.bbox[0]),
        round(inferResult.bbox[1]),
        round(inferResult.bbox[2] - inferResult.bbox[0]),
        round(inferResult.bbox[3] - inferResult.bbox[1])
    );

    cv::Point p(
        round((inferResult.bbox[0] + inferResult.bbox[2]) / 2.0),
        round((inferResult.bbox[1] + inferResult.bbox[3]) / 2.0)
    );

    cv::circle(img, p, 2, bboxColor, -1); // 绘制中心点
    std::string text = "(" + std::to_string(inferResult.coordinate[0]) + ", " + std::to_string(inferResult.coordinate[1]) + ", " + std::to_string(inferResult.coordinate[2]) + ")";// 构造坐标文本

    // 设置文本参数
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 1;

    std::string className = vClassNames[(int)inferResult.classId]; // 获取类别名称
    std::string labelStr = className + " " + std::to_string(inferResult.conf).substr(0, 4); // 创建标签字符串

    cv::Size textSize = cv::getTextSize(labelStr, cv::FONT_HERSHEY_PLAIN, 1.2, 2, NULL); // 获取文本大小
    cv::Point textOrg(p.x + 5, p.y + textSize.height / 2);// 文本位置在点的右边
    cv::putText(img, text, textOrg, fontFace, fontScale, bboxColor, thickness, cv::LINE_AA);// 绘制中心点坐标

    cv::rectangle(img, r, bboxColor, 2); // 绘制边界框
    cv::Point topLeft(r.x, r.y - textSize.height - 3); // 标签位置
    cv::Point bottomRight(r.x + textSize.width, r.y); // 标签背景位置
    cv::rectangle(img, topLeft, bottomRight, bboxColor, -1); // 绘制标签背景
    cv::putText(img, labelStr, cv::Point(r.x, r.y - 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA); // 绘制标签文本

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "d_camera_infer_node");
    ros::NodeHandle nh;
    DCameraInfer c(nh);
    ros::spin();
}