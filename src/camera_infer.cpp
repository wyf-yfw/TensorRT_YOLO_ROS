//
// Created by wyf on 24-8-1.
//
#include "utils.h"
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

CameraInfer::CameraInfer(ros::NodeHandle& nh):
it_(nh),
frame_count_(0),
start_time_(std::chrono::high_resolution_clock::now()),
nh_(nh),
YoloDetector()
{
    //订阅图像话题
    image_sub_ = it_.subscribe(rgbImageTopic, 1, &CameraInfer::cameraInferCallback, this);
}
int CameraInfer::run(cv::Mat& img){
    // 推理
    if (img.empty()) return -1;
    auto start = std::chrono::system_clock::now();

    results_msg_ = inference(img);
    results_pub_.publish(results_msg_);
    for (size_t i = 0; i < result_.size(); ++i) {
        Detection& det = result_[i];
        draw_image(img, det);
    }
    auto end = std::chrono::system_clock::now();
    int cost = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // ROS_INFO("cost: %d ms.", cost);
    // std::cout << "Image: " << file_names[i] << " done." << std::endl

    return 0;
}

void CameraInfer::cameraInferCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 使用 cv_bridge 转换 ROS 图像消息到 OpenCV 图像
    std::string Fps = "FPS:";
    cv::Mat img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    //推理图像
    run(img);
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
void CameraInfer::draw_image(cv::Mat& img, Detection& inferResult){
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
    std::string text = "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";// 构造坐标文本

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
    ros::init(argc, argv, "camera_infer_node");
    ros::NodeHandle nh;
    CameraInfer c(nh);
    ros::spin();
}