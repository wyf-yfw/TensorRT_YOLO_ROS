#ifndef UTILS_H
#define UTILS_H

#include <dirent.h>
#include <random>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "byte_tracker.h"
#include "config.h"
#include <ros/ros.h>
#include <InferResult.h>
#include <KeyPoint.h>
#include <Results.h>
#include <iomanip>
#include "types.h"
// 读取文件
static inline int read_files_in_dir(const char* p_dir_name, std::vector<std::string>& file_names)
{
    DIR *p_dir = opendir(p_dir_name);
    if (p_dir == nullptr) {
        return -1;
    }

    struct dirent* p_file = nullptr;
    while ((p_file = readdir(p_dir)) != nullptr) {
        if (strcmp(p_file->d_name, ".") != 0 &&
            strcmp(p_file->d_name, "..") != 0) {
            //std::string cur_file_name(p_dir_name);
            //cur_file_name += "/";
            //cur_file_name += p_file->d_name;
            std::string cur_file_name(p_file->d_name);
            file_names.push_back(cur_file_name);
        }
    }

    closedir(p_dir);
    return 0;
}
// 0-255随机整数
static inline int get_random_int(int minThres=0, int maxThres=255){
    // 获取处于某一范围内的一个随机整数
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(minThres, maxThres);

    int random_integer = distrib(gen);

    return random_integer;
}
//infer_result格式转化为追踪下的detect_result格式
inline void yolo_detece2detect_result(tensorrt_yolo::Results& results_msg_, std::vector<detect_result>& objects){
    for(int i = 0; i < results_msg_.results.size();i++){
        detect_result det;
        det.classId = results_msg_.results[i].classId;
        det.conf = results_msg_.results[i].conf;
        cv::Rect rect(results_msg_.results[i].bbox[0],
                      results_msg_.results[i].bbox[1],
                      (results_msg_.results[i].bbox[2]-results_msg_.results[i].bbox[0]),
                      (results_msg_.results[i].bbox[3] - results_msg_.results[i].bbox[1])
        );
        det.box = rect;
        det.kpts = results_msg_.results[i].kpts;
        det.vKpts = results_msg_.results[i].vKpts;
        objects.push_back(det);
    }
}
// 绘制追踪边框
inline void draw_tracking_id(cv::Mat& img, std::vector<strack>& output_stracks, tensorrt_yolo::Results& results_msg_){
    for (unsigned long i = 0; i < output_stracks.size(); i++)
    {
        tensorrt_yolo::InferResult inf;
        for(int j = 0;j < 4; j ++){
            inf.bbox[j] = output_stracks[i].tlbr[j];
        }
        inf.conf = output_stracks[i].score;
        inf.classId = output_stracks[i].classId_;
        inf.Id = output_stracks[i].track_id;
        inf.kpts = output_stracks[i].kpts_;
        inf.vKpts = output_stracks[i].vKpts_;
        results_msg_.results.push_back(inf);
        std::vector<float> tlwh = output_stracks[i].tlwh;

        bool vertical = tlwh[2] / tlwh[3] > 1.6;
        if (tlwh[2] * tlwh[3] > 20 && !vertical)
        {
            cv::Scalar s((inf.classId * 30 + 123) % 255, (inf.classId * 20 + 78) % 255 , (inf.classId + 478) % 255); // 随机颜色

            cv::putText(img, cv::format("ID: %d", output_stracks[i].track_id), cv::Point(tlwh[0], tlwh[1] + 20),
                        0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
//            cv::rectangle(img, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
        }
    }

}
// 绘制帧率
inline void draw_fps(cv::Mat& img, int& frame_count_, std::chrono::high_resolution_clock::time_point& start_time_){
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
        std::string f = "Fps" + std::to_string(fps);
        cv::putText(img, f, textOrg, fontFace, fontScale, color, thickness);
        // 重置计数器和时间
        frame_count_ = 0;
        start_time_ = now;
    }
}
// 绘制目标检测边框
inline void draw_detection_box(cv::Mat& img, tensorrt_yolo::Results& results_msgs_){
    for(tensorrt_yolo::InferResult inferResult : results_msgs_.results) {
        // 在图像上绘制检测结果

        cv::Scalar bboxColor((inferResult.classId * 30 + 123) % 255, (inferResult.classId * 20 + 78) % 255,
                             (inferResult.classId + 478) % 255); // 随机颜色

        cv::Rect r(
                round(inferResult.bbox[0]),
                round(inferResult.bbox[1]),
                round(inferResult.bbox[2] - inferResult.bbox[0]),
                round(inferResult.bbox[3] - inferResult.bbox[1])
        );

        std::string className = vClassNames[(int) inferResult.classId]; // 获取类别名称
        std::string labelStr = className + " " + std::to_string(inferResult.conf).substr(0, 4); // 创建标签字符串

        cv::Size textSize = cv::getTextSize(labelStr, cv::FONT_HERSHEY_PLAIN, 1.2, 2, NULL); // 获取文本大小
        cv::rectangle(img, r, bboxColor, 2); // 绘制边界框
        cv::Point topLeft(r.x, r.y - textSize.height - 3); // 标签位置
        cv::Point bottomRight(r.x + textSize.width, r.y); // 标签背景位置
        cv::rectangle(img, topLeft, bottomRight, bboxColor, -1); // 绘制标签背景
        cv::putText(img, labelStr, cv::Point(r.x, r.y - 2), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 255, 255), 2,
                    cv::LINE_AA); // 绘制标签文本
    }
}
// 计算并绘制相机坐标中心点坐标
inline void calculate_draw_center_point(cv::Mat& img, cv::Mat& depth_img, tensorrt_yolo::Results& results_msg_){
    std::stringstream stream;
    for(int i = 0;i<results_msg_.results.size();i++) {
        cv::Scalar bboxColor((results_msg_.results[i].classId * 30 + 123) % 255, (results_msg_.results[i].classId * 20 + 78) % 255,
                             (results_msg_.results[i].classId + 478) % 255); // 随机颜色
        int pixel_x = round((results_msg_.results[i].bbox[0] + results_msg_.results[i].bbox[2]) / 2.0);
        int pixel_y = round((results_msg_.results[i].bbox[1] + results_msg_.results[i].bbox[3]) / 2.0);
        if (pixel_x < 0 || pixel_y < 0)
        {
            ROS_WARN("Pixel coordinates not set. Skipping depth image processing.");
            return;
        }

        try
        {
            if (pixel_x >= depth_img.cols || pixel_y >= depth_img.rows)
            {
                ROS_WARN("Pixel coordinates out of bounds.");
                return;
            }

            uint16_t depth_value = depth_img.at<uint16_t>(pixel_y, pixel_x); // 深度值是 16 位无符号整数（单位：毫米）

            double depth_m = depth_value / 10.0; // 转换为cm

            double fx = K[0];
            double fy = K[4];
            double cx = K[2];
            double cy = K[5];

            double X = (pixel_x - cx) * depth_m / fx;
            double Y = -(pixel_y - cy) * depth_m / fy;
            double Z = depth_m;
            results_msg_.results[i].coordinate[0] = X;
            results_msg_.results[i].coordinate[1] = Y;
            results_msg_.results[i].coordinate[2] = Z;

            // 设置文本参数
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 0.5;
            int thickness = 1;

            cv::Point p(
                    round(pixel_x),
                    round(pixel_y)
            );
            stream << std::fixed << std::setprecision(2)
                   << "(" << results_msg_.results[i].coordinate[0] << ", "
                   << results_msg_.results[i].coordinate[1] << ", "
                   << results_msg_.results[i].coordinate[2] << ")";

            // 构造坐标文本
            std::string text = stream.str();
            cv::circle(img, p, 2, bboxColor, -1); // 绘制中心点
            cv::Point textOrg(p.x + 5, p.y + 5);// 文本位置在点的右边
            cv::putText(img, text, textOrg, fontFace, fontScale, bboxColor, thickness, cv::LINE_AA);// 绘制中心点坐标

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
}
// 计算并绘制像素坐标中心点
inline void calculate_draw_center_point(cv::Mat& img, tensorrt_yolo::Results& results_msg_) {
    for(int i = 0;i<results_msg_.results.size();i++) {
        cv::Scalar bboxColor((results_msg_.results[i].classId * 30 + 123) % 255, (results_msg_.results[i].classId * 20 + 78) % 255,
                             (results_msg_.results[i].classId + 478) % 255); // 随机颜色
        int pixel_x = round((results_msg_.results[i].bbox[0] + results_msg_.results[i].bbox[2]) / 2.0);
        int pixel_y = round((results_msg_.results[i].bbox[1] + results_msg_.results[i].bbox[3]) / 2.0);
        if (pixel_x < 0 || pixel_y < 0)
        {
            ROS_WARN("Pixel coordinates not set. Skipping depth image processing.");
            return;
        }

        try
        {

            results_msg_.results[i].coordinate[0] = pixel_x;
            results_msg_.results[i].coordinate[1] = pixel_y;
            results_msg_.results[i].coordinate[2] = 0;

            // 设置文本参数
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 0.5;
            int thickness = 1;

            cv::Point p(
                    round(pixel_x),
                    round(pixel_y)
            );
            std::string text =
                    "(" + std::to_string(static_cast<int>(results_msg_.results[i].coordinate[0])) + ", "
                    + std::to_string(static_cast<int>(results_msg_.results[i].coordinate[1])) + ")";// 构造坐标文本

            cv::circle(img, p, 2, bboxColor, -1); // 绘制中心点
            cv::Point textOrg(p.x + 5, p.y + 5);// 文本位置在点的右边
            cv::putText(img, text, textOrg, fontFace, fontScale, bboxColor, thickness, cv::LINE_AA);// 绘制中心点坐标

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
}
inline void draw_pose_points(cv::Mat& img, tensorrt_yolo::Results results_msg_){
    // draw inference result on image
    for (size_t j = 0; j < results_msg_.results.size(); j++)
    {
        int x, y;
        float conf;
        int radius = std::min(img.rows, img.cols) / 100;
        cv::Scalar kptColor(get_random_int(), get_random_int(), get_random_int());

        std::vector<tensorrt_yolo::KeyPoint> vScaledKpts = results_msg_.results[j].vKpts;

        for (size_t k = 0; k < vScaledKpts.size(); k++){
            x = (int)vScaledKpts[k].x;
            y = (int)vScaledKpts[k].y;
            conf = vScaledKpts[k].visible;
            if (x < 0 || x > img.cols || y < 0 || y > img.rows) continue;
            if (conf < 0.5) continue;
            cv::circle(img, cv::Point(x, y), radius, kptColor, -1);
        }
        // draw skeleton between key points
        int kpt1_idx, kpt2_idx, kpt1_x, kpt1_y, kpt2_x, kpt2_y;
        float kpt1_conf, kpt2_conf;
        int skeleton_width = std::min(img.rows, img.cols) / 300;
        cv::Scalar skeletonColor((results_msg_.results[j].classId * 30 + 123) % 255, (results_msg_.results[j].classId * 20 + 78) % 255,
                             (results_msg_.results[j].classId + 478) % 255); // 随机颜色

        for (size_t m = 0; m < skeleton.size(); m++){
            kpt1_idx = skeleton[m][0] - 1;
            kpt2_idx = skeleton[m][1] - 1;

            kpt1_x = (int)vScaledKpts[kpt1_idx].x;

            kpt1_y = (int)vScaledKpts[kpt1_idx].y;

            kpt1_conf = vScaledKpts[kpt1_idx].visible;

            kpt2_x = (int)vScaledKpts[kpt2_idx].x;
            kpt2_y = (int)vScaledKpts[kpt2_idx].y;

            kpt2_conf = vScaledKpts[kpt2_idx].visible;

            if (kpt1_conf < 0.5 || kpt2_conf < 0.5) continue;
            if (kpt1_x > img.cols || kpt1_y > img.rows || kpt1_x < 0 || kpt1_y < 0) continue;
            if (kpt2_x > img.cols || kpt2_y > img.rows || kpt2_x < 0 || kpt2_y < 0) continue;
            cv::line(img, cv::Point(kpt1_x, kpt1_y), cv::Point(kpt2_x, kpt2_y), skeletonColor, skeleton_width, cv::LINE_AA);
        }

    }
}

#endif  // UTILS_H
