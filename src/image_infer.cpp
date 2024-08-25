//
// Created by wyf on 24-8-1.
//
#include "utils.h"
#include "infer.h"
#include <utility>
#include "image_infer.h"
#include "config.h"

ImageInfer::ImageInfer(ros::NodeHandle& nh):save_(true),imageDir_(imageDir),YoloDetector(nh){
    if (read_files_in_dir(imageDir_, file_names_) < 0) {
        std::cout << "read_files_in_dir failed." << std::endl;
    }

}

int ImageInfer::run(){
    // 推理
    for (long unsigned int i = 0; i < file_names_.size(); i++){
        imagePath_ = std::string(imageDir_) + "/" + file_names_[i];
        cv::Mat img = cv::imread(imagePath_, cv::IMREAD_COLOR);
        if (img.empty()) continue;

        auto start = std::chrono::system_clock::now();

        tensorrt_yolo::Results res = inference(img);
        for(int i = 0; i < res.results.size();i++) {
            draw_image(img, res.results[i]);
        }
        auto end = std::chrono::system_clock::now();
        int cost = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Image: " << file_names_[i] << " cost: " << cost << " ms."  << std::endl;
        if(save_) {
            SaveResult(img, i);
        }
    }

    return 0;
}
void ImageInfer::SaveResult(const cv::Mat& img, int num) {

    cv::imwrite("_" + file_names_[num], img);

}
void ImageInfer::draw_image(cv::Mat& img, tensorrt_yolo::InferResult inferResult){
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
    ros::init(argc, argv, "image_infer_node");
    ros::NodeHandle nh;
    ImageInfer i(nh);
    i.run();
    return 0;
}
