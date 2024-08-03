//
// Created by wyf on 24-8-1.
//

#ifndef IMAGE_INFER_H
#define IMAGE_INFER_H

#include "utils.h"
#include "infer.h"
#include <unistd.h>

class ImageInfer : public YoloDetector{
public:
    ImageInfer(ros::NodeHandle& nh);
    void SaveResult(const cv::Mat& img, int num);
    int run();
    void draw_image(cv::Mat& img, Detection& inferResult);
private:
    const char* imageDir_;
    std::string imagePath_;
    std::vector<std::string> file_names_;
    bool save_;
};
#endif //IMAGE_INFER_H
