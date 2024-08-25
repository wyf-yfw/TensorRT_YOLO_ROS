//
// Created by wyf on 24-8-2.
//
#include "image_infer.h"
#include <string>
#include <vector>

const char* imageDir = "/home/wyf/catkin_ws/src/TensorRT_YOLO_ROS/images"; // 图片文件夹


const std::vector<int> track_classes = {0};
const int kGpuId = 0; //显卡id，一张显卡默认为0


// image的高和宽
const int kInputH = 640;
const int kInputW = 640;
const int kMaxNumOutputBbox = 1000;  // assume the box outputs no more than kMaxNumOutputBbox boxes that conf >= kNmsThresh;

// for FP16 mode
const bool bFP16Mode = false;

// 目前无法进行INT8量化
// for INT8 mode
const bool bINT8Mode = true;
const std::string cacheFile = "./int8.cache";
const std::string calibrationDataPath = "../calibrator";  // 存放用于 int8 量化校准的图像

//模型种类名称
const std::vector<std::string> vClassNames {
    "person"
};
// 相机内参
const double K[9] = {383.6372985839844, 0.0, 316.88177490234375,
                      0.0, 383.6372985839844, 241.00013732910156,
                      0.0, 0.0, 1.0};
std::vector<double> D = {0.0, 0.0, 0.0, 0.0};
// 模型骨架
const std::vector<std::vector<int>> skeleton {
        {16, 14},
        {14, 12},
        {17, 15},
        {15, 13},
        {12, 13},
        {6, 12},
        {7, 13},
        {6, 7},
        {6, 8},
        {7, 9},
        {8, 10},
        {9, 11},
        {2, 3},
        {1, 2},
        {1, 3},
        {2, 4},
        {3, 5},
        {4, 6},
        {5, 7}
};