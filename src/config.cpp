//
// Created by wyf on 24-8-2.
//
#include "image_infer.h"
#include <string>
#include <vector>

const char* imageDir = "/home/wyf/catkin_ws/src/tensorrt_yolo/images";
const std::string trtFile = "/home/wyf/catkin_ws/src/tensorrt_yolo/onnx_model/yolov8s.plan";
const std::string onnxFile = "/home/wyf/catkin_ws/src/tensorrt_yolo/onnx_model/yolov8s.onnx";
const std::string rgbImageTopic = "camera/color/image_raw";
const std::string depthImageTopic = "/camera/depth/image_rect_raw";
const int kGpuId = 0;
const int kNumClass = 80;
const int kInputH = 640;
const int kInputW = 640;
const float kNmsThresh = 0.7f;
const float kConfThresh = 0.6f;
const int kMaxNumOutputBbox = 1000;  // assume the box outputs no more than kMaxNumOutputBbox boxes that conf >= kNmsThresh;
const int kNumBoxElement = 7;  // left, top, right, bottom, confidence, class, keepflag(whether drop when NMS)

// for FP16 mode
const bool bFP16Mode = true;
// for INT8 mode
const bool bINT8Mode = false;
const std::string cacheFile = "./int8.cache";
const std::string calibrationDataPath = "../calibrator";  // 存放用于 int8 量化校准的图像

const std::vector<std::string> vClassNames {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};
const double K[9] = {383.6372985839844, 0.0, 316.88177490234375,
                              0.0, 383.6372985839844, 241.00013732910156,
                              0.0, 0.0, 1.0};
std::vector<double> D = {0.0, 0.0, 0.0, 0.0};