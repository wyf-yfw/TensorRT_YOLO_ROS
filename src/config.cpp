//
// Created by wyf on 24-8-2.
//
#include "image_infer.h"
#include <string>
#include <vector>

const char* imageDir = "/home/wyf/catkin_ws/src/tensorrt_yolo/images"; // 图片文件夹
const std::string trtFile = "/home/wyf/catkin_ws/src/TensorRT_YOLO_ROS/onnx_model/yolov8s.plan"; // .plan文件地址，如果不存在将在此处创建.plan文件
const std::string onnxFile = "/home/wyf/catkin_ws/src/TensorRT_YOLO_ROS/onnx_model/yolov8s.onnx"; //.onnx模型地址
const std::string rgbImageTopic = "camera/color/image_raw"; //ros中image的topic
const std::string depthImageTopic = "/camera/depth/image_rect_raw"; //ros中depth image的topic
const std::vector<int> track_classes = {0};
const int kGpuId = 0; //显卡id，一张显卡默认为0
const int kNumClass = 80; //模型中推理种类的数量
// image的高和宽
const int kInputH = 640;
const int kInputW = 640;
const float kNmsThresh = 0.7f;
const float kConfThresh = 0.6f; //推理置信区间
const int kMaxNumOutputBbox = 1000;  // assume the box outputs no more than kMaxNumOutputBbox boxes that conf >= kNmsThresh;
const int kNumBoxElement = 7;  // left, top, right, bottom, confidence, class, keepflag(whether drop when NMS)

// for FP16 mode
const bool bFP16Mode = true;

// 目前无法进行INT8量化
// for INT8 mode
const bool bINT8Mode = false;
const std::string cacheFile = "./int8.cache";
const std::string calibrationDataPath = "../calibrator";  // 存放用于 int8 量化校准的图像

//模型种类名称
const std::vector<std::string> vClassNames {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
    "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
    "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};
// 相机内参
const double K[9] = {383.6372985839844, 0.0, 316.88177490234375,
                      0.0, 383.6372985839844, 241.00013732910156,
                      0.0, 0.0, 1.0};
std::vector<double> D = {0.0, 0.0, 0.0, 0.0};