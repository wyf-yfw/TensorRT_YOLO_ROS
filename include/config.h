#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>
#include <boost/array.hpp>
extern const char* imageDir;
extern const std::string trtFile;
extern const std::string onnxFile;
extern const std::string rgbImageTopic;
extern const std::string depthImageTopic;
extern const std::vector<int> track_classes;

extern const int kGpuId;
extern const int kNumClass ;
extern const int kInputH ;
extern const int kInputW ;
extern const float kNmsThresh ;
extern const float kConfThresh ;
extern const int kMaxNumOutputBbox;  // assume the box outputs no more than kMaxNumOutputBbox boxes that conf >= kNmsThresh;
extern const int kNumBoxElement;  // left, top, right, bottom, confidence, class, keepflag(whether drop when NMS)


// const std::string testDataDir = "../images";  // 用于推理

// for FP16 mode
extern const bool bFP16Mode;
// for INT8 mode
extern const bool bINT8Mode;
extern const std::string cacheFile;
extern const std::string calibrationDataPath ;  // 存放用于 int8 量化校准的图像

extern const std::vector<std::string> vClassNames;
extern const double K[9];
extern std::vector<double> D;

#endif  // CONFIG_H
