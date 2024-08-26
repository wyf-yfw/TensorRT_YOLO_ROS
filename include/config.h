#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>
#include <boost/array.hpp>
extern const char* imageDir;
extern const char* kInputTensorName;
extern const char* kOutputTensorName;

extern const std::vector<int> track_classes;

extern const int kGpuId;

extern const int kInputH ;
extern const int kInputW ;

extern const int kMaxNumOutputBbox;  // assume the box outputs no more than kMaxNumOutputBbox boxes that conf >= kNmsThresh;


// for FP16 mode
extern const bool bFP16Mode;
// for INT8 mode
extern const bool bINT8Mode;
extern const std::string cacheFile;
extern const std::string calibrationDataPath ;  // 存放用于 int8 量化校准的图像

extern const std::vector<std::string> vClassNames;
extern const double K[9];
extern std::vector<double> D;
extern const std::vector<std::vector<int>> skeleton;
#endif  // CONFIG_H
