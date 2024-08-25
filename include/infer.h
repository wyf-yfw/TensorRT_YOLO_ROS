#ifndef INFER_H
#define INFER_H

#include <opencv2/opencv.hpp>
#include "public.h"
#include "config.h"
#include "types.h"
#include <ros/ros.h>
#include <InferResult.h>
#include <Results.h>

using namespace nvinfer1;



class YoloDetector
{
public:
    YoloDetector(
    );
    ~YoloDetector();
    tensorrt_yolo::Results inference(cv::Mat& img);
private:
    void get_engine();

private:
    Logger              gLogger;
    std::string         trtFile_;

    int                 numClass_;
    float               nmsThresh_;
    float               confThresh_;

    ICudaEngine *       engine;
    IRuntime *          runtime;
    IExecutionContext * context;

    cudaStream_t        stream;

    float *             outputData;
    std::vector<void *> vBufferD;
    float *             transposeDevice;
    float *             decodeDevice;

    int                 OUTPUT_CANDIDATES;  // 8400: 80 * 80 + 40 * 40 + 20 * 20
    cv::Mat*            img_;

};

#endif  // INFER_H
