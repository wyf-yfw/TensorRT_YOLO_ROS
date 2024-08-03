#include <iostream>
#include <fstream>

#include <NvOnnxParser.h>

#include "infer.h"
#include "preprocess.h"
#include "postprocess.h"
#include "calibrator.h"
#include "utils.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

using namespace nvinfer1;

YoloDetector::YoloDetector(
        int gpuId,                // GPU ID
        float nmsThresh,          // 非极大值抑制 (NMS) 阈值
        float confThresh,         // 置信度阈值
        int numClass             // 类别数量
    ): trtFile_(trtFile), nmsThresh_(nmsThresh), confThresh_(confThresh), numClass_(numClass), img_(nullptr)
{
    gLogger = Logger(ILogger::Severity::kERROR); // 设置日志记录器
    cudaSetDevice(gpuId); // 设置当前 GPU

    CHECK(cudaStreamCreate(&stream)); // 创建 CUDA 流

    // 加载 TensorRT 引擎
    get_engine();

    context = engine->createExecutionContext(); // 创建推理上下文
    context->setBindingDimensions(0, Dims32 {4, {1, 3, kInputH, kInputW}}); // 设置输入维度

    // 获取输出维度信息
    Dims32 outDims = context->getBindingDimensions(1);  // 获取输出维度 [1, 84, 8400]
    OUTPUT_CANDIDATES = outDims.d[2];  // 设置输出候选框数量 (8400)
    int outputSize = 1;  // 计算输出数据总大小
    for (int i = 0; i < outDims.nbDims; i++){
        outputSize *= outDims.d[i];
    }

    // 在主机上分配输出数据空间
    outputData = new float[1 + kMaxNumOutputBbox * kNumBoxElement];
    // 在设备上分配输入和输出空间
    vBufferD.resize(2, nullptr);
    CHECK(cudaMalloc(&vBufferD[0], 3 * kInputH * kInputW * sizeof(float))); // 输入数据
    CHECK(cudaMalloc(&vBufferD[1], outputSize * sizeof(float))); // 输出数据

    CHECK(cudaMalloc(&transposeDevice, outputSize * sizeof(float))); // 转置数据
    CHECK(cudaMalloc(&decodeDevice, (1 + kMaxNumOutputBbox * kNumBoxElement) * sizeof(float))); // 解码数据
}

void YoloDetector::get_engine(){
    if (access(trtFile_.c_str(), F_OK) == 0){ // 检查 TensorRT 文件是否存在
        std::ifstream engineFile(trtFile_, std::ios::binary);
        long int fsize = 0;

        engineFile.seekg(0, engineFile.end);
        fsize = engineFile.tellg();
        engineFile.seekg(0, engineFile.beg);
        std::vector<char> engineString(fsize);
        engineFile.read(engineString.data(), fsize);
        if (engineString.size() == 0) { ROS_INFO("Failed getting serialized engine!"); return;}
        ROS_INFO("Succeeded getting serialized engine!");
        runtime = createInferRuntime(gLogger); // 创建推理运行时
        engine = runtime->deserializeCudaEngine(engineString.data(), fsize); // 反序列化引擎
        if (engine == nullptr) { ROS_INFO("Failed loading engine!"); return; }
        ROS_INFO("Succeeded loading engine!");
    } else {
        IBuilder *            builder     = createInferBuilder(gLogger); // 创建构建器
        INetworkDefinition *  network     = builder->createNetworkV2(1U << int(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH)); // 创建网络定义
        IOptimizationProfile* profile     = builder->createOptimizationProfile(); // 创建优化配置
        IBuilderConfig *      config      = builder->createBuilderConfig(); // 创建构建配置
        config->setMaxWorkspaceSize(1 << 30); // 设置最大工作区大小
        IInt8Calibrator *     pCalibrator = nullptr;
        if (bFP16Mode){
            config->setFlag(BuilderFlag::kFP16); // 启用 FP16 精度
        }
        if (bINT8Mode){
            config->setFlag(BuilderFlag::kINT8); // 启用 INT8 精度
            int batchSize = 8;
            pCalibrator = new Int8EntropyCalibrator2(batchSize, kInputW, kInputH, calibrationDataPath.c_str(), cacheFile.c_str()); // 创建 INT8 校准器
            config->setInt8Calibrator(pCalibrator);
        }

        nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, gLogger); // 创建 ONNX 解析器
        if (!parser->parseFromFile(onnxFile.c_str(), int(gLogger.reportableSeverity))){
           ROS_INFO("Failed parsing .onnx file!");
            for (int i = 0; i < parser->getNbErrors(); ++i){
                auto *error = parser->getError(i);
                std::cout << std::to_string(int(error->code())) << std::string(":") << std::string(error->desc()) << std::endl;
            }
            return;
        }
        ROS_INFO("Succeeded parsing .onnx file!");

        ITensor* inputTensor = network->getInput(0);
        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kMIN, Dims32 {4, {1, 3, kInputH, kInputW}}); // 设置最小尺寸
        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kOPT, Dims32 {4, {1, 3, kInputH, kInputW}}); // 设置最优尺寸
        profile->setDimensions(inputTensor->getName(), OptProfileSelector::kMAX, Dims32 {4, {1, 3, kInputH, kInputW}}); // 设置最大尺寸
        config->addOptimizationProfile(profile); // 添加优化配置文件

        IHostMemory *engineString = builder->buildSerializedNetwork(*network, *config); // 构建序列化网络
        ROS_INFO("Succeeded building serialized engine!");

        runtime = createInferRuntime(gLogger); // 创建推理运行时
        engine = runtime->deserializeCudaEngine(engineString->data(), engineString->size()); // 反序列化引擎
        if (engine == nullptr) { ROS_INFO("Failed building engine!"); return; }
        ROS_INFO("Succeeded building engine!");

        if (bINT8Mode && pCalibrator != nullptr){
            delete pCalibrator; // 删除校准器
        }

        std::ofstream engineFile(trtFile_, std::ios::binary);
        engineFile.write(static_cast<char *>(engineString->data()), engineString->size()); // 保存 .plan 文件
       ROS_INFO("Succeeded saving .plan file!");

        delete engineString; // 释放主机内存
        delete parser; // 释放解析器
        delete config; // 释放构建配置
        delete network; // 释放网络定义
        delete builder; // 释放构建器
    }
}

YoloDetector::~YoloDetector(){
    cudaStreamDestroy(stream); // 销毁 CUDA 流

    for (int i = 0; i < 2; ++i)
    {
        CHECK(cudaFree(vBufferD[i])); // 释放设备内存
    }

    CHECK(cudaFree(transposeDevice)); // 释放设备内存
    CHECK(cudaFree(decodeDevice)); // 释放设备内存

    delete [] outputData; // 释放主机内存

    delete context; // 释放推理上下文
    delete engine; // 释放引擎
    delete runtime; // 释放推理运行时
}

std::vector<Detection> YoloDetector::inference(cv::Mat& img){

    img_ = &img;
    if (img.empty()) return {}; // 如果图像为空，返回空结果

    // 将输入图像数据放到设备上，并进行预处理
    preprocess(img, (float*)vBufferD[0], kInputH, kInputW, stream);

    // 执行 TensorRT 推理
    context->enqueueV2(vBufferD.data(), stream, nullptr);

    // 转置数据 [1 84 8400] 到 [1 8400 84]
    transpose((float*)vBufferD[1], transposeDevice, OUTPUT_CANDIDATES, numClass_ + 4, stream);
    // 解码数据 [1 8400 84] 到 [1 7001]
    decode(transposeDevice, decodeDevice, OUTPUT_CANDIDATES, numClass_, confThresh_, kMaxNumOutputBbox, kNumBoxElement, stream);
    // 执行 CUDA 非极大值抑制 (NMS)
    nms(decodeDevice, nmsThresh_, kMaxNumOutputBbox, kNumBoxElement, stream);

    // 异步拷贝结果到主机
    CHECK(cudaMemcpyAsync(outputData, decodeDevice, (1 + kMaxNumOutputBbox * kNumBoxElement) * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream); // 等待 CUDA 流完成所有操作

    // 解析检测结果
    std::vector<Detection> vDetections;
    int count = std::min((int)outputData[0], kMaxNumOutputBbox); // 获取检测框数量
    for (int i = 0; i < count; i++){
        int pos = 1 + i * kNumBoxElement;
        int keepFlag = (int)outputData[pos + 6];
        if (keepFlag == 1){
            Detection det;
            memcpy(det.bbox, &outputData[pos], 4 * sizeof(float)); // 复制边界框数据
            det.conf = outputData[pos + 4]; // 复制置信度
            det.classId = (int)outputData[pos + 5]; // 复制类别 ID
            vDetections.push_back(det); // 将检测结果添加到列表中
        }
    }
    // 对检测框进行缩放
    for (size_t j = 0; j < vDetections.size(); j++){
        scale_bbox(img, vDetections[j].bbox);
    }

    return vDetections; // 返回检测结果
}

