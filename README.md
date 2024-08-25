# YOLOv8 目标检测模型通过 TensorRT部署

## 更新日志

### v3.0 - 2024.8.25

- 大量config参数从config.cpp文件转移到launch文件当中,方便参数调整
- 合并多个publish为一
- 增加pose检测

### v2.1 - 2024.8.22

- 合并d_camera_infer_node和camera_infer_node，统一使用camera_infer_node
- 增加depth变量

### v2.0 -  2024.8.19

- 加入bytetrack算法
- 增加d_camera_infer_node和camera_infer_node的track功能

### v1.1 - 2024.8.14

- 删除用于表示检测结果的type.h文件
- 增加infer_result.msg和results.msg文件用于表示和publish检测结果
- 实现检测结果在ros上的publish
- 增加d435i_yolo.launch文件
- 增加track变量
- 完善config文件
- 完善readme

### v1.0 - 2024.8.3

- 实现基本的相机和照片的目标检测功能

## 实现效果

### 目标检测

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=112898986738795&bvid=BV1Niv9erEuw&cid=500001637389749&p=1" 
    scrolling="no" 
    border="0" 
    frameborder="no" 
    framespacing="0" 
    allowfullscreen="true" 
    width="640" 
    height="480">
</iframe>

### 目标追踪

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=112989013279998&bvid=BV1NmpSeeE2o&cid=500001655010412&p=1"   scrolling="no" 
    border="0" 
    frameborder="no" 
    framespacing="0" 
    allowfullscreen="true" 
    width="640" 
    height="480">></iframe>



### pose检测

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=113023624675958&bvid=BV1fzWCeBEPx&cid=500001662189599&p=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" 
    width="640" 
    height="480"></iframe>


## 环境
ubuntu 20.04 + ros noetic

opencv 4.8.0

cuda 12.1 + cudnn 8.9.7

TensorRT 8.6.1.6

cmake 3.22.1

注意：

- 运行时可能报错 段错误(核心已转储)，这是你自己的opencv版本和ros默认的版本造成冲突导致的，删除自己的版本使用ros默认的opencv即可解决报错
- 目前仅支持TensorRT 8，使用10会报错

## 文件结构

.
├── CMakeLists.txt

├── images

│   ├── bus.jpg

│   ├── dog.jpg

│   ├── eagle.jpg

│   ├── field.jpg

│   ├── giraffe.jpg

│   ├── herd_of_horses.jpg

│   ├── person.jpg

│   ├── room.jpg

│   ├── street.jpg

│   └── zidane.jpg

├── include

│   ├── BytekalmanFilter.h 

│   ├── BYTETracker.h

│   ├── calibrator.h

│   ├── camera_infer.h

│   ├── config.h

│   ├── dataType.h

│   ├── image_infer.h

│   ├── infer.h

│   ├── InferResult.h

│   ├── KeyPoint.h

│   ├── lapjv.h

│   ├── postprocess.h

│   ├── preprocess.h

│   ├── public.h

│   ├── Results.h

│   ├── STrack.h

│   ├── types.h

│   └── utils.h

├── launch

│   └── d435i_yolo.launch

├── msg

│   ├── infer_result.msg

│   └── results.msg

├── onnx_model

│   ├── yolov8s.onnx

│   └── yolov8s.plan

├── package.xml

├── README-en.md

├── README.md

└── src

​    ├── BytekalmanFilter.cpp

​    ├── BYTETracker.cpp

​    ├── calibrator.cpp

​    ├── camera_infer.cpp

​    ├── config.cpp

​    ├── image_infer.cpp

​    ├── infer.cpp

​    ├── lapjv.cpp

​    ├── postprocess.cu

​    ├── preprocess.cu

​    └── STrack.cpp



## 导出ONNX模型

1. 安装 `YOLOv8`

```bash
pip install ultralytics
```

- 建议同时从 `GitHub` 上 clone 或下载一份 `YOLOv8` 源码到本地；
- 在本地 `YOLOv8`一级 `ultralytics` 目录下，新建 `weights` 目录，并且放入`.pt`模型

2. 安装onnx相关库

```bash
pip install onnx==1.12.0
pip install onnxsim==0.4.33
```

3. 导出onnx模型

- 可以在一级 `ultralytics` 目录下，新建 `export_onnx.py` 文件
- 向文件中写入如下内容：

```python
from ultralytics import YOLO

model = YOLO("./weights/Your.pt", task="detect")
path = model.export(format="onnx", simplify=True, device=0, opset=12, dynamic=False, imgsz=640)
```

- 运行 `python export_onnx.py` 后，会在 `weights` 目录下生成 `.onnx`

## 安装编译

1. 将仓库clone到自己的ros工作空间中；

   ```bash
   cd catkin_ws/src
   git clone https://github.com/wyf-yfw/TensorRT_YOLO_ROS.git
   ```

2. 如果是自己数据集上训练得到的模型，记得更改 `src/config.cpp` 中的相关配置，所有的配置信息全部都包含在`config.cpp`中；

3. 确认 `CMakeLists.txt` 文件中 `cuda` 和 `tensorrt` 库的路径，与自己环境要对应，一般情况下是不需修改的；

4. 将已导出的 `onnx` 模型拷贝到 `onnx_model` 目录下

5. 编译工作空间

## 运行节点

目前共有两个节点，分别是image_infer_node、camera_infer_node

先运行自己的相机节点，然后运行相应的推理节点

d435i相机可以直接运行launch文件

```bash
roslaunch tensorrt_yolo d435i_yolo.launch 
```

在launch文件中调整自己的参数，需要移植可以直接将下面这部分复制到自己的launch文件当中

```xml
<!-- 启动目标检测节点 -->
 <node name="yolo_node" pkg="tensorrt_yolo" type="camera_infer_node" output="screen">
     
      <!-- 是否启动目标跟踪 -->
        <param name="track" value="true"/>
        <!-- 是否启动深度相机 -->
        <param name="depth" value="true"/>
        <!-- 是否启动姿态检测 -->
        <param name="pose" value="true"/>

        <!-- rgb图像topic -->
        <param name="rgbImageTopic" value="camera/color/image_raw"/>
        <!-- depth图像订阅地址,没有则忽略 -->
        <param name="depthImageTopic" value="/camera/depth/image_rect_raw"/>

        <!-- .plan文件地址 -->
        <param name="planFile" value="/home/wyf/catkin_ws/src/TensorRT_YOLO_ROS/onnx_model/yolov8s-pose-fp16.plan"/>
        <!-- .onnx文件地址 -->
        <param name="onnxFile" value="/home/wyf/catkin_ws/src/TensorRT_YOLO_ROS/onnx_model/yolov8s-pose.onnx"/>

        <!-- 非极大值抑制 -->
        <param name="nmsThresh" type = "double" value="0.7"/>
        <!-- 置信度 -->
        <param name="confThresh" type = "double" value="0.7"/>

        <!-- 目标检测类型数量 -->
        <param name="numClass" type = "int" value="1"/>
        <!-- 姿态检测特征点 -->
        <param name="numKpt" type = "int" value="17"/>
        <!-- 姿态检测维度 -->
        <param name="kptDims" type = "int" value="3"/>

   </node>
```

## 节点订阅数据

```
const std::string rgbImageTopic = "/camera/color/image_raw"; //ros中image的topic
const std::string depthImageTopic = "/camera/depth/image_rect_raw"; //ros中depth image的topic
```

## 节点发布数据

节点对外publisher有一个，为infer_results，发布的内容均为一个列表，列表中的元素结构是

```cpp
float32[4] bbox
float32 conf
int32 classId
float32[3] coordinate //d_camera_infer_node发布三维空间坐标，camera_infer_node发布二维坐标，z=0
int32 Id // 关闭追踪模式默认为0，开启目标追踪为当前追踪的id值
float32 kpts // 存储未缩放到原始图像上的关键点数据
KeyPoint[] kpts // 存储经过缩放处理后的关键点数据，为了将关键点坐标映射回原始图像中的坐标系

```



