# YOLOv8 目标检测模型转 TensorRT

## 环境
ubuntu 20.04 + ros noetic

opencv 4.8.0

cuda 12.1 + cudnn 8.9.7

TensorRT 8.6.1.6

cmake 3.22.1

注意：运行时可能报错 段错误(核心已转储)，这是你自己的opencv版本和ros默认的版本造成冲突导致的，删除自己的版本使用ros默认的opencv即可解决报错

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

│   ├── d_camera_infer.h

│   ├── image_infer.h

│   ├── infer.h

│   ├── infer_result.h

│   ├── lapjv.h

│   ├── postprocess.h

│   ├── preprocess.h

│   ├── public.h

│   ├── results.h

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

​    ├── d_camera_infer.cpp

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

目前共有三个节点，分别是image_infer_node、camera_infer_node以及d_camera_infer_node

先运行自己的相机节点，然后运行相应的推理节点

d435i相机可以直接运行launch文件

```bash
roslaunch tensorrt_yolo d435i_yolo.launch 
```

可以在launch文件中选择是否开启追踪

```xml
<!-- 启动目标检测节点 -->
 <node name="yolo_node" pkg="tensorrt_yolo" type="camera_infer_node" output="screen">
     <!-- 是否启动目标跟踪 -->
     <param name="track" value="false"/>
   </node>
```

默认为false关闭追踪，可以自行选择开启

## 节点订阅数据

```
const std::string rgbImageTopic = "camera/color/image_raw"; //ros中image的topic
const std::string depthImageTopic = "/camera/depth/image_rect_raw"; //ros中depth image的topic
```

## 节点发布数据

节点对外发布节点名称有两个，关闭目标追踪仅目标检测为/detect_results，开启目标追踪为/track_results，发布的内容均为一个列表，列表中的元素结构是

```cpp
float32[4] bbox
float32 conf
int32 classId
float32[3] coordinate //d_camera_infer_node发布三维空间坐标，camera_infer_node发布二维坐标，z=0
int32 Id // 关闭追踪模式默认为0，开启目标追踪为当前追踪的id值
```



