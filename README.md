# 手机3D视觉重建

## 编译方式

```bash
## install 里创立链接，减少文件的创建
colcon build --symlink-install

## 单个 package 编译
colcon build --packages-select video_player

## 添加 CMakeLists.txt 打印输出
colcon build --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

## 测试运行

```bash
## 清理编译环境
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

source install/setup.bash

ros2 run feature_tracker feature_tracker_node
ros2 run vins_estimator vins_estimator_node
```

### 测试命令

```bash
## 录包
ros2 bag record /your_topic_name
ros2 bag record /topic1 /topic2
ros2 bag record -o output_file /topic_name

## 播包
ros2 bag play my_bag
### 以特定速率播放
ros2 bag play my_bag --rate 0.5
ros2 bag play my_bag --rate 2.0

### 循环播放
ros2 bag play my_bag --loop

```

## 集成运行

```bash
# 1. 检查路径解析
# ros2 launch video_player video_player.launch.py
ros2 launch feature_tracker feature_tracker.launch.py
# 2. 查看实际加载的参数
ros2 param list
# 3. 确认视频文件路径
ros2 param get /video_player video_path
```

## 实现流程

### 视频发送节点

+ 读取视频的每一帧数据，转换为ROS2的 Image 消息发布
+ 可以对图片进行预处理，如降噪，分辨率调整，以提升后续算法效率和精度

### 特征追踪

+ 输入： 订阅 /video_frames 话题(Image 消息)
+ 处理：
    + 计算关键点和描述子，匹配相邻帧之间的特征点
+ 输出 每一关键帧的关键点

### 位姿估计

### 位姿图优化

### 稀疏重建 SFM

通过2D特征点匹配关系，求解相机在每一帧的3D位置和姿态

### 稠密重建