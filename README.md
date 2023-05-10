# stage_ros2 使用说明

stage_ros2是一个基于ROS2的2D机器人模拟器（Ubuntu22.04 + ROS2 humble）。

## 1.安装

请先调用如下指令安装依赖：

```
sudo apt-get install git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.1-dev
```

进入ROS2工作空间的src目录，调用如下指令下载相关仓库：

```
git clone https://github.com/damuxt/Stage.git
git clone https://github.com/damuxt/stage_ros2.git
```

进入工作空间目录，使用`colcon build`指令进行构建。

## 2.使用

在功能包下，已经内置了使用示例，终端下可以执行如下指令启动示例：

```
ros2 launch stage_ros2 my_house.launch.py
```

2D模拟器以及rviz2将被启动，并且在rviz2中可以显示里程计、激光雷达以及深度相机等相关信息。

