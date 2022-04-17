# HikRobot-Camera-ROS
海康威视 工业相机 转发图像到ROS

* 本项目提供了两种方式：C++程序可以发布RGB图；Python脚本可以发布灰度图

## 使用说明

**Preqeuest**

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/jike5/HikRobot-Camera-ROS.git
catkin_make
```

C++程序：

```bash
roscore
# new terminal
rosrun mvs_image_ros pub_image
```

Python:

```bash
roscore
# new terminal
python src/py_pub_image.py
```

