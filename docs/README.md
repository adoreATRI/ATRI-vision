# ATRI_vision

## 目录

- [功能介绍](#功能介绍)
- [部署](#部署)
- [ATRI_vision编译和运行](#atri_vision编译和运行)
- [按键控制节点的运行](#按键控制节点的运行)
- [其他工具的使用](#其他工具的使用)

## 功能介绍

- usb_camera功能包：使用l4v2的api进行开发，有重连机制，发布图像话题
- detector功能包：使用yolov8模型获得roi，再使用传统opencv进行图像处理，色块检测采用hsv直方图作为特征进行匹配，采用学习锁定机制与圆形色块解耦并增强鲁棒性
- tracker功能包：参考华师的开源，使用EKF和GNS对检测结果进行平滑和预测
- serial_driver功能包：参考华师的开源，增加了角度解算和按键控制等功能

## 部署

- ONNXRuntime
下载对应适合自己系统的版本，在.bashrc中添加以下内容：

```bash
export ONNXRUNTIME_DIR=/home/adore/onnxruntime-*
```

修改参数

- atri_detector的pnp_solver的目标大小
- atri_tracker中tracker.hpp中BUFF_R的大小
- atri_serial_driver中的tf树调整和时间补偿

可选：

- usb_camera中相机参数的调整
- atri_serial_driver中的角度补偿

## ATRI_vision编译和运行

在`ATRI_vision`的根目录终端运行以下命令：

```bash
colcon build
source install/setup.bash
ros2 launch bringup launch.py
```

## 按键控制节点的运行

在`ATRI_vision`的根目录终端运行以下命令：

```bash
source install/setup.bash
ros2 run atri_keyboard keyboard_node
```

其中r控制重置检测器和追踪器状态，并切换能量机关模式
s控制数据的发送

## 其他工具的使用

### 仿真器的使用

运行

```bash
/usr/bin/env python3 ./src/simulator/buff_simulator.py
```

仿真器使用说明：
    r     → 重置色块颜色
    m     → 切换能量机关模式
    d     → 切换方向
    p     → 重置参数

### 调试工具

```bash
ros2 run plotjuggler plotjuggler

ros2 bag record -o bags/01 /detector/color_blocks /tf /tf_static
```
