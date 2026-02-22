## ATRI_vision环境配置
模型通过ONNX Runtime推理的

### onnxruntime配置
在`~/.bashrc`中添加`export ONNXRUNTIME_DIR=/home/*/onnxruntime-linux-x64-gpu-*`版本根据自己gpu型号选择

## ATRI_vision编译和运行
在`ATRI_vision`的根目录下运行以下命令：
```bash
colcon build
source install/setup.bash
ros2 launch bringup system.launch.py
```

## 其他工具的使用
### 仿真器的使用
运行
```bash
/usr/bin/env python3 ./src/simulator/buff_simulator.py
```
仿真器使用说明：
    r     → 重置色块颜色
    m     → 切换能量机关模式






