## 仿真器的使用
运行
```bash
/usr/bin/env python3 /home/adore/ATRI_vision/src/simulator/buff_simulator.py
```

## ATRI_vision编译和运行
在`ATRI_vision`的根目录下运行以下命令：
```bash
colcon build
source install/setup.bash
ros2 launch bringup system.launch.py
```