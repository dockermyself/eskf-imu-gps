实现了IMU + GPS融合定位的功能，具体实现细节请参考代码注释。

## 1. 依赖
- Eigen3
## 2. 编译
```bash
mkdir build
cd build
cmake ..
make
```
## 3. 运行
```bash

```
## 4.效果

![ESKF融合定位结果](<Position Trajectory.png>)
![速度随时间变化](<Velocity Trajectory.png>)