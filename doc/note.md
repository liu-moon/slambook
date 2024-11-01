# Note 
本项目将视觉slam十四讲的代码利用ROS2进行重新编写，学习的同时熟悉ROS2的相关操作。

## 第2讲 初识SLAM

### helloSLAM.cpp

helloSLAM 打印 Hello SLAM!

这里采用log的方式进行打印

在ros2中操作的流程为：

1. 初始化系统
2. 创建节点
3. 阻塞节点
4. 关闭系统

具体代码查看 src/ch2/src/helloSLAM.cpp

## 第3讲 三维空间刚体运动

### eigenMatrix.cpp

eigenMatrix.cpp 主要演示了如何使用eigen库以及矩阵的基本操作与运算。

主体代码和书中的代码相同，不同的是我将代码添加到了节点的构造函数中

具体代码查看 src/ch3/src/eigenMatrix.cpp

### useGeometry.cpp

useGeometry.cpp 演示了如何在Eigen中使用四元数、欧拉角和旋转矩阵，以及他们的转换方式。

与eigenMatrix一样，将代码添加到了节点的构造函数中。

具体代码查看 src/ch3/src/useGeometry.cpp

### coordinateTransform.cpp

coordinateTransform.cpp 演示坐标变换，要注意的是四元数在使用之前需要归一化。

与eigenMatrix一样，将代码添加到了节点的构造函数中。

具体代码查看 src/ch3/src/coordinateTransform.cpp

### plotTrajectory.cpp

plotTrajectory.cpp 演示了 Pangolin 库的GUI功能，这里我采用了 rviz2 进行修改

```shell
cd slambook
rviz2 -d src/ch3/rviz/plotTrajectory.rviz
ros2 run ch3 plotTrajectory
```

### plotTrajectory_arrow.cpp

plotTrajectory_arrow.cpp 在 plotTrajectory 的基础上添加了位姿的坐标轴

```shell
cd slambook
rviz2 -d src/ch3/rviz/plotTrajectory_arrow.rviz
ros2 run ch3 plotTrajectory_arrow
```

## 第4讲 李群和李代数

### useSophus.cpp

useSophus.cpp 演示了 Sophus 库的使用

### trajectoryError.cpp

trajectoryError.cpp 演示了如何评估轨迹的误差

```shell
cd slambook
rviz2 -d src/ch4/rviz/trajectoryError.rviz
ros2 run ch4 trajectoryError
```

## 第5讲 相机与图像

### imageBasics.cpp

imageBasics.cpp 演示了图像的基本操作，我在案例中添加了定时器发布图像数据的功能。

### undistortImage.cpp

undistortImage.cpp 演示了如何去除图像的径向和切向畸变的过程。

### stereoVision.cpp

stereoVision.cpp 演示了利用双目视觉原理构成点云 主要采用了 SGBM 算法实现的 对于SGBM 算法中的参数设置 可以通过设置滑块的方式 查看相应效果再确定

```shell
cd slambook
rviz2 -d src/ch5/rviz/stereoVision.rviz
ros2 run ch5 stereoVision
```

### jointMap.cpp

jointMap.cpp 演示了利用RGB-D相机对点云进行拼接的过程

```shell
cd slambook
rviz2 -d src/ch5/rviz/jointMap.rviz
ros2 run ch5 jointMap
```
