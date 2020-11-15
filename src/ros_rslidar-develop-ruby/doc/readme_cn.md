# RSLidar ROS 驱动说明

### 1. 依赖
* 1.安装ubuntu系统的电脑，推荐Ubuntu14.04或Ubuntu16.04，不要使用虚拟机
* 2.安装 ROS full-desktop版本。建议安装Indigo或Kinect
* 3.安装libpcap-dev

### 2. 安装和使用
* 1.将这个rslidar驱动拷贝到ROS工作区，例如“~/catkin_ws/src"
* 2.设置文件访问属性
```
cd ~/catkin_ws/src/ros_rslidar/rslidar_drvier
chmod 777 cfg/*
cd ~/catkin_ws/src/ros_rslidar/rslidar_pointcloud
chmod 777 cfg/*
```
* 3.编译和安装
```
cd ~/catkin_ws
catkin_make
```

* 4.设置PC ip  
默认情况下，RSLIDAR雷达设备ip设置为**192.168.1.200**，而PC ip则设置为**192.168.1.102**；同时默认**MSOP**端口为**6699**，**DIFOP**端口为**7788**. 所以需将PC ip设置为静态ip **192.168.1.102**才能和雷达通信。

* 5.以node方式运行  
这里提供示例launch文件，可以运行launch文件来观看点云数据。观看RS-LIDAR-16实时点云数据例子操作如下：
   
   * 1)打开一个终端并执行以下命令
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch rslidar_pointcloud rs_lidar_128.launch
```
   * 2)新开一个终端并执行命令：
```
rviz
```
然后设置Fixed Frame为**“rslidar”**，接着添加Pointcloud2 类型和设置topic为**“rslidar_points"**。

* 6.以nodelet方式运行  
同时支持driver node和cloud node以nodelet方式运行。打开终端并执行以下命令：
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch rslidar_pointcloud cloud_nodelet.launch
```
然后运行“rviz”来观看点云。

* 7.雷达校准参数  
**“rslidar_pointcloud/data”**目录下，存放雷达的校准参数文件。通过默认launch文件“rs_lidar_16.launch”加载以下3个文件：
   
 * rslidar_pointcloud/data/rs_lidar_128/angle.csv
 * rslidar_pointcloud/data/rs_lidar_128/ChannelNum.csv

 
### 3.参数说明
雷达有一些参数需要设置，需要通过launch文件进行传递。
* model: 指定雷达类型。目前两个选项 RS32 和 RS16 分别指32线和16线雷达
* device_ip：雷达ip
* msop_port：MSOP数据包接收端口
* difop_port：DIFOP数据包接收端口
* lidar_param_path：雷达静态量文件存放位置
* pcap：要解析的pcap包存放路径。注意指定此参数，则驱动就不会接收从雷达发出的数据
* angle_path：指定静态文件angle.csv完整路径
* channel_path：指定静态文件ChannelNum.csv完整路径
* max_distance：指定点云距离上限
* min_distance：指定点云距离下限
* cut_angle：采用角度分帧方式