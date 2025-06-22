# Livox Mid360 无人机部署
时间： 2025 年 6 月 21 日

源码地址：

（1）Livox SDK2 : https://github.com/Livox-SDK/Livox-SDK2.git

（2）Livox ros driver2 : https://github.com/Livox-SDK/livox_ros_driver2.git

（3）Fast LIO2 : https://github.com/hku-mars/FAST_LIO.git

> 说明：本仓库提供的Fast LIO2代码为修改过后的Fast LIO代码，其相比源码将FastLIO的里程计信息（话题：/Odometry）转发给mavros（话题：/mavros/vision_pose/odom）用于融合。

可参考链接：

（1）Livox Mid360 部署方法 : 

https://hitxjf.github.io/2023/02/12/Mid-360%20%E5%BF%AB%E9%80%9F%E9%85%8D%E7%BD%AE%E6%8C%87%E5%8D%97/

https://developer.aliyun.com/article/1592459

http://www.chinasem.cn/article/351411

https://blog.csdn.net/m0_62948300/article/details/140209873

（2）雷达里程计信息传入飞控的参数设置方法 : 

https://blog.csdn.net/woaixiaojiang/article/details/141469147


## 1、前置——Livox Viewer 2 测试mid360 (可选)

前往Livox官网 https://www.livoxtech.com/cn/mid-360/downloads 下载并解压Livox Viewer 2 - Ubuntu

```bash 
./LivoxViewer2.sh
```

将雷达与电脑连接，运行LivoxViewer2后正常情况无需任何配置即可看到雷达点云

## 2、Livox SDK2 安装（必须）

``` bash 
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

## 3、Livox ros driver2 安装 （必须）

```bash
mkdir -p livox_ws/src
cd livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh ROS1
```

## 4、Mid360激光雷达配置及驱动测试 （必须）
### 雷达参数配置
（1）固定PC端有线网IP

    设定有线IPv4方式：手动；

    地址：192.168.1.50；
    
    子网掩码：255.255.255.0；
    
    网关：192.168.1.1；

（2）更改livox_ws/src/livox_ros_driver2/config/MID360_config.json中的IP
```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.50",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.50",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50",
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.50",
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.1XX",
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

"cmd_data_ip" : "192.168.1.50" 为设置的本地固定IP

"ip" : "192.168.1.1XX" 为激光雷达的ip地址，XX对应SN码的后两位


### 测试

```bash 
cd livox_ws/src
source devel/setup.bash
roslaunch livox_ros_driver2 rviz_MID360.launch
```

> 说明: rviz_MID360.launch 启动代码 发布的点云话题消息类型为 PointCloud2 标准消息 能够在rviz中可视化；msg_MID360.launch 启动代码 发布的点云话题消息类型为 Livox 自定义消息 不能够在rviz中可视化。

## 5、Fast LIO 安装

### 克隆源码
```bash 
mkdir -p  ~/fast_lio_ws/src
cd ~/fast_lio_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
```

### 克隆ikd-tree
克隆下来的代码缺少ikd-tree源码需要手动添加
```bash
git clone https://github.com/hku-mars/ikd-Tree.git
```
克隆ikd-tree源码复制到 ~/fast_lio_ws/src/FAST_LIO/ikd-Tree下


### 修改源码
因为mid360用的是livox_ros_driver2，为了使其与mid360适配，必须修改代码。

用vscode打开fast_lio_ws工作空间，搜索：livox_ros_driver

直接选择全部替换为：livox_ros_driver2

### 编译
```bash 
cd ~/fast_lio_ws/src
catkin_make
```

### 运行

打开两个终端
```bash
# 终端1
cd ~/livox_ws
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
```
```bash
# 终端2
cd ~/fast_lio_ws
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

## FAST LIO里程计信息传入mavros

### 修改FAST LIO代码
转发FAST LIO里程计话题/Odometry 到mavros 

/mavros/odometry/out话题（或 /mavros/vision_pose/pose）

具体可参考仓库代码中的laserMapping.cpp


### 飞控参数设置

(1) 搜索参数ekf2_aid_mask，把它改为使用vision的位置和yaw值，即改为24

(2) 无人机的高度来源也需要修改，修改参数"EKF2_HGT_MODE"为"vision"
