# lslidar_c16
#version v2.0.3_200103
## version track
Author: zx
### ver2.0.3 zx

## Description
The `lslidar_c16` package is a linux ROS driver for lslidar c16.
The package is tested on Ubuntu 16.04 and Ubuntu 18.04 with ROS kinetic.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
```

## Example Usage

### lslidar_c16_decoder

**Parameters**

`lidar_ip` (`string`, `default: 192.168.1.200`)

By default, the IP address of the device is 192.168.1.200.

`frame_id` (`string`, `default: laser_link`)

The frame ID entry for the sent messages.

**Published Topics**

`lslidar_point_cloud`

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

### lslidar_c16_decoder

**Parameters**

`min_range` (`double`, `0.3`)

`max_range` (`double`, `200.0`)

Points outside this range will be removed.

`frequency` (`frequency`, `10.0`)

Note that the driver does not change the frequency of the sensor. 

`publish_point_cloud` (`bool`, `true`)

If set to true, the decoder will additionally send out a local point cloud consisting of the points in each revolution.

**Published Topics**

`lslidar_sweep` (`lslidar_c16_msgs/LslidarChSweep`)

The message arranges the points within each sweep based on its scan index and azimuth.

`lslidar_point_cloud` (`sensor_msgs/PointCloud2`)

This is only published when the `publish_point_cloud` is set to `true` in the launch file.

**Node**

```
roslaunch lslidar_c16_decoder lslidar_c16.launch --screen
```
Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ


## Bug Report


##Version changes
/***********2020-01-03****************/
Original version : lslidar_c16_v2.02_190919
Revised version  : lslidar_c16_v2.03_200103
Modify  		 : Add a new calibration decode for the new lslidar c16
Author			 : zx
Date			 : 2020-01-03



# 1.硬件连接

首先连接雷达网线和电源线，根据雷达设置目标IP设置电脑有线连接IP地址（默认为192.168.1.102)、子网掩码（255.255.255.0）和网关（192.168.1.1）

设置完成后重启雷达电源

测试是否可以ping通

```
ping 192.168.1.102
```

ping通后，查看雷达发送数据包情况

```
sudo tcpdump -n -i eth0
```

eth0为有线网络设备名，如果最后为1206个字节，则表示雷达数据发送正常



# 2.软件操作

创建ros工作空间，并在src文件夹下下载驱动

```
git clone https://github.com/zhangzhouyuan/lslidar_c16
```

编译

```
roslaunch lslidar_c16_decoder lslidar_c16.launch
```

备注：修改了雷达目的端口及转速，请打开 lslidar_c16.launch 进行相应的修改配置，默认端口为 2368，转速为 10HZ 即 point_num 为 2000 点。