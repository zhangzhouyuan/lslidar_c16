# lslidar_c16

## version track
Author: Yutong

### ver1.2 Yutong
1. Add group multicast function
2. Add truncate 3D angle area data


### ver1.1  Yutong
Using new message type to distinguish different channel data
topic name: scan_channel
topic type: LslidarC16Layer
details: LslidarC16Layer is consist of 16 sets data for different channel, each set of data is represented by Sensor_msgs/LaserScan rosmessage type
Usage: rostopic echo /scan_channel  will output all 16 channels data
       rostopic echo /scan_channel/scan_channel[*]  (* can be from 0 to 15 represents channel num)  --> output data will be sensor_msgs/LaserScan message type
Example: There is an example script to show you how to obtain each channel data, located at /lslidar_c16_decoder/scripts/Test_MultiChannel.py
	 You will need python package numpy and matplotlib.pyplot(optional) to fully run this script


### ver1.05 Yutong
Using rostopic to select the channel you wish to output
topic name: layer_num
topic type: std_msgs/Int8
details: send layer number to topic layer_num 
Usage: rostopic pub /layer_num std_msgs/Int8 "data: 5"  --> output channel 5 data to topic /scan, message type is sensor_msgs/LaserScan . data number can only from 0 to 15

## Description
The `lslidar_c16` package is a linux ROS driver for lslidar c16.
The package is tested on Ubuntu 14.04 with ROS indigo.

## Compling
This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```
cd your_work_space
catkin_make 
```

## Example Usage

### lslidar_c16_decoder

**Parameters**

`device_ip` (`string`, `default: 192.168.1.200`)

By default, the IP address of the device is 192.168.1.200.

`frame_id` (`string`, `default: laser`)

The frame ID entry for the sent messages.

**Published Topics**

`lslidar_packets` (`lslidar_c16_msgs/LslidarC16Packet`)

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

### lslidar_c16_decoder

**Parameters**

`min_range` (`double`, `0.15`)

`max_range` (`double`, `150.0`)

Points outside this range will be removed.

`frequency` (`frequency`, `10.0`)

Note that the driver does not change the frequency of the sensor. 

`publish_point_cloud` (`bool`, `true`)

If set to true, the decoder will additionally send out a local point cloud consisting of the points in each revolution.

**Published Topics**

`lslidar_sweep` (`lslidar_c16_msgs/LslidarC16Sweep`)

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

# 1.硬件连接
首先连接雷达网线和电源线，根据雷达设置目标IP设置电脑有线连接IP地址（默认为192.168.1.102），当电脑有线网络衔接正常后，测试是否可以ping通，之后在通过
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
roslaunch lslidar_c16_decoder c16.launch

```
备注：若修改了雷达目的端口及转速，请打开 lslidar_c16.launch 进行相应的修改配置，默认端口为 2368，转速为 10HZ 即 point_num 为 2000 点。







