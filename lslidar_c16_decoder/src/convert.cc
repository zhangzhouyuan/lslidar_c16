/*
 * This file is part of lslidar_n301 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "lslidar_c16_decoder/convert.h"
#include <pcl_conversions/pcl_conversions.h>

namespace lslidar_c16_decoder
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new lslidar_rawdata::RawData())
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters
  private_nh.param("model", model, std::string("LSC16"));
  ROS_INFO_STREAM("convert model : " << model);

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  private_nh.param("output_points_topic", output_points_topic, std::string("lslidar_point_cloud"));
  output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);
  ROS_INFO_STREAM("convert output_points_topic : " << output_points_topic);

  // subscribe to lslidar packets
  std::string input_packets_topic;
  private_nh.param("input_packets_topic", input_packets_topic, std::string("lslidar_packet"));
  packet_sub_ = node.subscribe(input_packets_topic, 10, &Convert::processScan, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));

  private_nh.param("time_synchronization", time_synchronization_, false);
  if (time_synchronization_)
  {
    sync_sub_ = node.subscribe("sync_header", 10, &Convert::timeSync, (Convert*)this,
                   ros::TransportHints().tcpNoDelay(true));
  }
}

void Convert::timeSync(const sensor_msgs::TimeReferenceConstPtr &time_msg)
{
  global_time = time_msg->header.stamp;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const lslidar_c16_msgs::LslidarC16ScanUnified::ConstPtr& scanMsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  if (time_synchronization_)
    outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  else
  {
    outPoints->header.stamp = ros::Time::now().toNSec() / 1000ull;
  }
  outPoints->header.frame_id = scanMsg->header.frame_id;

  outPoints->clear();
  outPoints->height = 16;
  outPoints->width = 24 * (int)scanMsg->packets.size();
  //ROS_INFO("packets.size=%d",scanMsg->packets.size());
  outPoints->is_dense = false;
  outPoints->resize(outPoints->height * outPoints->width);

  // process each packet provided by the driver
  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], outPoints);
  }

  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*outPoints, outMsg);

  output_.publish(outMsg);
}
}  // namespace lslidar_c16_decoder
