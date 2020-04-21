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

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include "rawdata.h"
#include <sensor_msgs/TimeReference.h>

namespace lslidar_c16_decoder
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~Convert()
  {
  }

private:

  void processScan(const lslidar_c16_msgs::LslidarC16ScanUnified::ConstPtr& scanMsg);
  void timeSync(const sensor_msgs::TimeReferenceConstPtr& time_msg);
  /// Pointer to dynamic reconfigure service srv_

  boost::shared_ptr<lslidar_rawdata::RawData> data_;
  ros::Subscriber packet_sub_;
  ros::Subscriber sync_sub_;
  ros::Time global_time;

    bool time_synchronization_;
  ros::Publisher output_;
};

}  // namespace lslidar_c16_decoder
#endif
