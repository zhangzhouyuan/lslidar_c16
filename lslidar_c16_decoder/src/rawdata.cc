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

#include "lslidar_c16_decoder/rawdata.h"
#include <angles/angles.h>
namespace lslidar_rawdata
{
RawData::RawData()
{
  this->is_init_angle_ = false;
  this->is_init_curve_ = false;
  this->is_init_top_fw_ = false;
}

void RawData::loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string model;
  std::string resolution_param;

  private_nh.param("start_angle", start_angle_, float(0));
  private_nh.param("end_angle", end_angle_, float(360));
  private_nh.param("calibration_file", calibration_file_, std::string(""));

  if (start_angle_ < 0 || start_angle_ > 360 || end_angle_ < 0 || end_angle_ > 360)
  {
    start_angle_ = 0;
    end_angle_ = 360;
    ROS_INFO_STREAM("start angle and end angle select feature deactivated.");
  }
  else
  {
    ROS_INFO_STREAM("start angle and end angle select feature activated.");
  }

  angle_flag_ = true;
  if (start_angle_ > end_angle_)
  {
    angle_flag_ = false;
    ROS_INFO_STREAM("Start angle is smaller than end angle, not the normal state!");
  }

  ROS_INFO_STREAM("start_angle: " << start_angle_ << " end_angle: " << end_angle_ << " angle_flag: " << angle_flag_);

  start_angle_ = start_angle_ / 180 * M_PI;
  end_angle_ = end_angle_ / 180 * M_PI;

  private_nh.param("max_distance", max_distance_, 200.0f);
  private_nh.param("min_distance", min_distance_, 0.2f);
  private_nh.param("cbMethod", cbMethod_, true);

  ROS_INFO_STREAM("distance threshlod, max: " << max_distance_ << ", min: " << min_distance_);

  intensity_mode_ = 1;
  info_print_flag_ = false;

  private_nh.param("model", model, std::string("LSC16"));
  numOfLasers = 16;
  R1_ = 0.04319;
  R2_ = 0.010875;

  intensityFactor = 51;

  //return mode default
  return_mode_ = 1;

  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_azimuth_table[rot_index] = cosf(rotation);
    sin_azimuth_table[rot_index] = sinf(rotation);
  }

  // receive difop data
  // subscribe to difop lslidar packets, if not right correct data in difop, it will not revise the correct data in the
  // VERT_ANGLE, HORI_ANGLE etc.
  difop_sub_ = node.subscribe("lslidar_packets_difop", 10, &RawData::processDifop, (RawData*)this);
}

void RawData::processDifop(const lslidar_c16_msgs::LslidarC16Packet::ConstPtr& difop_msg)
{
  // std::cout << "Enter difop callback!" << std::endl;
  const uint8_t* data = &difop_msg->data[0];
  bool is_support_dual_return = false;

  // check header
  if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a)
  {
    return;
  }

  // rpm
  if ((data[4]==0x04) && (data[5]==0xB0))
  {
    ROS_INFO("rpm is 1200");
  }
  else if ((data[4]==0x02) && (data[5]==0x58))
  {
    ROS_INFO("rpm is 600");
  }
  else if ((data[4]==0x01) && (data[5]==0x2C))
  {
    ROS_INFO("rpm is 300");
  } else{
    ROS_WARN("Invalid motor rpm!");
  }
}

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const lslidar_c16_msgs::LslidarC16Packet& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  float azimuth;  // 0.01 dgree
  float intensity;
  float azimuth_diff;
  float azimuth_corrected_f;
  int azimuth_corrected;

  const raw_packet_t* raw = (const raw_packet_t*)&pkt.data[0];

  for (int block = 0; block < BLOCKS_PER_PACKET; block++, this->block_num++)  // 1 packet:12 data blocks
  {
    if (UPPER_BANK != raw->blocks[block].header)
    {
      ROS_INFO_STREAM_THROTTLE(180, "skipping LSLIDAR DIFOP packet");
      ROS_INFO("distance=%d",1111);
      break;
    }
    azimuth = (float)(256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1);
    // ROS_INFO("azimuth=%0.2f",azimuth);

    if (2 == return_mode_){

      if (block < (BLOCKS_PER_PACKET - 2))  // 12
      {
        int azi1, azi2;
        azi1 = 256 * raw->blocks[block + 2].rotation_2 + raw->blocks[block + 1].rotation_1;
        azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
        azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
      }
      else
      {
        int azi1, azi2;
        azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
        azi2 = 256 * raw->blocks[block - 2].rotation_2 + raw->blocks[block - 1].rotation_1;
        azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
      }

    }
    else {

      if (block < (BLOCKS_PER_PACKET - 1))  // 12
      {
        int azi1, azi2;
        azi1 = 256 * raw->blocks[block + 1].rotation_2 + raw->blocks[block + 1].rotation_1;
        azi2 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
        azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
      }
      else
      {
        int azi1, azi2;
        azi1 = 256 * raw->blocks[block].rotation_2 + raw->blocks[block].rotation_1;
        azi2 = 256 * raw->blocks[block - 1].rotation_2 + raw->blocks[block - 1].rotation_1;
        azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);
      }
    }


    for (int firing = 0, k = 0; firing < LSC16_FIRINGS_PER_BLOCK; firing++)  // 2
    {
      for (int dsr = 0; dsr < LSC16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)  // 16   3
      {

        azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * LSC16_DSR_TOFFSET) + (firing * LSC16_FIRING_TOFFSET)) /
                                         LSC16_BLOCK_TDURATION);

        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;  // convert to integral value...
        //ROS_INFO("distance0=%x",raw->blocks[block].data[k]);
        //ROS_INFO("distance1=%x",raw->blocks[block].data[k+1]);
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];
        int distance = tmp.uint;
        //ROS_INFO("dsr = %d, distance=%d",dsr, distance);
        // read intensity
        intensity = raw->blocks[block].data[k + 2];

        float distance2 =distance* DISTANCE_RESOLUTION;
        //ROS_INFO("distance=%0.2f",distance2);
        //float arg_horiz = (float)azimuth_corrected / 18000.0f * M_PI;
        float arg_horiz = (float)azimuth_corrected_f / 100.0f;
        arg_horiz = arg_horiz > 360 ? (arg_horiz - 360) : arg_horiz;

        float cos_azimuth = cos_azimuth_table[azimuth_corrected];
        float sin_azimuth = sin_azimuth_table[azimuth_corrected];

        float arg_horiz_orginal = (arg_horiz - 14.68)*M_PI/180;
        float arg_vert = VERT_ANGLE[dsr];

        pcl::PointXYZI point;
        if (distance2 > max_distance_ || distance2 < min_distance_)
        {
            point.x = NAN;
            point.y = NAN;
            point.z = NAN;
            point.intensity = 0;
            pointcloud->at(2 * this->block_num + firing, dsr) = point;
        }
        else
        {
            if(cbMethod_){
                point.x = distance2 * cos_scan_altitude[dsr]  * cos_azimuth + R1_ * cos(arg_horiz_orginal);
                point.y = -distance2 * cos_scan_altitude[dsr]  * sin_azimuth - R1_ * sin(arg_horiz_orginal);
                point.z = distance2 * sin_scan_altitude[dsr];
            }else{
                point.x = distance2 * cos_scan_altitude[dsr]  * cos_azimuth;
                point.y = -distance2 * cos_scan_altitude[dsr]  * sin_azimuth;
                point.z = distance2 * sin_scan_altitude[dsr];
            }

            point.intensity = intensity;
            pointcloud->at(2 * this->block_num + firing, dsr) = point;
        }
      }
    }
  }
}

}  // namespace lslidar_c16_decoder
