/*
 *	Copyright (C) 2018-2020 Robosense Authors
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  RSLIDAR 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw RSLIDAR LIDAR packets into useful
 *  formats.
 *
 */
#include "rawdata.h"

namespace rslidar_rawdata
{
RawData::RawData()
{
  this->is_init_angle_ = false;

  // lookup table init
  this->cos_lookup_table_.resize(36000);
  this->sin_lookup_table_.resize(36000);
  for (unsigned int i = 0; i < 36000; i++)
  {
    double rad = RS_TO_RADS(i / 100.0f);

    this->cos_lookup_table_[i] = std::cos(rad);
    this->sin_lookup_table_[i] = std::sin(rad);
  }
}

void RawData::loadConfigFile(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  std::string anglePath, channelPath;
  std::string model;
  std::string input_difop_packets_topic;

  private_nh.param("angle_path", anglePath, std::string(""));
  private_nh.param("channel_path", channelPath, std::string(""));
  private_nh.param("start_angle", start_angle_, float(0));
  private_nh.param("end_angle", end_angle_, float(360));
  private_nh.param("input_difop_packets_topic", input_difop_packets_topic, std::string("rslidar_packets_difop"));

  if (start_angle_ < 0.0f || start_angle_ > 360.0f || end_angle_ < 0.0f || end_angle_ > 360.0f)
  {
    start_angle_ = 0.0f;
    end_angle_ = 360.0f;
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

  start_angle_ = start_angle_ * 100;
  end_angle_ = end_angle_ * 100;

  private_nh.param("max_distance", max_distance_, 200.0f);
  private_nh.param("min_distance", min_distance_, 0.2f);

  ROS_INFO_STREAM("distance threshlod, max: " << max_distance_ << ", min: " << min_distance_);

  info_print_flag_ = false;

  private_nh.param("model", model, std::string("RS128"));
  if (model == "RS128")
  {
    numOfLasers = 128;
    TEMPERATURE_RANGE = 50;
    Rx_ = 0.03615f;
    Ry_ = -0.017f;
    Rz_ = 0.0f;
  }
  else
  {
    std::cout << "Bad model!" << std::endl;
  }

  // return mode default
  return_mode_ = 1;

  //=============================================================
  FILE* f_angle = fopen(anglePath.c_str(), "r");
  if (!f_angle)
  {
    ROS_ERROR_STREAM(anglePath << " does not exist");
  }
  else
  {
    float b[128], d[128];
    int loopk = 0;
    int loopn = 0;
    while (!feof(f_angle))
    {
      int tmp = fscanf(f_angle, "%f,%f\n", &b[loopk], &d[loopk]);
      loopk++;
      if (loopk > (numOfLasers - 1))
        break;
    }
    for (loopn = 0; loopn < numOfLasers; loopn++)
    {
      VERT_ANGLE[loopn] = (int)(b[loopn] * 100);
      HORI_ANGLE[loopn] = (int)(d[loopn] * 100);
      if (model == "RS16")
      {
        HORI_ANGLE[loopn] = 0.0f;
      }
    }
    fclose(f_angle);
  }


  //=============================================================
  FILE* f_channel = fopen(channelPath.c_str(), "r");
  if (!f_channel)
  {
    ROS_ERROR_STREAM(channelPath << " does not exist");
  }
  else
  {
    int loopl = 0;
    int loopm = 0;
    int c[51];
    int tempMode = 1;
    while (!feof(f_channel))
    { 
      int tmp = fscanf(f_channel,
                        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%"
                        "d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                        &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
                        &c[13], &c[14], &c[15], &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24],
                        &c[25], &c[26], &c[27], &c[28], &c[29], &c[30], &c[31], &c[32], &c[33], &c[34], &c[35], &c[36],
                        &c[37], &c[38], &c[39], &c[40], &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48],
                        &c[49], &c[50]);
      
      for (loopl = 0; loopl < TEMPERATURE_RANGE + 1; loopl++)
      {
        g_ChannelNum[loopm][loopl] = c[tempMode * loopl];
      }
      loopm++;
      if (loopm > (numOfLasers - 1))
      {
        break;
      }
    }
    fclose(f_channel);
  }

  // receive difop data
  // subscribe to difop rslidar packets, if not right correct data in difop, it will not revise the correct data in the
  // VERT_ANGLE, HORI_ANGLE etc.

  difop_sub_ = node.subscribe(input_difop_packets_topic, 10, &RawData::processDifop, (RawData*)this);

  temperature_pub_ = node.advertise<std_msgs::Float32>("temperature", 10);
}

void RawData::processDifop(const rslidar_msgs::rslidarPacket::ConstPtr& difop_msg)
{
  // std::cout << "Enter difop callback!" << std::endl;
  const uint8_t* data = &difop_msg->data[0];

  if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a)
  {
    // std::cout << "Enter processDifop, the correct one!" << std::endl;
    // return mode check
    if (data[300] == 0x01 || data[300] == 0x02)
    {
      return_mode_ = data[300];
    }
    else
    {
      return_mode_ = 0;
    }

    if (!this->is_init_angle_)
    {
      bool angle_flag = true;
      // check difop reigon has beed flashed the right data
      if ((data[468] == 0x00 || data[468] == 0xff) && (data[469] == 0x00 || data[469] == 0xff) &&
          (data[470] == 0x00 || data[470] == 0xff) && (data[471] == 0x00 || data[471] == 0xff))
      {
        angle_flag = false;
      }
      // angle
      if (angle_flag)
      {
        int bit1, bit2, bit3, symbolbit;
        for (int loopn = 0; loopn < 128; ++loopn)
        {
          // vertical angle
          bit1 = static_cast<int>(*(data + 468 + loopn * 3));
          bit2 = static_cast<int>(*(data + 468 + loopn * 3 + 1));
          bit3 = static_cast<int>(*(data + 468 + loopn * 3 + 2));
          if (bit1 == 0)
            symbolbit = 1;
          else if (bit1 == 1)
            symbolbit = -1;
          VERT_ANGLE[loopn] = (bit2 * 256 + bit3) * symbolbit; //do not convert to 1 degree
          // horizontal offset angle
          bit1 = static_cast<int>(*(data + 852 + loopn * 3));  
          bit2 = static_cast<int>(*(data + 852 + loopn * 3 + 1));
          bit3 = static_cast<int>(*(data + 852 + loopn * 3 + 2));
          if (bit1 == 0) 
            symbolbit = 1;
          else if (bit1 == 1)
            symbolbit = -1;
          HORI_ANGLE[loopn] = (bit2 * 256 + bit3) * symbolbit; //do not convert to 1 degree
        }
      }
      this->is_init_angle_ = true;
      ROS_INFO_STREAM("read angle from difop finish!");
    }

  }
  else
  {
    return;
  }
  
}

float RawData::pixelToDistance(int pixel_value, int passageway)
{
  int input_temp_index = estimateTemperature(temper) - TEMPERATURE_MIN;
  float delta_dist = g_ChannelNum[passageway][input_temp_index];

  float esti_distance = 0.0f;
  if (pixel_value < delta_dist)
  {
    esti_distance = 0.0f;
  }
  else
  {
    esti_distance = (float)(pixel_value - delta_dist);
  }
  return esti_distance;
}

unsigned int RawData::correctAzimuth(float azimuth_f, int passageway)
{
  unsigned int azimuth;
  if (azimuth_f > 0.0f && azimuth_f < 3000.0f)
  {
    azimuth_f = azimuth_f + HORI_ANGLE[passageway] + 36000.0f;
  }
  else
  {
    azimuth_f = azimuth_f + HORI_ANGLE[passageway];
  }

  if (azimuth_f < 0)
  {
    azimuth_f += 36000.0f;
  }
  azimuth = (unsigned int)azimuth_f;
  azimuth %= 36000;

  return azimuth;
}


float RawData::computeTemperature128(unsigned char bit2, unsigned char bit1)
{
  float Temp;
  float bitneg = bit2 & 128;   // 10000000
  float highbit = bit2 & 127;  // 01111111
  float lowbit = bit1 >> 4;
  if (bitneg == 128)
  {
    Temp = -1.0f * (highbit * 16 + lowbit) * 0.0625f;
  }
  else
  {
    Temp = (highbit * 16.0f + lowbit) * 0.0625f;
  }

  return Temp;
}

//------------------------------------------------------------
int RawData::estimateTemperature(float Temper)
{
  int temp = int(Temper + 0.5);
  if (temp < TEMPERATURE_MIN)
  {
    temp = TEMPERATURE_MIN;
  }
  else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE)
  {
    temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
  }

  return temp;
}
//------------------------------------------------------------

/** @brief convert raw packet to point cloud
 *
 *  @param pkt raw packet to unpack
 *  @param pc shared pointer to point cloud (points are appended)
 */
void RawData::unpack(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  if (numOfLasers == 128)
  {
    unpack_RS128(pkt, pointcloud);
    return;
  }

}

void RawData::unpack_RS128(const rslidar_msgs::rslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  float curAzimuth;  // 0.01 dgree
  float intensity;
  float azimuth_corrected_f;
  unsigned int azimuth_corrected;

  const raw_packet_rs128* raw = (const raw_packet_rs128*)&pkt.data[80];

  // update temperature information per 20000 packets
  if (tempPacketNum < 600 && tempPacketNum > 0)
  {
    tempPacketNum++;
  }
  else
  {
    temper = computeTemperature128(pkt.data[8], pkt.data[9]);
    // ROS_INFO_STREAM("Temp is: " << temper);
    tempPacketNum = 1;

    std_msgs::Float32 temperature_msgs;
    temperature_msgs.data = temper;
    temperature_pub_.publish(temperature_msgs);
  }

  // process block
  for (int block = 0; block < BLOCKS_PER_PACKET_RS128; block++, this->block_num++)  // 1 packet:3 data blocks
  {
    // calculte current azimuth
    curAzimuth = (float)(256 * raw->blocks[block].rotation_1 + raw->blocks[block].rotation_2);
    
    // get the real azimuth diff
    if ((lastAzimuth != -1) && (lastAzimuth != curAzimuth))
    {
      azimuthDiff = (float)((int)(36000 + curAzimuth - lastAzimuth) % 36000);
    }
    
    // Ingnore the block if the azimuth change abnormal
    if (azimuthDiff <= 0.0 || azimuthDiff > 40)
    {
      continue;
    }

    // process firing data
    for (int dsr = 0; dsr < 128; dsr++)
    {
      //dsr_temp
      int dsr_temp;
      dsr_temp = dsr / 4;
      dsr_temp = dsr_temp % 16;

      // azimuth
      azimuth_corrected_f = curAzimuth + (azimuthDiff * (dsr_temp * RS128_DSR_TOFFSET) / RS128_BLOCK_TDURATION);
      azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);

      // distance
      union two_bytes tmp;
      tmp.bytes[1] = raw->blocks[block].data[dsr * 3];
      tmp.bytes[0] = raw->blocks[block].data[dsr * 3 + 1];
      int distance = tmp.uint;

      // calculate intensity
      intensity = (float)raw->blocks[block].data[dsr * 3 + 2];

      // calculate distance
      float distance2 = pixelToDistance(distance, dsr);
      distance2 = distance2 * DISTANCE_RESOLUTION;

      unsigned int arg_horiz_orginal = (unsigned int)(round(azimuth_corrected_f)) % 36000;
      unsigned int arg_horiz = azimuth_corrected;
      unsigned int arg_vert = ((VERT_ANGLE[dsr]) % 36000 + 36000) % 36000;
      pcl::PointXYZI point;

      if (distance2 > max_distance_ || distance2 < min_distance_ ||
          (angle_flag_ && (arg_horiz < start_angle_ || arg_horiz > end_angle_)) ||
          (!angle_flag_ && (arg_horiz > end_angle_ && arg_horiz < start_angle_)))  // invalid distance
      {
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
        point.intensity = 0.0f;
        pointcloud->at(this->block_num, dsr) = point;
      }
      else
      {
        // If you want to fix the rslidar X aixs to the front side of the cable, please use below
        point.x = distance2 * this->cos_lookup_table_[arg_vert] * this->cos_lookup_table_[arg_horiz] + 
                  Rx_ * this->cos_lookup_table_[arg_horiz_orginal];
        point.y = -distance2 * this->cos_lookup_table_[arg_vert] * this->sin_lookup_table_[arg_horiz] - 
                  Rx_ * this->sin_lookup_table_[arg_horiz_orginal];
        point.z = distance2 * this->sin_lookup_table_[arg_vert] + Rz_;
        point.intensity = intensity;
        pointcloud->at(this->block_num, dsr) = point;
      }
    }
    // after process firing data, update lastAzimuth
    lastAzimuth = curAzimuth;
  }
}

}  // namespace rslidar_rawdata
