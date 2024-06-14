﻿/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, EAIBOT, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include "CYdLidar.h"
#include "core/common/ydlidar_help.h"
#include "filters/NoiseFilter.h"
#include "filters/StrongLightFilter.h"

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif


int main(int argc, char *argv[])
{
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);
  std::string port;
  ydlidar::os_init();

  // 让用户选择IP或者手动输入IP
  {
    // 命令TCP 192.168.0.11 8090
    // 点云UDP 8000
    // 广播UDP 7777
    std::map<std::string, std::string> ports;
    std::map<std::string, std::string>::iterator it;
    ports["IP1"] = "192.168.0.11";
    ports["IP2"] = "Manual input IP";
    int id = 0;
    for (it = ports.begin(); it != ports.end(); ++it)
    {
      printf("[%d] %s %s\n", id, it->first.c_str(), it->second.c_str());
      id++;
    }

    while (ydlidar::os_isOk())
    {
      printf("Please select the lidar port index: ");
      std::string number;
      std::cin >> number;

      if ((size_t)atoi(number.c_str()) >= ports.size())
        continue;

      it = ports.begin();
      id = atoi(number.c_str());
      while (id)
      {
        id--;
        it++;
      }
      port = it->second;
      break;
    }

    if (port == ports["IP2"])
    {
      printf("Please enter the lidar IP: ");
      std::cin >> port;
    }

  //串口
  // ports = ydlidar::lidarPortList();
  // if (ports.size() == 1)
  // {
  //   port = ports.begin()->second;
  // }
  // else
  // {
  //   int id = 0;

  //   for (it = ports.begin(); it != ports.end(); it++)
  //   {
  //     printf("[%d] %s %s\n", id, it->first.c_str(), it->second.c_str());
  //     id++;
  //   }

  //   if (ports.empty())
  //   {
  //     printf("Not Lidar was detected. Please enter the lidar serial port:");
  //     std::cin >> port;
  //   }
  //   else
  //   {
  //     while (ydlidar::os_isOk())
  //     {
  //       printf("Please select the lidar port:");
  //       std::string number;
  //       std::cin >> number;

  //       if ((size_t)atoi(number.c_str()) >= ports.size())
  //       {
  //         continue;
  //       }

  //       it = ports.begin();
  //       id = atoi(number.c_str());

  //       while (id)
  //       {
  //         id--;
  //         it++;
  //       }

  //       port = it->second;
  //       break;
  //     }
  //   }
  // }

  }

  // int baudrate = 512000; //8090
  int baudrate = 8090;

  bool isSingleChannel = false;

  float frequency = 20.0f;

  // std::string input_frequency;
  // while (ydlidar::os_isOk() && !isSingleChannel)
  // {
  //   printf("Please input the lidar scan frequency[10-30]: ");
  //   std::cin >> input_frequency;
  //   frequency = atof(input_frequency.c_str());
  //   if (frequency <= 30.0 && frequency >= 10.0)
  //   {
  //     break;
  //   }
  //   fprintf(stderr, "Invalid scan frequency Please re-input.\n");
  // }

  /// instance
  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  /// tof lidar
  int optval = TYPE_TOF;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  // optval = YDLIDAR_TYPE_SERIAL;
  optval = YDLIDAR_TYPE_TCP;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 20;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
  //  optval = 16;
  //  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = false;
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = isSingleChannel;
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = true;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  /// unit: m
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  // laser.setEnableDebug(true); //启用调试

  /// initialize SDK and LiDAR.
  bool ret = laser.initialize();

  if (ret)
  { // success
    /// Start the device scanning routine which runs on a separate thread and enable motor.
    ret = laser.turnOn();
  }
  else
  { // failed
    fprintf(stderr, "%s\n", laser.DescribeError());
    fflush(stderr);
  }

  LaserScan scan;
  LaserScan outScan;
  StrongLightFilter filter; //拖尾滤波器
  filter.setMaxDist(0.1); //最大距离阈值
  filter.setMaxAngle(12.0); //最大角度阈值
  filter.setMinNoise(2); //最小连续噪点数

  float minScanTime = 1.0f / (frequency + 2.5);
  float maxScanTime = 1.0f / (frequency - 2.5);
  while (ret && ydlidar::os_isOk())
  {
    //循环获取点云数据
    if (laser.doProcessSimple(scan))
    {
      // if (scan.config.scan_time < minScanTime ||
      //   scan.config.scan_time > maxScanTime)
      // {
      //   core::common::error("[%u] points ScanTime [%f]s IncrTime [%f]s",
      //     uint32_t(scan.points.size()),
      //     scan.config.scan_time,
      //     scan.config.time_increment);
      // }
      
      // core::common::info("[%u] points Stamp [%u]ms",
      //   uint32_t(scan.points.size()),
      //   uint32_t(scan.stamp / 1000000));

      // for (size_t i = 0; i < scan.points.size(); ++i)
      // {
      //   const LaserPoint &p = scan.points.at(i);
      //   printf("%d d %.0f a %.02f i %.0f\n", i, p.range, p.angle * 180.0 / M_PI, p.intensity);
      // }
      // fflush(stdout);

      // 使用强光滤波器
      // filter.filter(scan, 0, 0, outScan);
    }
    else
    {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  /// Stop the device scanning thread and disable motor.
  laser.turnOff();
  /// Uninitialize the SDK and Disconnect the LiDAR.
  laser.disconnecting();

  return 0;
}
