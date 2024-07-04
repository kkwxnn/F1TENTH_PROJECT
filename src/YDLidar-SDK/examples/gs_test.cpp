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
#include <core/base/timer.h>
#include "CYdLidar.h"
#include "core/common/ydlidar_help.h"


using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

/**
 * @brief gs test
 * @param argc
 * @param argv
 * @return
 * @par Flow chart
 * Step1: instance CYdLidar.\n
 * Step2: set paramters.\n
 * Step3: initialize SDK and LiDAR.(::CYdLidar::initialize)\n
 * Step4: Start the device scanning routine which runs on a separate thread and enable motor.(::CYdLidar::turnOn)\n
 * Step5: Get the LiDAR Scan Data.(::CYdLidar::doProcessSimple)\n
 * Step6: Stop the device scanning thread and disable motor.(::CYdLidar::turnOff)\n
 * Step7: Uninitialize the SDK and Disconnect the LiDAR.(::CYdLidar::disconnecting)\n
 */

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

  std::map<std::string, std::string> ports = ydlidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  ports["IP1"] = "192.168.1.200";

  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      printf("[%d] %s %s\n", id, it->first.c_str(), it->second.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::os_isOk()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  int baudrate = 921600;
  std::map<int, int> baudrateList;
  baudrateList[0] = 8000; //网络端口
  baudrateList[1] = 921600; //串口波特率
  printf("Baudrate:\n");
  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++) {
    printf("[%d] %d\n", it->first, it->second);
  }

  while (ydlidar::os_isOk()) 
  {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) > baudrateList.size()) {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::os_isOk()) {
    return 0;
  }

  bool isSingleChannel = false;
  float frequency = 8.0;

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
  /// gs lidar
  int optval = TYPE_GS;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type (YDLIDAR_TYPE_TCP,YDLIDAR_TYPE_SERIAL)
  optval = baudrate == baudrateList[0] ? YDLIDAR_TYPE_TCP : YDLIDAR_TYPE_SERIAL; 
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = isSingleChannel ? 3 : 4;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
  /// Intenstiy bit count
  optval = 8;
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
  /// intensity
  b_optvalue = true;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = true;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  /// HeartBeat
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  // unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  // unit: m
  f_optvalue = 1.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.025f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  // unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  //是否启用调试
  laser.setEnableDebug(false); 

  //雷达初始化
  bool ret = laser.initialize();
  if (!ret)
  {
    fprintf(stderr, "Fail to initialize %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }
  //设置雷达工作模式（0表示避障模式，1表示沿边模式）
  ret &= laser.setWorkMode(1, 0x01);
  // ret &= laser.setWorkMode(0, 0x02);
  // ret &= laser.setWorkMode(1, 0x04);
  if (!ret)
  {
    fprintf(stderr, "Fail to set work mode %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }
  //获取级联雷达设备信息
  // std::vector<device_info_ex> dis;
  // ret = laser.getDeviceInfo(dis);
  // if (!ret)
  // {
  //   fprintf(stderr, "Fail to get Device infomations %s\n", laser.DescribeError());
  //   fflush(stderr);
  //   return -1;
  // }
  // for (int i=0; i<dis.size(); ++i)
  // {
  //   const device_info_ex& di = dis.at(i);
  //   printf("Device [%u]\n", di.id);
  //   ydlidar::core::common::printfDeviceInfo(di.di, EPT_Module);
  // }

  //启动扫描
  ret = laser.turnOn();
  if (!ret)
  {
    fprintf(stderr, "Fail to turn on %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }

  LaserScan scan;
  //打印帧间隔相关
  std::map<int, uint32_t> ts;
  ts[0] = getms();
  ts[1] = getms();
  ts[2] = getms();

  while (ret && ydlidar::os_isOk())
  {
    if (laser.doProcessSimple(scan))
    {
      printf("Module [%d] [%d] points in [%.02f]Hz\n",
        scan.moduleNum,
        int(scan.points.size()),
        scan.scanFreq);
      
      //打印帧间隔
      // uint32_t t = getms();
      // printf("module[%d] time[%lld]\n", scan.moduleNum, t - ts[scan.moduleNum]);
      // ts[scan.moduleNum] = t;
      
      //打印点云
      // for (size_t i = 0; i < scan.points.size(); ++i)
      // {
      //   const LaserPoint &p = scan.points.at(i);
      //   printf("%d a %.01f r %.01f\n", int(i), p.angle * 180.0f / M_PI, p.range * 1000.0f);
      // }
      // fflush(stdout);
    }
    else
    {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
