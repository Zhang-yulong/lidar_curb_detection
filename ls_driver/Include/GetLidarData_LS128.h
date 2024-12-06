/******************************************************************************
 * This file is part of lslidar_sdk driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include "GetLidarData.h"
#include <atomic>
#include <ctime>
#include <deque>

#define DEG2RAD(x) ((x)*0.017453293)
static const int POINTS_PER_PACKET_SINGLE_ECHO = 1192;
static const int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;
static float g_fDistanceAcc_LS128 = 0.1 * 0.01 * 1;
static double g_min_range = 0.15;
static double g_max_range = 500;
static double g_angle_disable_min = 0.0;
static double g_angle_disable_max = 0.0;
static double cos30 = cos(DEG2RAD(30));
static double sin30 = sin(DEG2RAD(30));
static double cos60 = cos(DEG2RAD(60));
static double sin60 = sin(DEG2RAD(60));

class GetLidarData_LS128 : public GetLidarData 
{
public:
    GetLidarData_LS128();

    ~GetLidarData_LS128();

    void LidarRun() ;

    //bool setLidarSoureSelection(int StateValue) override;

    //bool setLidarRotateSpeed(int SpeedValue) override;

    //bool setLidarRotateState(int StateValue) override;

    //bool setLidarWorkState(int StateValue) override;

private:
    double prism_angle[4]{};
    double packet_end_time{};
    double current_packet_time{};
    double last_packet_time{};
    double g_fAngleAcc_V;
    bool is_add_frame_;
    double cos_mirror_angle[4]{};
    double sin_mirror_angle[4]{};
    bool is_get_difop_;
    bool use_gps_ts_;
    std::atomic<bool> isOneFrame_{false};
    UTC_Time m_UTC_Time_;
    double cos_table_[36000]{};
    double sin_table_[36000]{};

    std::deque<std::vector<MuchLidarData>> total_data_;

    int convertCoordinate(struct Firing lidardata, MuchLidarData &m_DataT);

    void lslidarLSPacketProcess(const LslidarLSPacket &msg);
};



