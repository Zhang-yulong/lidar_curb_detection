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

#include "GetLidarData_LS128.h"


GetLidarData_LS128::GetLidarData_LS128()
{
    for (int i = 0; i < 4; i++) {
        prism_angle[i] = 0.0;
    }

    double mirror_angle[4] = {0, -2, -1, -3};   //摆镜角度   //根据通道不同偏移角度不同
    for (int i = 0; i < 4; ++i) {
        cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
        sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
    }

    all_lidardata.clear();
    all_lidardata_d.clear();
    current_packet_time = 0.0;
    last_packet_time = 0.0;
    Model = 1;
    g_fAngleAcc_V = 0.01;
    is_add_frame_ = false;
    is_get_difop_ = false;
    use_gps_ts_ = true;
    for (int j = 0; j < 36000; ++j) {
        double angle = static_cast<double>(j) / 100.0 * M_PI / 180.0;
        sin_table_[j] = sin(angle);
        cos_table_[j] = cos(angle);
    }
    //std::cout << "lidar_type: " << __func__ << std::endl;
}

GetLidarData_LS128::~GetLidarData_LS128()
{

}

void GetLidarData_LS128::LidarRun() 
{
    while (true) 
    {
        if (isQuit) 
        {
            clearQueue(allDataValue);
            break;
        }
        if (!allDataValue.empty()) {
            unsigned char data[1206] = {0};
            m_mutex.lock();
            memcpy(data, allDataValue.front(), 1206);
            delete allDataValue.front();
            allDataValue.pop();
            m_mutex.unlock();

            uint64_t allTmpTimestamp;
            double PointIntervalT_ns;
            if ((0x00 == data[0] || 0xa5 == data[0]) && 0xff == data[1] &&
                0x00 == data[2] && 0x5a == data[3]) {
                //231字节 ==64 叠加两帧
                if (data[231] == 64 || data[231] == 65) {
                    is_add_frame_ = true;
                }
                int majorVersion = data[1202];
                int minorVersion1 = data[1203] / 16;
                int minorVersion2 = data[1203] % 16;

                //v1.1 :0.01   //after v1.2  ： 0.0025
                if (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) {
                    g_fAngleAcc_V = 0.0025;
                } else {
                    g_fAngleAcc_V = 0.01;
                }

                float fInitAngle_V = data[188] * 256 + data[189];
                if (fInitAngle_V > 32767) {
                    fInitAngle_V = fInitAngle_V - 65536;
                }
                this->prism_angle[0] = fInitAngle_V * g_fAngleAcc_V;

                fInitAngle_V = data[190] * 256 + data[191];
                if (fInitAngle_V > 32767) {
                    fInitAngle_V = fInitAngle_V - 65536;
                }
                this->prism_angle[1] = fInitAngle_V * g_fAngleAcc_V;

                fInitAngle_V = data[192] * 256 + data[193];
                if (fInitAngle_V > 32767) {
                    fInitAngle_V = fInitAngle_V - 65536;
                }
                this->prism_angle[2] = fInitAngle_V * g_fAngleAcc_V;

                fInitAngle_V = data[194] * 256 + data[195];
                if (fInitAngle_V > 32767) {
                    fInitAngle_V = fInitAngle_V - 65536;
                }
                this->prism_angle[3] = fInitAngle_V * g_fAngleAcc_V;
                is_get_difop_ = true;
                continue;
            }
            if (!is_get_difop_) continue;
            if (use_gps_ts_) {
                if (0xff == data[1194]) {    //ptp
                    uint64_t timestamp_s = ((data[1195] * 0) + (data[1196] << 24) + (data[1197] << 16) +
                                            (data[1198] << 8) + data[1199]);
                    uint64_t timestamp_nsce = ((data[1200] << 24) + (data[1201] << 16) + (data[1202] << 8) +
                                                data[1203]);
                    allTmpTimestamp = timestamp_s * pow(10, 9) + timestamp_nsce;
                } else {     //gps
                    m_UTC_Time_.year = data[1194] + 2000 - 1900;
                    m_UTC_Time_.month = data[1195] - 1;
                    m_UTC_Time_.day = data[1196];
                    m_UTC_Time_.hour = data[1197];
                    m_UTC_Time_.minute = data[1198];
                    m_UTC_Time_.second = data[1199];

                    uint64_t timestamp_nsce = ((data[1200] << 24) + (data[1201] << 16) + (data[1202] << 8) +
                                                data[1203]);
                    struct tm t{};
                    t.tm_sec = m_UTC_Time_.second;
                    t.tm_min = m_UTC_Time_.minute;
                    t.tm_hour = m_UTC_Time_.hour;
                    t.tm_mday = m_UTC_Time_.day;
                    t.tm_mon = m_UTC_Time_.month;
                    t.tm_year = m_UTC_Time_.year;
                    t.tm_isdst = 0;
                    //auto timestamp_s = mktime(&t);
                    auto timestamp_s = static_cast<uint64_t>(timegm(&t)); //s
                    allTmpTimestamp = timestamp_s * pow(10, 9) + timestamp_nsce;
                }
            } else {    //system time
                auto time_now = std::chrono::system_clock::now().time_since_epoch().count();
                allTmpTimestamp = time_now;
            }


            LslidarLSPacket packet;
            packet.prism_angle[0] = this->prism_angle[0];
            packet.prism_angle[1] = this->prism_angle[1];
            packet.prism_angle[2] = this->prism_angle[2];
            packet.prism_angle[3] = this->prism_angle[3];

            packet.stamp = allTmpTimestamp;
            allTimestamp = allTmpTimestamp;
            current_packet_time = allTmpTimestamp;
            memcpy(packet.data, data, 1206);

            lslidarLSPacketProcess(packet);
            //Test whether the lidar service time is normal
#ifdef DEBUG
            if(allTimestamp - lastAllTimestamp < 0.0) {
                printf("system time= %.9f, lidar time fallback! diff= %.9f , current packet time= %.9f, last packet time= %.9f\n",
                    std::chrono::system_clock::now().time_since_epoch().count()/1e9, (allTimestamp - lastAllTimestamp)/1e9,allTimestamp/1e9, lastAllTimestamp/1e9);
            } else if (allTimestamp - lastAllTimestamp > 0.01 * 1e9) {
                printf("system time= %.9f, lidar time forward! diff= %.9f , current packet time= %.9f, last packet time= %.9f\n",
                    std::chrono::system_clock::now().time_since_epoch().count()/1e9, (allTimestamp - lastAllTimestamp)/1e9,allTimestamp/1e9, lastAllTimestamp/1e9);
            }
#endif
            lastAllTimestamp = allTimestamp;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

int GetLidarData_LS128::convertCoordinate(struct Firing lidardata, MuchLidarData &m_DataT) {
    if (lidardata.distance * g_fDistanceAcc_LS128 < g_min_range || lidardata.distance * g_fDistanceAcc_LS128 > g_max_range) {
        return -1;
    }
    if ((lidardata.azimuth > g_angle_disable_min) && (lidardata.azimuth < g_angle_disable_max)) {
        return -1;
    }

    double fAngle_H = 0.0;         //Horizontal angle
    double fAngle_V = 0.0;        // Vertical angle
    fAngle_H = lidardata.azimuth;
    fAngle_V = lidardata.vertical_angle;

    double fSinV_angle = 0;
    double fCosV_angle = 0;

    double fGalvanometrtAngle = 0;
    //fGalvanometrtAngle = (((fAngle_V + 0.05) / 0.8) + 1) * 0.46 + 6.72;
    fGalvanometrtAngle = fAngle_V + 7.26;     //note: 7.26 Offset rotation for vertical angle

    while (fGalvanometrtAngle < 0.0) {
        fGalvanometrtAngle += 360.0;
    }
    while (fAngle_H < 0.0) {
        fAngle_H += 360.0;
    }

    int table_index_V = int(fGalvanometrtAngle * 100) % 36000;
    int table_index_H = int(fAngle_H * 100) % 36000;

    double fAngle_R0 = cos30 * cos_mirror_angle[lidardata.channel_number % 4] * cos_table_[table_index_V] -
                        sin_table_[table_index_V] * sin_mirror_angle[lidardata.channel_number % 4];

    fSinV_angle = 2 * fAngle_R0 * sin_table_[table_index_V] + sin_mirror_angle[lidardata.channel_number % 4];
    fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

    double fSinCite = (2 * fAngle_R0 * cos_table_[table_index_V] * sin30 -
                        cos_mirror_angle[lidardata.channel_number % 4] * sin60) / fCosV_angle;
    double fCosCite = sqrt(1 - pow(fSinCite, 2));

    double fSinCite_H = sin_table_[table_index_H] * fCosCite + cos_table_[table_index_H] * fSinCite;
    double fCosCite_H = cos_table_[table_index_H] * fCosCite - sin_table_[table_index_H] * fSinCite;

    double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
    x_coord = (lidardata.distance * fCosV_angle * fSinCite_H) * g_fDistanceAcc_LS128;
    y_coord = (lidardata.distance * fCosV_angle * fCosCite_H) * g_fDistanceAcc_LS128;
    z_coord = (lidardata.distance * fSinV_angle) * g_fDistanceAcc_LS128;

    //MuchLidarData m_DataT;
    m_DataT.X = x_coord;
    m_DataT.Y = y_coord;
    m_DataT.Z = z_coord;
    m_DataT.ID = lidardata.channel_number;
    m_DataT.H_angle = lidardata.azimuth;
    m_DataT.V_angle = lidardata.vertical_angle;
    m_DataT.Distance = lidardata.distance * g_fDistanceAcc_LS128;
    m_DataT.Intensity = lidardata.intensity;
    m_DataT.Mtimestamp_nsce = lidardata.time;
if(m_DataT.Y < 1.8 || fabs(m_DataT.X) > 60 || fabs(m_DataT.Y) > 60 || m_DataT.Z > 2.5)
{
    return 0;
}
    all_lidardata.push_back(m_DataT);
    return 0;
}

void GetLidarData_LS128::lslidarLSPacketProcess(
        const LslidarLSPacket &msg) {
    isOneFrame_ = false;
    struct Firing lidardata{};
    // Convert the msg to the raw packet type.
    packet_end_time = msg.stamp;
    bool packetType = false;
    static int s_iPacketNum = 0;
    if (msg.data[1205] == 0x02) {
        Model = 2;
    }
    s_iPacketNum++;
    //printf("s_iPacketNum = %d, Model = %d\n", s_iPacketNum, Model);
    if (Model == 1) 
    {
        double packet_interval_time =
                (current_packet_time - last_packet_time) / (POINTS_PER_PACKET_SINGLE_ECHO / 8.0);
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8) 
        {
            if ((msg.data[point_idx] == 0xff) && (msg.data[point_idx + 1] == 0xaa) &&
                (msg.data[point_idx + 2] == 0xbb) && (msg.data[point_idx + 3] == 0xcc) &&
                (msg.data[point_idx + 4] == 0xdd)) 
            {
                packetType = true;
                isOneFrame_ = true;
                // printf("s_iPacketNum = %d, Model = %d\n", s_iPacketNum, Model);
                s_iPacketNum = 0;
            } 
            else 
            {
                // Compute the time of the point
                double point_time;
                if (last_packet_time > 1e-6) 
                {
                    point_time =
                            packet_end_time -
                            packet_interval_time * ((POINTS_PER_PACKET_SINGLE_ECHO - point_idx) / 8 - 1);
                } 
                else 
                {
                    point_time = current_packet_time;
                }

//                if (msg.data[point_idx] < 255) {
                memset(&lidardata, 0, sizeof(lidardata));
                double fAngle_H = msg.data[point_idx + 1] + (msg.data[point_idx] << 8);
                if (fAngle_H > 32767) 
                {
                    fAngle_H = (fAngle_H - 65536);
                }
                lidardata.azimuth = fAngle_H * 0.01;
                //Vertical angle+channel number
                int iTempAngle = msg.data[point_idx + 2];
                int iChannelNumber = iTempAngle >> 6;
                int iSymmbol = (iTempAngle >> 5) & 0x01;
                double fAngle_V = 0.0;
                if (1 == iSymmbol) //Sign bit 0: positive number 1: negative number
                {
                    int iAngle_V = msg.data[point_idx + 3] + (msg.data[point_idx + 2] << 8);

                    fAngle_V = iAngle_V | 0xc000;
                    if (fAngle_V > 32767) 
                    {
                        fAngle_V = (fAngle_V - 65536);
                    }
                } 
                else 
                {
                    int iAngle_Hight = iTempAngle & 0x3f;
                    fAngle_V = msg.data[point_idx + 3] + (iAngle_Hight << 8);
                }
                // Prism angle
                for (int i = 0; i < 4; ++i) 
                {
                    lidardata.prism_angle[i] = msg.prism_angle[i];
                }

                lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance = ((msg.data[point_idx + 4] << 16) + (msg.data[point_idx + 5] << 8) +
                                        msg.data[point_idx + 6]);
                lidardata.intensity = msg.data[point_idx + 7];
                lidardata.time = point_time;
                lidardata.azimuth = fAngle_H * 0.01;
				MuchLidarData m_DataT;
                convertCoordinate(lidardata, m_DataT);
				
//                }
            }
            if (packetType) 
            {
                static int count = 0;
                static double total_time = 0.0;
                if (is_add_frame_) 
                {
                    if (total_data_.size() >= 2) 
                    {
                        total_data_.pop_front();
                    }
                    if (total_data_.size() < 2) 
                    {
                        total_data_.push_back(all_lidardata);
                    }
                    if (total_data_.size() == 2) 
                    {
                        count++;
//                            time_t time1 = clock();
//                            all_lidardata.clear();
                        all_lidardata.insert(all_lidardata.begin(), total_data_[0].begin(), total_data_[0].end());
//                            all_lidardata.insert(all_lidardata.end(), total_data_[1].begin(), total_data_[1].end());
//                            time_t time2 = clock();
//                            std::cout <<"cost time: " << double (time2-time1)/CLOCKS_PER_SEC << std::endl;
                        sendLidarData();
//                            total_time += double(time2 - time1) / CLOCKS_PER_SEC;
//                            if (count % 100 == 1)
//                                std::cout << "************************avg  time: " << total_time / count << std::endl;
                    }
                } 
                else 
                {
                    sendLidarData();
                }
                packetType = false;
            }
        }
    } else {
        double packet_interval_time =
                (current_packet_time - last_packet_time) / (POINTS_PER_PACKET_DOUBLE_ECHO / 12.0);
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12) {
            if ((msg.data[point_idx] == 0xff) && (msg.data[point_idx + 1] == 0xaa) &&
                (msg.data[point_idx + 2] == 0xbb) && (msg.data[point_idx + 3] == 0xcc) &&
                (msg.data[point_idx + 4] == 0xdd)) {
                packetType = true;
                isOneFrame_ = true;
                
            } else {
                // Compute the time of the point
                double point_time;
                if (last_packet_time > 1e-6) {
                    point_time =
                            packet_end_time -
                            packet_interval_time * ((POINTS_PER_PACKET_SINGLE_ECHO - point_idx) / 12 - 1);
                } else {
                    point_time = current_packet_time;
                }
//                if (msg.data[point_idx] < 255) {
                memset(&lidardata, 0, sizeof(lidardata));
                double fAngle_H = msg.data[point_idx + 1] + (msg.data[point_idx] << 8);
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                lidardata.azimuth = fAngle_H * 0.01;

                int iTempAngle = msg.data[point_idx + 2];
                int iChannelNumber = iTempAngle >> 6;
                int iSymmbol = (iTempAngle >> 5) & 0x01;
                double fAngle_V = 0.0;
                if (1 == iSymmbol) {
                    int iAngle_V = msg.data[point_idx + 3] + (msg.data[point_idx + 2] << 8);

                    fAngle_V = iAngle_V | 0xc000;
                    if (fAngle_V > 32767) {
                        fAngle_V = (fAngle_V - 65536);
                    }
                } else {
                    int iAngle_Hight = iTempAngle & 0x3f;
                    fAngle_V = msg.data[point_idx + 3] + (iAngle_Hight << 8);
                }

                for (int i = 0; i < 4; ++i) {
                    lidardata.prism_angle[i] = msg.prism_angle[i];
                }

                lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance = ((msg.data[point_idx + 4] << 16) + (msg.data[point_idx + 5] << 8) +
                                        msg.data[point_idx + 6]);
                lidardata.intensity = msg.data[point_idx + 7];
                lidardata.time = point_time;
				MuchLidarData m_DataT;
                convertCoordinate(lidardata, m_DataT);  //First point

                lidardata.distance = ((msg.data[point_idx + 8] << 16) + (msg.data[point_idx + 9] << 8) +
                                        msg.data[point_idx + 10]);
                lidardata.intensity = msg.data[point_idx + 11];
                lidardata.time = point_time;
                convertCoordinate(lidardata, m_DataT);  //Second point
//                }
            }

            if (packetType) {
                static int count = 0;
                static double total_time = 0.0;
                if (is_add_frame_) {
                    if (total_data_.size() >= 2) {
                        total_data_.pop_front();
                    }
                    if (total_data_.size() < 2) {
                        total_data_.push_back(all_lidardata);
                    }
                    if (total_data_.size() == 2) {
                        Model = 1;
                        count++;
//                            time_t time1 = clock();
//                            all_lidardata.clear();
                        all_lidardata.insert(all_lidardata.begin(), total_data_[0].begin(), total_data_[0].end());
//                            all_lidardata.insert(all_lidardata.end(), total_data_[1].begin(), total_data_[1].end());
//                            time_t time2 = clock();
//                            std::cout <<"cost time: " << double (time2-time1)/CLOCKS_PER_SEC << std::endl;
                        sendLidarData();
//                            total_time += double(time2 - time1) / CLOCKS_PER_SEC;
//                            if (count % 100 == 1)
//                                std::cout << "************************avg  time: " << total_time / count << std::endl;
                    }
                } else {
                    sendLidarData();
                }
                packetType = false;
            }
        }
    }
    last_packet_time = current_packet_time;
}
#if 0
bool GetLidarData_LS128::setLidarSoureSelection(int StateValue) {
    if (setLidarParam()) {
        if (StateValue != 0 && StateValue != 1) {
            std::string str = "StateValue can only be equal to 0 or 1, please check the input parameters";
            messFunction(str, 0);
            return false;
        }
        Rest_UCWP_buff_[44] = StateValue;
        return true;
    } else {
        std::string str = "Equipment package is not update!!!";
        messFunction(str, 0);
        return false;
    }
}

bool GetLidarData_LS128::setLidarRotateSpeed(int SpeedValue) {
    std::string str = "This version of lidar does not support setLidarRotateSpeed function";
    messFunction(str, 0);
    return false;
}

bool GetLidarData_LS128::setLidarRotateState(int StateValue) {
    std::string str = "This version of lidar does not support setLidarRotateSpeed function";
    messFunction(str, 0);
    return false;
}

bool GetLidarData_LS128::setLidarWorkState(int StateValue) {
    std::string str = "This version of lidar does not support setLidarRotateSpeed function";
    messFunction(str, 0);
    return false;
}
#endif
