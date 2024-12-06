#include "GetLidarData_CH128x1.h"

double BigAngle[32] = {
	-17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5,
	-4.125, -4, -3.125, -3, -2.125, -2, -1.125, -1, -0.125, 0, 0.875,
	1, 1.875, 2, 3, 4, 5, 6, 7
};

GetLidarData_CH128x1::GetLidarData_CH128x1() {


    for (int i = 0; i < 128; i++) {
        sinTheta_1[i] = sin(BigAngle[i / 4] * PI / 180);
        sinTheta_2[i] = sin((i % 4) * (-0.17) * PI / 180);

        cosTheta_1[i] = cos(BigAngle[i / 4] * PI / 180);
        cosTheta_2[i] = cos((i % 4) * (-0.17) * PI / 180);
    }

    all_lidardata.clear();
    all_lidardata_d.clear();
    Model = 0;
    count = 0;
}

GetLidarData_CH128x1::~GetLidarData_CH128x1() {
}

void GetLidarData_CH128x1::LidarRun() {
    while (true) {
        if (isQuit) {
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

            if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a) {
                if (data[44] == 0x00) {   //gps授时
                    time_service_mode = "gps";
                } else if (data[44] == 0x01) { //ptp授时
                    time_service_mode = "gptp";
                }

				m_UTC_Time.year = data[52] + 2000;
				m_UTC_Time.month = data[53];
				m_UTC_Time.day = data[54];
				m_UTC_Time.hour = data[55];
				m_UTC_Time.minute = data[56];
				m_UTC_Time.second = data[57];

				std::vector<int> anglePrism;
				for (int i = 0; i < 5; i++)
				{
					short int angleP = data[240 + i * 2] * 256 + data[241 + i * 2];
					anglePrism.push_back(angleP);
				}
				getOffsetAngle(anglePrism);

                continue;
            }

            if (time_service_mode == "gps") {
                timestamp_nsce = (data[1200] * pow(2, 24) + data[1201] * pow(2, 16) + data[1202] * pow(2, 8) +
                                  data[1203] * pow(2, 0)) * pow(10, 3);
            } else if (time_service_mode == "gptp") {
				timestamp_nsce = (data[1200] * pow(2, 24) + data[1201] * pow(2, 16) + data[1202] * pow(2, 8) +
                                  data[1203] * pow(2, 0));
            }
            int lidar_EchoModel = data[1205];

            //双回波
            if (2 == lidar_EchoModel) {
                handleDoubleEcho(data);
            }
                //单回波
            else {
                handleSingleEcho(data);
            }

            lastAllTimestamp = allTimestamp;
#pragma endregion
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void GetLidarData_CH128x1::getOffsetAngle(std::vector<int> OffsetAngleValue)
{
	double prismAngle[4];
	float PrismOffsetVAngle = 0.0f;

	if (abs(OffsetAngleValue[0]) != 0)
	{
		PrismOffsetVAngle = OffsetAngleValue[0] / 100.0;

		for (int i = 0; i < 128; i++)
		{
			//左边
			if (i / 4 % 2 == 0)
			{
				sinTheta_1[i] = sin((BigAngle[i / 4] + PrismOffsetVAngle) * PI / 180);
				cosTheta_1[i] = cos((BigAngle[i / 4] + PrismOffsetVAngle) * PI / 180);
			}
			else
			{
				sinTheta_1[i] = sin(BigAngle[i / 4] * PI / 180);
				cosTheta_1[i] = cos(BigAngle[i / 4] * PI / 180);
			}
		}
	}
	else
	{
		for (int i = 0; i < 128; i++)
		{
			sinTheta_1[i] = sin((BigAngle[i / 4]) * PI / 180);
			cosTheta_1[i] = cos((BigAngle[i / 4]) * PI / 180);
		}
	}

	if (abs(OffsetAngleValue[1]) == 0 && abs(OffsetAngleValue[2]) == 0 && abs(OffsetAngleValue[3]) == 0 && abs(OffsetAngleValue[4]) == 0)
	{
		for (int i = 0; i < 4; i++)
		{
			prismAngle[i] = i * -0.17;
		}

		for (int i = 0; i < 128; i++)
		{
			sinTheta_2[i] = sin((prismAngle[i % 4])  * PI / 180);
			cosTheta_2[i] = cos((prismAngle[i % 4])  * PI / 180);
		}
	}
	else
	{
		for (int i = 0; i < 4; i++)
		{
			prismAngle[i] = OffsetAngleValue[i + 1] / 100.0;
		}
		for (int i = 0; i < 128; i++)
		{
			sinTheta_2[i] = sin((prismAngle[i % 4])  * PI / 180);
			cosTheta_2[i] = cos((prismAngle[i % 4])  * PI / 180);
		}
	}
	return;
}

void GetLidarData_CH128x1::handleSingleEcho(unsigned char *data) {
    m_UTC_Time.hour = data[1197];
    m_UTC_Time.minute = data[1198];
    m_UTC_Time.second = data[1199];
    struct tm t;
    t.tm_sec = m_UTC_Time.second;
    t.tm_min = m_UTC_Time.minute;
    t.tm_hour = m_UTC_Time.hour;
    t.tm_mday = m_UTC_Time.day;
    t.tm_mon = m_UTC_Time.month - 1;
    t.tm_year = m_UTC_Time.year - 1900;
    t.tm_isdst = 0;
    time_t _t = static_cast<uint64_t>(timegm(&t)); //s//mktime(&t);
    if (-1 == _t) {
        perror("parse error");
    }

	allTimestamp = _t * pow(10, 9) + timestamp_nsce;
    PointIntervalT_ns = (allTimestamp - lastAllTimestamp) * 1.0 / 171;
    for (int i = 0; i < 1197; i = i + 7) {
        if (data[i] == 0xff && data[i + 1] == 0xaa && data[i + 2] == 0xbb && data[i + 3] == 0xcc &&
            data[i + 4] == 0xdd) {
            count++;
        }
        if (count == 1) {
            if (callback != NULL) {
                sendLidarData();
            } else {
                std::cout << "Please input the callback function!!!" << std::endl;
            }

            count = 0;
            continue;
        }
        if (count == 0) {
            if (data[i] < 128) {
                float m_Distance = (data[i + 3] * 65536 + data[i + 4] * 256 + data[i + 5]) / 256.0 / ConversionUnit;
                int m_ID = data[i];
                float m_H_angle = (data[i + 1] * 256 + data[i + 2]) / 100.f;
                int m_Intensity = data[i + 6];


                m_PointXYZ m_point_1 = XYZ_calculate(m_ID, m_H_angle, m_Distance);
                MuchLidarData m_DataT;
                m_DataT.X = m_point_1.x;
                m_DataT.Y = m_point_1.y;
                m_DataT.Z = m_point_1.z;
                m_DataT.ID = m_ID;
                m_DataT.H_angle = m_H_angle;
                m_DataT.Distance = m_Distance;
                m_DataT.Intensity = m_Intensity;
                m_DataT.Mtimestamp_nsce = allTimestamp - PointIntervalT_ns * (170 - i / 7);
                all_lidardata.push_back(m_DataT);
            }
        }
    }
}

void GetLidarData_CH128x1::handleDoubleEcho(unsigned char *data) {
    m_UTC_Time.second = data[1199];
    struct tm t;
    t.tm_sec = m_UTC_Time.second;
    t.tm_min = m_UTC_Time.minute;
    t.tm_hour = m_UTC_Time.hour;
    t.tm_mday = m_UTC_Time.day;
    t.tm_mon = m_UTC_Time.month - 1;
    t.tm_year = m_UTC_Time.year - 1900;
    t.tm_isdst = 0;
    time_t _t = static_cast<uint64_t>(timegm(&t)); //s//mktime(&t);
    if (-1 == _t) {
        perror("parse error");
    }
	allTimestamp = _t * pow(10, 9) + timestamp_nsce;
    PointIntervalT_ns = (allTimestamp - lastAllTimestamp) * 1.0 / 109;

    for (int i = 0; i < 1199; i = i + 11) {
        if (data[i] == 0xff && data[i + 1] == 0xaa && data[i + 2] == 0xbb && data[i + 3] == 0xcc &&
            data[i + 4] == 0xdd) {
            count++;
        }
        if (count == 1) {
            if (callback != NULL) {
                sendLidarData();
            } else {
                std::cout << "Please input the callback function!!!" << std::endl;
            }

            count = 0;
            continue;
        }
        if (count == 0) {
            if (data[i] < 128) {
                float m_Distance = (data[i + 3] * 65536 + data[i + 4] * 256 + data[i + 5]) / 256.0 / ConversionUnit;
                int m_ID = data[i];
                float m_H_angle = (data[i + 1] * 256 + data[i + 2]) / 100.f;
                int m_Intensity = data[i + 6];

                m_PointXYZ m_point_1 = XYZ_calculate(m_ID, m_H_angle, m_Distance);
                MuchLidarData m_DataT;
                m_DataT.X = m_point_1.x;
                m_DataT.Y = m_point_1.y;
                m_DataT.Z = m_point_1.z;
                m_DataT.ID = m_ID;
                m_DataT.H_angle = m_H_angle;
                m_DataT.Distance = m_Distance;
                m_DataT.Intensity = m_Intensity;
                m_DataT.Mtimestamp_nsce = allTimestamp - PointIntervalT_ns * (108 - i / 11);
                all_lidardata.push_back(m_DataT);


                float m_Distance_2 = (data[i + 7] * 65536 + data[i + 8] * 256 + data[i + 9]) / 256.0 / ConversionUnit;
                int m_Intensity_2 = (data[i + 10]);

                m_PointXYZ m_point_2 = XYZ_calculate(m_ID, m_H_angle, m_Distance_2);
                MuchLidarData m_DataT_d;
                m_DataT_d.X = m_point_2.x;
                m_DataT_d.Y = m_point_2.y;
                m_DataT_d.Z = m_point_2.z;
                m_DataT_d.ID = m_ID;
                m_DataT_d.H_angle = m_H_angle;
                m_DataT_d.Distance = m_Distance_2;
                m_DataT_d.Intensity = m_Intensity_2;
                m_DataT_d.Mtimestamp_nsce = allTimestamp - PointIntervalT_ns * (108 - i / 11);
                all_lidardata_d.push_back(m_DataT_d);
            }
        }
    }
}

m_PointXYZ GetLidarData_CH128x1::XYZ_calculate(int ID, double H_angle, double Distance) {
    m_PointXYZ point;

    double _R_ = cosTheta_2[ID] * cosTheta_1[ID] * cos((H_angle / 2.0) * PI / 180) - sinTheta_2[ID] * sinTheta_1[ID];

    float sin_Theat;
    float cos_Theta;

    sin_Theat = sinTheta_1[ID] + 2 * _R_ * sinTheta_2[ID];
    cos_Theta = sqrt(1 - pow(sin_Theat, 2));

    point.x = float(Distance * cos_Theta * cos(H_angle * PI / 180));
    point.y = float(Distance * cos_Theta * sin(H_angle * PI / 180));
    point.z = float(Distance * sin_Theat);

    return point;
}


