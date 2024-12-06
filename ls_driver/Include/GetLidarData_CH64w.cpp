#include "GetLidarData_CH64w.h"

GetLidarData_CH64w::GetLidarData_CH64w() {
    for (int i = 0; i < 4; i++) {
        prismAngle[i] = i * 0.35;
    }

    for (int i = 0; i < 128; i++) {
        //右边
        if (i / 4 % 2 == 0) {
            sinTheta_1[i] = sin((-25 + floor(i / 8) * 2.5) * PI / 180);
            sinTheta_2[i] = sin((prismAngle[i % 4]) * PI / 180);
            cosTheta_1[i] = cos((-25 + floor(i / 8) * 2.5) * PI / 180);
            cosTheta_2[i] = cos((prismAngle[i % 4]) * PI / 180);
        }
            //左边
        else {
            sinTheta_1[i] = sin((-24 + floor(i / 8) * 2.5) * PI / 180);
            sinTheta_2[i] = sin((prismAngle[i % 4]) * PI / 180);
            cosTheta_1[i] = cos((-24 + floor(i / 8) * 2.5) * PI / 180);
            cosTheta_2[i] = cos((prismAngle[i % 4]) * PI / 180);
        }
    }


    all_lidardata.clear();
    all_lidardata_d.clear();
    Model = 0;
    count = 0;
}

GetLidarData_CH64w::~GetLidarData_CH64w() {
}

void GetLidarData_CH64w::LidarRun() {
    int count = 0;
    while (true) {
        if (isQuit) {
            clearQueue(allDataValue);
            break;
        }
        // printf("LidarRun 的 allDataValue size: %d\n",allDataValue.size());
        if (!allDataValue.empty()) {
            unsigned char data[1206] = {0};
            m_mutex.lock();
            memcpy(data, allDataValue.front(), sizeof(data));
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
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void GetLidarData_CH64w::getOffsetAngle(std::vector<int> OffsetAngleValue)
{
	double prismAngle[4];
	float PrismOffsetVAngle = 0.0f;

	if (abs(OffsetAngleValue[0]) != 0)
	{
		PrismOffsetVAngle = OffsetAngleValue[0] / 100.0;

		for (int i = 0; i < 128; i++)
		{
			//右边
			if (i / 4 % 2 == 0)
			{
				sinTheta_1[i] = sin((-25 + floor(i / 8) * 2.5) * PI / 180);
				cosTheta_1[i] = cos((-25 + floor(i / 8) * 2.5) * PI / 180);
			}
			//左边
			else
			{
				sinTheta_1[i] = sin((-24 + PrismOffsetVAngle + floor(i / 8) * 2.5) * PI / 180);

				cosTheta_1[i] = cos((-24 + PrismOffsetVAngle + floor(i / 8) * 2.5) * PI / 180);
			}
		}
	}

	if (abs(OffsetAngleValue[1]) == 0 && abs(OffsetAngleValue[2]) == 0 && abs(OffsetAngleValue[3]) == 0 && abs(OffsetAngleValue[4]) == 0)
	{
		for (int i = 0; i < 4; i++)
		{
			prismAngle[i] = i * 0.35;
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

void GetLidarData_CH64w::handleSingleEcho(unsigned char *data) {
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

    static int s_iPacketNum = 0;
    s_iPacketNum++;
	allTimestamp = _t * pow(10, 9) + timestamp_nsce;
    PointIntervalT_ns = (allTimestamp - lastAllTimestamp) * 1.0 / 171.0;

    for (int i = 0; i < 1197; i = i + 7) 
    {
        if (data[i] == 0xff && data[i + 1] == 0xaa && data[i + 2] == 0xbb && data[i + 3] == 0xcc &&
            data[i + 4] == 0xdd) 
        {
            count++;
        }
        if (count == 1) 
        {
            if (callback != NULL) 
            {
                // printf("s_iPacketNum = %d\n", s_iPacketNum);
                sendLidarData();
            } 
            else 
            {
                std::cout << "Please input the callback function!!!" << std::endl;
            }
            s_iPacketNum = 0;
            count = 0;
            continue;
        }
        if (count == 0) {
            if (data[i] < 128) {
                float m_Distance = (data[i + 3] * 65536 + data[i + 4] * 256 + data[i + 5]) / 256.0 / ConversionUnit;
                int m_ID = data[i];
                float m_H_angle = (data[i + 1] * 256 + data[i + 2]) / 100.f;
                int m_Intensity = data[i + 6];
				
				if(m_H_angle < 0 || m_H_angle > 180)
				{
					continue;
				}
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

void GetLidarData_CH64w::handleDoubleEcho(unsigned char *data) {
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
    PointIntervalT_ns = (allTimestamp - lastAllTimestamp) * 1.0 / 171.0;

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

m_PointXYZ GetLidarData_CH64w::XYZ_calculate(int ID, double H_angle, double Distance) {
    m_PointXYZ point;

	double cos_xita;
	double sin_xita;


	//右边
	if (ID / 4 % 2 == 0)
	{
		cos_xita = cos((H_angle / 2.0 + 22.5) * PI / 180);
		sin_xita = sin((H_angle / 2.0 + 22.5) * PI / 180);
	}
	else
	{
		cos_xita = cos((-H_angle / 2.0 + 112.5) *PI / 180);
		sin_xita = sin((-H_angle / 2.0 + 112.5) *PI / 180);
	}

	//中间变量
	double _R_ = cosTheta_2[ID] * cosTheta_1[ID] * cos_xita - sinTheta_2[ID] * sinTheta_1[ID];

	//垂直角度
	double sin_Theta = sinTheta_1[ID] + 2 * _R_ * sinTheta_2[ID];
	double cos_Theta = sqrt(1 - pow(sin_Theta, 2));

	//水平角度
	double cos_H_xita = (2 * _R_ * cosTheta_2[ID] * cos_xita - cosTheta_1[ID]) / cos_Theta;

	//double sin_H_xita = sqrt(1 - pow(cos_H_xita, 2));

	double sin_H_xita = (2 * _R_ * cosTheta_2[ID] * sin_xita) / cos_Theta;

	double cos_xita_F = 0;
	double sin_xita_F = 0;

	float addDis = 0;
	//右边
	if (ID / 4 % 2 == 0)
	{
		cos_xita_F = (cos_H_xita + sin_H_xita) * sqrt(0.5);
		sin_xita_F = (sin_H_xita - cos_H_xita) * sqrt(0.5);

		if (cos_xita_F > 1)
		{
			cos_xita_F = 1;
		}

		double xita_Hangle = acos(cos_xita_F) * 180 / PI;
		double xita_Hangle_new = -3.6636 * pow(10, -7) * pow(xita_Hangle, 3)
			+ 5.2766 * pow(10, -5) * pow(xita_Hangle, 2)
			+ 0.9885  * pow(xita_Hangle, 1)
			+ 0.5894;

		cos_xita_F = cos(xita_Hangle_new * PI / 180);
		sin_xita_F = sin(xita_Hangle_new * PI / 180);

		addDis = 0.017;
	}
	else
	{
		cos_xita_F = (cos_H_xita + sin_H_xita) * (-sqrt(0.5));
		sin_xita_F = (sin_H_xita - cos_H_xita) * sqrt(0.5);

		if (cos_xita_F < -1)
		{
			cos_xita_F = -1;
		}

		double xita_Hangle = acos(cos_xita_F) * 180 / PI;
		double xita_Hangle_new = -3.6636 * pow(10, -7) * pow(xita_Hangle, 3)
			+ 1.4507 * pow(10, -4) * pow(xita_Hangle, 2)
			+ 0.9719  * pow(xita_Hangle, 1)
			+ 1.9003;

		cos_xita_F = cos(xita_Hangle_new * PI / 180);
		sin_xita_F = sin(xita_Hangle_new * PI / 180);


		addDis = -0.017;
	}

	point.x = float(Distance * cos_Theta * cos_xita_F + addDis);
	point.y = float(Distance * cos_Theta * sin_xita_F);
	point.z = float(Distance * sin_Theta);

	return point;
}


