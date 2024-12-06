#include "GetLidarData_CH32_WideAngle.h"

GetLidarData_CH32_WideAngle::GetLidarData_CH32_WideAngle()
{
	float theta[8] = { -14.0, -11.0, -8.0, -5.0, -2.0, 0.0, 2.0, 5.0 };
	//求垂直角度cos值
	for (int i = 0; i < 32; i++)
	{
		cosTheta[i] = cos((theta[i / 4] + (i % 4)*0.33)* PI / 180);
		sinTheta[i] = sin((theta[i / 4] + (i % 4)*0.33) * PI / 180);
	}

	//求垂直角度sin值
	for (int i = 0; i < 32; i = i + 4)
	{
		//求一撇值
		Theat_T[i] = sin((theta[i / 4])*PI / 180);
		Theat_T[i + 1] = sin((theta[i / 4])*PI / 180);
		Theat_T[i + 2] = sin((theta[i / 4])*PI / 180);
		Theat_T[i + 3] = sin((theta[i / 4])*PI / 180);

		//求两撇值
		Theat_Q[i] = 0;
		Theat_Q[i + 1] = sin(0.33 * PI / 180);
		Theat_Q[i + 2] = sin(0.67 * PI / 180);
		Theat_Q[i + 3] = sin(1.0  * PI / 180);
	}

	all_lidardata.clear();
	all_lidardata_d.clear();
	Model = 0;
	count = 0;
}

GetLidarData_CH32_WideAngle::~GetLidarData_CH32_WideAngle()
{
}

void GetLidarData_CH32_WideAngle::LidarRun()
{
	while (true)
	{
		if (isQuit)
		{
			clearQueue(allDataValue);
			break;
		}
		if (!allDataValue.empty())
		{
			unsigned char data[1206] = { 0 };
			m_mutex.lock();
			memcpy(data, allDataValue.front(), 1206);
			delete allDataValue.front();
			allDataValue.pop();
			m_mutex.unlock();

			if (data[0] == 0xa5 && data[1] == 0xff && data[2] == 0x00 && data[3] == 0x5a)
			{
				m_UTC_Time.year = data[36] + 2000;
				m_UTC_Time.month = data[37];
				m_UTC_Time.day = data[38];
				m_UTC_Time.hour = data[39];
				m_UTC_Time.minute = data[40];
				m_UTC_Time.second = data[41];
				continue;
			}

			timestamp_nsce = (data[1203] * pow(2, 24) + data[1204] * pow(2, 16) + data[1205] * pow(2, 8) + data[1206] * pow(2, 0)) * pow(10, 3);
			int lidar_EchoModel = data[1205];


			//双回波
			if (2 == lidar_EchoModel)
			{
				handleDoubleEcho(data);
			}
			//单回波
			else
			{
				handleSingleEcho(data);
			}

			lastAllTimestamp = allTimestamp;
#pragma endregion
		}
		else
		{
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}

void GetLidarData_CH32_WideAngle::handleSingleEcho(unsigned char* data)
{
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
	time_t _t = mktime(&t);
	if (-1 == _t){
		perror("parse error");
	}

	allTimestamp = _t * pow(10, 9) + timestamp_nsce;
	PointIntervalT_ns = (allTimestamp - lastAllTimestamp) * 1.0 / 171;
	for (int i = 0; i < 1197; i = i + 7)
	{
		if (data[i] == 0xff && data[i + 1] == 0xaa && data[i + 2] == 0xbb)
		{
			count++;
		}
		if (count == 1)
		{
			if (callback != NULL)
			{
				std::cout << all_lidardata.size() << std::endl;
				sendLidarData();
			}
			else
			{
				std::cout << "Please input the callback function!!!" << std::endl;
			}

			count = 0;
			continue;
		}
		if (count == 0)
		{
			if (data[i] < 128)
			{
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

void GetLidarData_CH32_WideAngle::handleDoubleEcho(unsigned char* data)
{
	m_UTC_Time.second = data[1199];
	struct tm t;
	t.tm_sec = m_UTC_Time.second;
	t.tm_min = m_UTC_Time.minute;
	t.tm_hour = m_UTC_Time.hour;
	t.tm_mday = m_UTC_Time.day;
	t.tm_mon = m_UTC_Time.month - 1;
	t.tm_year = m_UTC_Time.year - 1900;
	t.tm_isdst = 0;
	time_t _t = mktime(&t);
	if (-1 == _t){
		perror("parse error");
	}
	allTimestamp = _t * pow(10, 9) + timestamp_nsce;
	PointIntervalT_ns = (allTimestamp - lastAllTimestamp) * 1.0 / 109;

	for (int i = 0; i < 1199; i = i + 11)
	{
		if (data[i] == 0xff && data[i + 1] == 0xaa && data[i + 2] == 0xbb && data[i + 3] == 0xcc && data[i + 4] == 0xdd)
		{
			count++;
		}
		if (count == 1)
		{
			if (callback != NULL)
			{
				sendLidarData();
			}
			else
			{
				std::cout << "Please input the callback function!!!" <<  std::endl;
			}

			count = 0;
			continue;
		}
		if (count == 0)
		{
			if (data[i] < 128)
			{
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

m_PointXYZ GetLidarData_CH32_WideAngle::XYZ_calculate(int ID, double H_angle, double Distance)
{
	m_PointXYZ point;

	point.x = Distance * cosTheta[ID] * cos(H_angle * PI / 180);
	point.y = Distance * cosTheta[ID] * sin(H_angle * PI / 180);
	point.z = Distance * (Theat_T[ID] + 2 * cos(H_angle * PI / 360)*Theat_Q[ID]);

	return point;
}


