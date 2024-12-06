#pragma once

#include "GetLidarData.h"

class GetLidarData_C32_v3_0 : public GetLidarData
{
public:
	GetLidarData_C32_v3_0();
	~GetLidarData_C32_v3_0();

	void LidarRun();
	m_PointXYZ XYZ_calculate(int, double, double);	
private:
	float vertialAngles[32];										//设备包内的垂直角度
	double cosTheta[32];
	double sinTheta[32];
	double AdjustAngle[2];											//两列夹角补偿
	MuchLidarData m_DataT[32];
	MuchLidarData m_DataT_d[32];

	double endFrameAngle = 0;										//结束一帧的的角度（也是开始的角度）
	std::vector<float>BlockAngle;									//每个块的角度
	std::vector<float>all_BlockAngle;								//每个帧的所有块的角度
	void setVertialAngles(unsigned char* data);					    //设置垂直角度

	void handleSingleEcho(unsigned char* data);						//单回波处理
	void handleDoubleEcho(unsigned char* data);						//双回波处理
};

