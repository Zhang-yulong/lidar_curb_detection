#pragma once

#include "GetLidarData.h"

class GetLidarData_CH128S1 : public GetLidarData
{
public:
	GetLidarData_CH128S1();
	~GetLidarData_CH128S1();

	void LidarRun();
	m_PointXYZ XYZ_calculate(int, double, double);	

private:
	float sinTheta_1[128];												//竖直角
	float sinTheta_2[128];												//竖直角
	float cosTheta_1[128];												//竖直角
	float cosTheta_2[128];												//竖直角

	void getOffsetAngle(std::vector<int> OffsetAngleValue);				//获取雷达补偿角

	int count = 0;													//同步帧判定
	void handleSingleEcho(unsigned char* data);						//单回波处理
	void handleDoubleEcho(unsigned char* data);						//双回波处理
};

