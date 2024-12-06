#pragma once

#include "GetLidarData.h"

class GetLidarData_CH32 : public GetLidarData
{
public:
	GetLidarData_CH32();
	~GetLidarData_CH32();

	void LidarRun();
	m_PointXYZ XYZ_calculate(int, double, double);	
private:
	float cosTheta[32];                                              //竖直角
	float sinTheta[32];                                              //竖直角
	float Theat_T[32];
	float Theat_Q[32];


	int count = 0;													//同步帧判定
	void handleSingleEcho(unsigned char* data);						//单回波处理
	void handleDoubleEcho(unsigned char* data);						//双回波处理
};

