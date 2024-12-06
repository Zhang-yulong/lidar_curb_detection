#pragma once

#include "GetLidarData.h"

class GetLidarData_CH64 : public GetLidarData
{
public:
	GetLidarData_CH64();
	~GetLidarData_CH64();

	void LidarRun();
	m_PointXYZ XYZ_calculate(int, double, double);	
private:
	float cosTheta[64];                                              //竖直角
	float sinTheta[64];                                              //竖直角
	float Theat_T[64];
	float Theat_Q[64];


	int count = 0;													//同步帧判定
	void handleSingleEcho(unsigned char* data);						//单回波处理
	void handleDoubleEcho(unsigned char* data);						//双回波处理
};

