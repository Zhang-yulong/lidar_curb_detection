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
	float vertialAngles[32];										//�豸���ڵĴ�ֱ�Ƕ�
	double cosTheta[32];
	double sinTheta[32];
	double AdjustAngle[2];											//���мнǲ���
	MuchLidarData m_DataT[32];
	MuchLidarData m_DataT_d[32];

	double endFrameAngle = 0;										//����һ֡�ĵĽǶȣ�Ҳ�ǿ�ʼ�ĽǶȣ�
	std::vector<float>BlockAngle;									//ÿ����ĽǶ�
	std::vector<float>all_BlockAngle;								//ÿ��֡�����п�ĽǶ�
	void setVertialAngles(unsigned char* data);					    //���ô�ֱ�Ƕ�

	void handleSingleEcho(unsigned char* data);						//���ز�����
	void handleDoubleEcho(unsigned char* data);						//˫�ز�����
};

