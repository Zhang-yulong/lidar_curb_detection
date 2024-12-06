#include "GetLidarData.h"

GetLidarData::GetLidarData()
{

}
GetLidarData::~GetLidarData()
{

}

void GetLidarData::setCallbackFunction(Fun* callbackValue)
{
	printf("进入 setCallbackFunction \n");
	callback = callbackValue;
}

// void GetLidarData::setCallback(std::function<void(std::vector<MuchLidarData>, int)>fun){
// 	callback = fun;
// }

void GetLidarData::LidarStar()
{
	isQuit = false;
	std::thread t1(&GetLidarData::LidarRun, this);
	t1.detach();
}

void GetLidarData::LidarStop()
{
	isQuit = true;
}

bool GetLidarData::setEcho_SingleOrDouble(int value)
{
	Model = value;
	return true;
}

void GetLidarData::sendLidarData()
{
	// printf("Model = %d, all_lidardata = %d, all_lidardata_d = %d\n", Model, all_lidardata.size(), all_lidardata_d.size());
	if (Model == 0)
	{
		all_lidardata.insert(all_lidardata.end(), all_lidardata_d.begin(), all_lidardata_d.end());
		(*callback)(all_lidardata, iDataPort);

	}
	else if (Model == 1)
	{
		(*callback)(all_lidardata, iDataPort);
	}

	else if (Model == 2)
	{
		(*callback)(all_lidardata_d, iDataPort);
	}
	all_lidardata.clear();
	all_lidardata_d.clear();
}

void GetLidarData::CollectionDataArrive(void * pData, uint16_t len)
{
	if (len >= 1206)
	{
		unsigned char *dataV = new unsigned char[1212];
		memset(dataV, 0, 1212);	 
		memcpy(dataV, pData, len);
		m_mutex.lock();
		allDataValue.push(dataV);
		m_mutex.unlock();
	}
}

void GetLidarData::clearQueue(std::queue<unsigned char *>& m_queue)
{
	std::queue<unsigned char *> empty;
	swap(empty, m_queue);
}
