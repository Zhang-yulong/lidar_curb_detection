#ifndef BASEDRIVE_H
#define BASEDRIVE_H

#include <functional>
#include <string>
#include <mutex>



class BaseDrive{

public:
    BaseDrive();
    virtual void Start() = 0;
    virtual void Init(std::string ComputerIP, int MsopPort, int DifopPort, const std::string LidarType, int Number) = 0;
    virtual void Free() = 0;
    virtual ~BaseDrive();
};

#endif