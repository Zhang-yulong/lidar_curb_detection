#ifndef BASEDRIVE_H
#define BASEDRIVE_H

#include <functional>
#include <string>
#include <mutex>

namespace Lidar_Curb_Dedection
{

class BaseDrive{

public:
    BaseDrive();
    virtual void Start() = 0;
    virtual void Init() = 0;
    virtual void Free() = 0;
    virtual ~BaseDrive();
};
}
#endif