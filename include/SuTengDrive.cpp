// #include "SuTengDrive.h"



// SutengDrive::SutengDrive(){

// }

// void SutengDrive::init(int MsopPort, int DifopPort, const std::string &LidarType, int Number){

//     m_iMsopPort = MsopPort;  //2368
//     m_iDifopPort = DifopPort;   //2369
//     m_sSuTengType = LidarType;  //LidarType::RS128
// }

// void SutengDrive::start(){

//     RSDriverParam param;                  ///< Create a parameter object
//     param.input_type = InputType::ONLINE_LIDAR;
//     param.input_param.msop_port = m_iMsopPort;   ///< Set the lidar msop port number, the default is 6699
//     param.input_param.difop_port = m_iDifopPort;  ///< Set the lidar difop port number, the default is 7788
//     param.lidar_type = m_sSuTengType;   ///< Set the lidar type. Make sure this type is correct
//     param.print();

//     LidarDriver<PointCloudMsg> driver;  
// }


// void SutengDrive::free(){

    
// }


// void SutengDrive::stop(){

    
// }