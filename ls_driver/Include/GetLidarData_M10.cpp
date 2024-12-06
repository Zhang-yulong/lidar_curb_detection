#include "GetLidarData_M10.h"
//#include "stdafx.h"
//#include "SerialPort.h"
#define PP 3.1415926

GetLidarData_M10::GetLidarData_M10()
{

}

GetLidarData_M10::~GetLidarData_M10()
{
	deleteLidarData();
}

void GetLidarData_M10::init(bool Serial, std::string computer_ip, int data_ip, std::string lidar_ip, int lidar_port)
{
	all_lidardata.clear();
	all_lidardata_d.clear();
	BlockAngle.clear();
	endFrameAngle = 90.0;					//固定在90度解析数据
	Model = 0;
	m_SerialP = Serial;

	if (m_SerialP)
	{
		comName = computer_ip;
		comPort = data_ip;
		openSerial(comName.c_str(), comPort);
	}
	else
	{
		cpt_ip = computer_ip;
		dataPort = data_ip;
	}

	if (!m_SerialP)
	{
		cpt_ip_1 = lidar_ip;
		lidarPort = lidar_port;
	}
}

void GetLidarData_M10::LidarStar()
{
	setLidarRotateState(0);

	isQuit = false;
	if (m_SerialP)
	{
		if (isQuit_com)
		{
			openSerial(comName.c_str(), comPort);
		}
		isQuit_com = false;
		std::thread t1(&GetLidarData_M10::receive, this);
		t1.detach();
	}
	else
	{
		std::thread t1(&GetLidarData_M10::LidarRun, this);
		t1.detach();
	}
}
void GetLidarData_M10::LidarRun()
{
#ifdef LINUX
#else
	wVerRequest = MAKEWORD(1, 1);
	WSAStartup(wVerRequest, &wsaData);
#endif
	//创建socket
	m_sock = socket(2, 2, 0);			//构建sock
	//UDP通信初始化
	//创建socket
	m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	//定义地址
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(dataPort);

	inet_pton(AF_INET, cpt_ip.c_str(), &sockAddr.sin_addr);

	int value = 1 * 1024 * 1024;
	int tmpCode = 0;
	tmpCode = ::setsockopt(m_sock, SOL_SOCKET, SO_RCVBUF, (char*)&value, sizeof(value));

	//绑定套接字
	int retVal = bind(m_sock, (struct sockaddr*)&sockAddr, sizeof(sockAddr));
	struct sockaddr_in addrFrom;

#ifdef LINUX
	socklen_t len = sizeof(sockaddr_in);
#else
	int len = sizeof(sockaddr_in);
#endif

	int recvLen;
	char recvBuf[1212] = { 0 };

	while (true)
	{
#ifdef LINUX
		recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);
#else
		recvLen = recvfrom(m_sock, recvBuf, sizeof(recvBuf), 0, (SOCKADDR*)&addrFrom, &len);
#endif
		if (recvLen >= 92)
		{
			u_char data[1212] = { 0 };
			memcpy(data, recvBuf, recvLen);
			if (isQuit)
			{
				#ifdef LINUX
					close(m_sock); //关闭套接字
				#else
					closesocket(m_sock);
				#endif
				break;
			}

			if (recvLen == 92)
			{
				handleSingleEcho(data);
			}
			else if (recvLen == 102)
			{
				handleSingleEcho_GPS(data);
			}
			else if (recvLen >= 100 && recvLen != 102)
			{
				handleSingleEcho_P(data);
			}
		}
		else
		{
			std::cout << "connection error" << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}
// 打开串口
bool GetLidarData_M10::openSerial(const char* portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag)
{
#ifdef LINUX
	int error_code = 0;
        //以读写方式打开串口设备
        /*
         * portname 	 串口设备的路径
		 * baudrate		 波特率
		 * parity		 奇偶校验
		 * databit		 起始位
		 * stopbit		 停止位
		 * 
         * O_RDWR        可读可写
         * O_NOCTTY      不将该设备作为控制终端
         * O_NDELAY      非阻塞模式
         * 返回值 fd_     打开的文件描述符
        */
	fd_ = open(portname, O_RDWR|O_NOCTTY|O_NDELAY);
	if (0 < fd_)  //串口打开成功
	{
		printf("Open Port %s  OK !\n", portname);
		error_code = 0;
    	//设置串口的参数，包括数据位、校验位和停止位
		
		//保存当前串口参数设置
		struct termios newtio;

		// 将newtio结构体清零，以便进行新的参数设置
		bzero(&newtio, sizeof(newtio));
		/* 步骤一，设置字符大小 */
		newtio.c_cflag |= CLOCAL;   //如果设置，modem 的控制线将会被忽略。如果没有设置，则 open()函数会阻塞直到载波检测线宣告 modem 处于摘机状态为止。
		newtio.c_cflag |= CREAD;    //使端口能读取输入的数据  串口可以接收数据
		/* 设置每个数据的位数 */
		switch (databit)
		{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
		}
		/* 设置奇偶校验位 */
		switch (parity)
		{
		case '1': //奇数
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;   //使能校验，如果不设PARODD则是偶校验
			newtio.c_cflag |= PARODD;   //奇校验
			break;
		case '2': //偶数
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case '0':  //无奇偶校验位
			newtio.c_cflag &= ~PARENB;
			break;
		}

		/* 设置波特率 */
		cfsetispeed(&newtio, B460800);
    	cfsetospeed(&newtio, B460800);
		std::cout<<"baudrate:"<<baudrate<<std::endl;

		/*
		* 设置停止位
		* 设置停止位的位数， 如果设置，则会在每帧后产生两个停止位， 如果没有设置，则产生一个
		* 停止位。一般都是使用一位停止位。需要两位停止位的设备已过时了。
		* */
		if (stopbit == 1)
			newtio.c_cflag &= ~CSTOPB;    //将 CSTOPB 标志位清除，停止位设置为1个
		else if (stopbit == 2)
			newtio.c_cflag |= CSTOPB;     //将 CSTOPB 标志位设为1，停止位设置为2个
		/* 设置等待时间和最小接收字符 */
		newtio.c_cc[VTIME] = 0;   // 不进行等待，即立即返回
		newtio.c_cc[VMIN] = 0;    // 不需要接收任何字符，即立即返回
		/* 处理未接收字符  清空输入缓冲区 */
		tcflush(fd_, TCIFLUSH);
		/* 激活新配置          立即生效  串口参数结构体的指针  波特率、数据位数、奇偶校验位、停止位等 */
		if ((tcsetattr(fd_, TCSANOW, &newtio)) != 0)  //配置信息更新为 newtio 结构体中的配置
		{
			perror("serial set error");
			return -1;
		}

	} else {          //串口打开失败
		printf("Open Port %s  error !\n", portname);
		error_code = -1;  //错误码设置为 -1
	}

	return error_code;  //返回错误码
#else
			this->synchronizeflag = synchronizeflag;
	HANDLE hCom = NULL;
	if (this->synchronizeflag)
	{
		//同步方式
		hCom = CreateFileA(portname,		//串口名
			GENERIC_READ | GENERIC_WRITE, 	//支持读写
			0, 								//独占方式，串口不支持共享
			NULL,							//安全属性指针，默认值为NULL
			OPEN_EXISTING, 					//打开现有的串口文件
			0, 								//0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
			NULL);							//用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL
	}
	else
	{
		//异步方式
		hCom = CreateFileA(portname, 		//串口名
			GENERIC_READ | GENERIC_WRITE, 	//支持读写
			0, 								//独占方式，串口不支持共享
			NULL,							//安全属性指针，默认值为NULL
			OPEN_EXISTING,  				//打开现有的串口文件
			FILE_FLAG_OVERLAPPED, 			//0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
			NULL);							//用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL
	}

	if (hCom == (HANDLE)-1)
	{
		return false;
	}

	//配置缓冲区大小 
	if (!SetupComm(hCom, 1024, 1024))
	{
		return false;
	}

	// 配置参数
	DCB p;
	memset(&p, 0, sizeof(p));
	p.DCBlength = sizeof(p);
	p.BaudRate = baudrate; 		// 波特率
	p.ByteSize = databit;  		// 数据位

	switch (parity)  			//校验位
	{
	case 0:
		p.Parity = NOPARITY; 	//无校验
		break;
	case 1:
		p.Parity = ODDPARITY; 	//奇校验
		break;
	case 2:
		p.Parity = EVENPARITY; 	//偶校验
		break;
	case 3:
		p.Parity = MARKPARITY; 	//标记校验
		break;
	}

	switch (stopbit) 			//停止位
	{
	case 1:
		p.StopBits = ONESTOPBIT; //1位停止位
		break;
	case 2:
		p.StopBits = TWOSTOPBITS; //2位停止位
		break;
	case 3:
		p.StopBits = ONE5STOPBITS; //1.5位停止位
		break;
	}

	if (!SetCommState(hCom, &p))
	{
		// 设置参数失败
		return false;
	}

	//超时处理,单位：毫秒
	//总超时＝时间系数×读或写的字符数＋时间常量
	COMMTIMEOUTS TimeOuts;
	TimeOuts.ReadIntervalTimeout = 1000; 	   //读间隔超时
	TimeOuts.ReadTotalTimeoutMultiplier = 500; //读时间系数
	TimeOuts.ReadTotalTimeoutConstant = 5000;  //读时间常量
	TimeOuts.WriteTotalTimeoutMultiplier = 500;// 写时间系数
	TimeOuts.WriteTotalTimeoutConstant = 2000; //写时间常量
	SetCommTimeouts(hCom, &TimeOuts);

	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//清空串口缓冲区

	memcpy(pHandle, &hCom, sizeof(hCom));		// 保存句柄

	return true;
#endif
}

//读取串口数据
std::string GetLidarData_M10::receive()
{
#ifdef LINUX
	int rc;
	unsigned char buf[92];		// 储存串口接收到的数据
	unsigned char* pb = buf;
    int count = 0;      		// 已经接收到的数据的总长度
	int count_2 = 0;    		// 每次接收到的数据的长度
	std::string rec_str = "";
	

	u_char data_1[92] = { 0 };
	static int subsize = 0;
	u_char data[92] = { 0 };
	memset(buf, 0, 92);			// 将缓冲区清零，以确保缓冲区中的数据为空

	while (true)
	{
		if (isQuit)
		{
			closeserial();
			break;
		}

		while(count <= 0)   	// 直到成功接收到一个字节的数据
		{   
			//读取一个字节的数据
			count = ::read(fd_, pb , 1);
		}
		if(pb[0] != 0xA5){
			count = 0;
			continue;
		}   //判断包头
		
		while(count_2 <= 0) 	//直到成功读取到一个字节的数据
		{
			count_2 = ::read(fd_, pb + count, 1);
			if(count_2 >= 0) count += count_2;
		}
		count_2 = 0;
		if(pb[1] != 0x5A) continue; //判断包头

		while(count < 92)		// 接受剩余90字节数据
    	{
			count_2 = ::read(fd_, pb + count, 92 - count);
        	if(count_2 >= 0) count += count_2;
		}
		count = 0;
		memcpy(data_1, buf, 92);
		
		if (subsize != 0)
		{
			for (int k = 0; k < 92 - m_subsize; k++)
			{
				data[subsize] = data_1[k];
				subsize += 1;

				if (subsize == 92)
				{
					subsize = 0;
					if (data[90] == 0xfa && data[91] == 0xfb)
					{
						handleSingleEcho(data);		// 解析数据
					}
					break;
				}
			}

		}
		else
		{
			for (int i = 0; i < 91; i++)
			{
				if (data_1[i] == 0xa5 && data_1[i + 1] == 0x5a)
				{
					subsize = 0;
					for (int k = 0; k < 92 - i; k++)
					{
						data[subsize] = data_1[i + k];
						subsize += 1;
						m_subsize = subsize;

						if (subsize == 92)
						{
							subsize = 0;
							if (data[90] == 0xfa && data[91] == 0xfb)
							{
								
								handleSingleEcho(data);	// 解析数据
							}

							break;
						}
					}
					break;
				}
			}
		}

	}
	return rec_str;
#else
		HANDLE hCom = *(HANDLE*)pHandle;
	std::string rec_str = "";
	char buf[92];

	if (this->synchronizeflag)
	{
		//同步方式
		DWORD wCount = 1024; 			//成功读取的数据字节数
		BOOL bReadStat = ReadFile(hCom, //串口句柄
			buf, 						//数据首地址
			wCount, 					//要读取的数据最大字节数
			&wCount, 					//DWORD*,用来接收返回成功读取的数据字节数
			NULL); 						//NULL为同步发送，OVERLAPPED*为异步发送
		return rec_str;
	}
	else
	{
		//异步方式
		DWORD wCount = 92; 				//成功读取的数据字节数
		DWORD dwErrorFlags; 			//错误标志
		COMSTAT comStat; 				//通讯状态
		OVERLAPPED m_osRead; 			//异步输入输出结构体

		//创建一个用于OVERLAPPED的事件处理，不会真正用到，但系统要求这么做
		memset(&m_osRead, 0, sizeof(m_osRead));
		m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, L"ReadEvent");

		ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误，获得设备当前状态
		

		u_char data_1[92] = { 0 };
		static int subsize = 0;
		u_char data[92] = { 0 };

		while (true)
		{
			if (isQuit)
			{
				closeserial();
				break;
			}
			wCount = 92;
			BOOL bReadStat = ReadFile(hCom, //串口句柄
				buf, 						//数据首地址
				wCount,  					//要读取的数据最大字节数
				&wCount, 					//DWORD*,用来接收返回成功读取的数据字节数
				&m_osRead); 				//NULL为同步发送，OVERLAPPED*为异步发送

			if (!bReadStat)
			{
				if (GetLastError() == ERROR_IO_PENDING) //如果串口正在读取中
				{
					//GetOverlappedResult函数的最后一个参数设为TRUE
					//函数会一直等待，直到读操作完成或由于错误而返回
					GetOverlappedResult(hCom, &m_osRead, &wCount, TRUE);
				}
				else
				{
					ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误
					CloseHandle(m_osRead.hEvent); 				   //关闭并释放hEvent的内存
					return "0";

				}
			}
			memcpy(data_1, buf, wCount);

			if (subsize != 0)
			{
				for (int k = 0; k < 92 - m_subsize; k++)
				{
					data[subsize] = data_1[k];
					subsize += 1;

					if (subsize == 92)
					{
						subsize = 0;
						if (data[90] == 0xfa && data[91] == 0xfb)
						{
							handleSingleEcho(data);	// 解析数据
						}
						break;
					}
				}

			}
			else
			{
				for (int i = 0; i < 91; i++)
				{
					if (data_1[i] == 0xa5 && data_1[i + 1] == 0x5a)
					{
						subsize = 0;
						for (int k = 0; k < 92 - i; k++)
						{
							data[subsize] = data_1[i + k];
							subsize += 1;
							m_subsize = subsize;

							if (subsize == 92)
							{
								subsize = 0;
								if (data[90] == 0xfa && data[91] == 0xfb)
								{
									handleSingleEcho(data);	// 解析数据
								}

								break;
							}
						}
						break;
					}
				}
			}


		}
		return rec_str;
	}
#endif


}


//发送数据
int GetLidarData_M10::send(char* dat, int datalong)
{
#ifdef LINUX
	datalong = write(fd_, dat, (size_t)datalong);	// 向串口写数据
	return datalong;
#else
	HANDLE hCom = *(HANDLE*)pHandle;

	if (this->synchronizeflag)
	{
		// 同步方式
		DWORD dwBytesWrite = datalong; 				//成功写入的数据字节数
		BOOL bWriteStat = WriteFile(hCom, 			//串口句柄
			(char*)dat, 							//数据首地址
			dwBytesWrite, 							//要发送的数据字节数
			&dwBytesWrite, 							//DWORD*，用来接收返回成功发送的数据字节数
			NULL); 									//NULL为同步发送，OVERLAPPED*为异步发送
		if (!bWriteStat)
		{
			return 0;
		}
		return dwBytesWrite;
	}
	else
	{
		//异步方式
		DWORD dwBytesWrite = static_cast <unsigned long>(datalong); //成功写入的数据字节数
		DWORD dwErrorFlags; 						//错误标志
		COMSTAT comStat; 							//通讯状态
		OVERLAPPED m_osWrite; 						//异步输入输出结构体

		//创建一个用于OVERLAPPED的事件处理，不会真正用到，但系统要求这么做
		memset(&m_osWrite, 0, sizeof(m_osWrite));
		m_osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, L"WriteEvent");

		ClearCommError(hCom, &dwErrorFlags, &comStat);	//清除通讯错误，获得设备当前状态
		BOOL bWriteStat = WriteFile(hCom, 			//串口句柄
			(char*)dat, 							//数据首地址
			dwBytesWrite,							//要发送的数据字节数
			&dwBytesWrite, 							//DWORD*，用来接收返回成功发送的数据字节数
			&m_osWrite); 							//NULL为同步发送，OVERLAPPED*为异步发送
		if (!bWriteStat)
		{
			if (GetLastError() == ERROR_IO_PENDING) //如果串口正在写入
			{
				WaitForSingleObject(m_osWrite.hEvent, 1000); //等待写入事件1秒钟
			}
			else
			{
				ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误
				CloseHandle(m_osWrite.hEvent); //关闭并释放hEvent内存
				return 0;
			}
		}
		return datalong;
	}
#endif

}

//关闭串口
void GetLidarData_M10::closeserial()
{
#ifdef LINUX
	::close(fd_);
#else
	HANDLE hCom = *(HANDLE*)pHandle;
	CloseHandle(hCom);
#endif
}

void GetLidarData_M10::LidarStop()
{
	isQuit = true;
	isQuit_com = true;
#ifdef LINUX
	sleep(1);
#else
	Sleep(1000);
#endif
	setLidarRotateState(1);		
}

void GetLidarData_M10::deleteLidarData()
{
	delete callback;
}

void GetLidarData_M10::handleSingleEcho(unsigned char* data)			// 解析 M10 数据
{
	if (data[0] == 0xa5 && data[1] == 0x5a )
	{
		std::vector<MuchLidarData> lidardata_tmp;						//数据点解析结构	
		lidardata_tmp.clear();
		bool isOneFrame = false;
		BlockAngle.clear();
		struct tm t;

		//第一个点的方位角度值
		float AngleBlock = (data[2] * 256 + data[3]) / 100.f;
		if (AngleBlock >= 360.0)
		{
			AngleBlock = AngleBlock - 360;
		}

		//雷达转速
		if ((data[4] * 256 + data[5]) != 0)
			float revolving_Speed = 2500000 / (data[4] * 256 + data[5]);

		int pCount = 0;
		for (int j = 0; j < 42; j++)
		{
			if ((data[6 + 2 * j] * 256 + data[7 + 2 * j]) != 0xffff)
			{
				m_DataT[pCount].Distance = (data[6 + 2 * j] * 256 + data[7 + 2 * j]) *0.001;   //将 mm 转换为 m
				pCount++;
			}
		}

		m_PointXYZ m_point_1;
		for (int z = 0; z < pCount; z++)
		{
			m_DataT[z].H_angle = AngleBlock + 15 / (float)pCount * z;
			if (m_DataT[z].H_angle >= 360)   //属于下一帧数据
			{
				m_DataT[z].H_angle = m_DataT[z].H_angle - 360;
			}

			if (fLastAzimuth > m_DataT[z].H_angle && abs(fLastAzimuth - m_DataT[z].H_angle) > 15)
			{
				m_point_1 = XYZ_calculate(m_DataT[z].H_angle, m_DataT[z].Distance);

				m_DataT[z].X = m_point_1.x;
				m_DataT[z].Y = m_point_1.y;
				m_DataT[z].Z = m_point_1.z;
				lidardata_tmp.push_back(m_DataT[z]);
				isOneFrame = true;
			}
			else
			{
				m_point_1 = XYZ_calculate(m_DataT[z].H_angle, m_DataT[z].Distance);
				m_DataT[z].X = m_point_1.x;
				m_DataT[z].Y = m_point_1.y;
				m_DataT[z].Z = m_point_1.z;
				all_lidardata.push_back(m_DataT[z]);
			}
			fLastAzimuth = m_DataT[z].H_angle;
		}

		if (isOneFrame)
		{
			sendLidarData();

			if (lidardata_tmp.size() > 0)
			{
				for (size_t SF = 0; SF < lidardata_tmp.size(); SF++)
				{
					all_lidardata.push_back(lidardata_tmp[SF]);
				}
			}
		}
	}
}

void GetLidarData_M10::handleSingleEcho_P(unsigned char* data)			// 解析 M10P 数据
{
	if (data[0] == 0xa5 && data[1] == 0x5a)
	{
		//数据帧长度，第一个字节到最后一个字节
		int Length = data[2] * 256 + data[3];    //数据长度不固定
		if (Length > 300)
		{
			Length = 300;
		}
		if (data[Length - 2] == 0xfa && data[Length - 1] == 0xfb)
		{
			std::vector<MuchLidarData> lidardata_tmp;						//数据点解析结构		
			lidardata_tmp.clear();
			bool isOneFrame = false;
			BlockAngle.clear();
			struct tm t;
	
			//第一个点的方位角度值
			float AngleBlock = (data[4] * 256 + data[5]) / 100.f;
			if (AngleBlock >= 360.0)
			{
				AngleBlock = AngleBlock - 360;
			}
	
			//雷达转速
			if ((data[6] * 256 + data[7]) != 0)
				float revolving_Speed = 2500000 / (data[6] * 256 + data[7]);
	
			int pCount = 0;
			for (int j = 0; j < (Length - 20) / 2; j++)
			{
				if (data[8 + 2 * j] & 0x80)					//过滤最高位为1的点云
					continue;
				if (((data[8 + 2 * j] & 0x7f) * 256 + data[9 + 2 * j]) != 0xffff)
				{
					m_DataT[pCount].Distance = ((data[8 + 2 * j] & 0x7f) * 256 + data[9 + 2 * j]) * 0.001;   //将 mm 转换为 m
					m_DataT[pCount].Intensity = (data[8 + 2 * j] & 0x80) == 1 ? 255 : 0;  					 //值为1表示高反，0为低反
					pCount++;
				}
			}
	
	
			m_PointXYZ m_point_1;
			for (int z = 0; z < pCount; z++)
			{
				m_DataT[z].H_angle = AngleBlock + 15 / (float)pCount * z;
				if (m_DataT[z].H_angle >= 360)    //属于下一帧数据
				{
					m_DataT[z].H_angle = m_DataT[z].H_angle - 360;
				}
	
				if (fLastAzimuth > m_DataT[z].H_angle && abs(fLastAzimuth - m_DataT[z].H_angle) > 15)
				{
					m_point_1 = XYZ_calculate(m_DataT[z].H_angle, m_DataT[z].Distance);
	
					m_DataT[z].X = m_point_1.x;
					m_DataT[z].Y = m_point_1.y;
					m_DataT[z].Z = m_point_1.z;
					lidardata_tmp.push_back(m_DataT[z]);
					isOneFrame = true;
				}
				else
				{
					m_point_1 = XYZ_calculate(m_DataT[z].H_angle, m_DataT[z].Distance);
					m_DataT[z].X = m_point_1.x;
					m_DataT[z].Y = m_point_1.y;
					m_DataT[z].Z = m_point_1.z;
					all_lidardata.push_back(m_DataT[z]);
				}
				fLastAzimuth = m_DataT[z].H_angle;
			}
	
			if (isOneFrame)
			{
				sendLidarData();
	
				if (lidardata_tmp.size() > 0)
				{
					for (size_t SF = 0; SF < lidardata_tmp.size(); SF++)
					{
						all_lidardata.push_back(lidardata_tmp[SF]);
					}
				}
			}
		}
	}
}

void GetLidarData_M10::handleSingleEcho_GPS(unsigned char* data)			// 解析 M10GPS 数据
{
	if (data[0] == 0xa5 && data[1] == 0x5a)
	{
		std::vector<MuchLidarData> lidardata_tmp;							//数据点解析结构		
		lidardata_tmp.clear();
		bool isOneFrame = false;
		BlockAngle.clear();
		struct tm t;

		//第一个点的方位角度值
		float AngleBlock = (data[2] * 256 + data[3]) / 100.f;
		if (AngleBlock >= 360.0)
		{
			AngleBlock = AngleBlock - 360;
		}

		//雷达转速
		if ((data[4] * 256 + data[5]) != 0)
			float revolving_Speed = 2500000 / (data[4] * 256 + data[5]);

		int pCount = 0;
		for (int j = 0; j < 42; j++)
		{
			if ((data[6 + 2 * j] * 256 + data[7 + 2 * j]) != 0xffff)
			{
				m_DataT[pCount].Distance = (data[6 + 2 * j] * 256 + data[7 + 2 * j]) * 0.001;  //将 mm 转换为 m
				pCount++;
			}
		}


		tm tm_{};
		static uint64_t nLastPackTimestamp{ 0 };
		tm_.tm_year = data[90] + 100;
		tm_.tm_mon = data[91] - 1;
		tm_.tm_mday = data[92];
		tm_.tm_hour = data[93];
		tm_.tm_min = data[94];
		tm_.tm_sec = data[95];
		tm_.tm_isdst = 0;
		const uint16_t millisecond = data[96] << 8 | data[97];
		const uint16_t microsecond = data[98] << 8 | data[99];
		const time_t timeStamp = mktime(&tm_);
		const auto nPackTimestamp = static_cast<uint64_t>(timeStamp * 1e+9l + millisecond * 1e+6l + microsecond * 1e+3l);
		const uint64_t nDiffTimestamp = (nPackTimestamp - nLastPackTimestamp) / pCount;


		m_PointXYZ m_point_1;
		for (int z = 0; z < pCount; z++)
		{
			m_DataT[z].Mtimestamp_nsce = static_cast<uint64_t>(nPackTimestamp - (pCount - z - 1) * nDiffTimestamp) * 1e-6;

			m_DataT[z].H_angle = AngleBlock + 15 / (float)pCount * z;
			if (m_DataT[z].H_angle >= 360)    //属于下一帧数据
			{
				m_DataT[z].H_angle = m_DataT[z].H_angle - 360;
			}

			if (fLastAzimuth > m_DataT[z].H_angle && abs(fLastAzimuth - m_DataT[z].H_angle) > 15)
			{
				m_point_1 = XYZ_calculate(m_DataT[z].H_angle, m_DataT[z].Distance);

				m_DataT[z].X = m_point_1.x;
				m_DataT[z].Y = m_point_1.y;
				m_DataT[z].Z = m_point_1.z;
				lidardata_tmp.push_back(m_DataT[z]);
				isOneFrame = true;
			}
			else
			{
				m_point_1 = XYZ_calculate(m_DataT[z].H_angle, m_DataT[z].Distance);
				m_DataT[z].X = m_point_1.x;
				m_DataT[z].Y = m_point_1.y;
				m_DataT[z].Z = m_point_1.z;
				all_lidardata.push_back(m_DataT[z]);
			}
			fLastAzimuth = m_DataT[z].H_angle;
			nLastPackTimestamp = nPackTimestamp;
		}

		if (isOneFrame)
		{
			sendLidarData();

			if (lidardata_tmp.size() > 0)
			{
				for (size_t SF = 0; SF < lidardata_tmp.size(); SF++)
				{
					all_lidardata.push_back(lidardata_tmp[SF]);
				}
			}
		}
	}
}


m_PointXYZ GetLidarData_M10::XYZ_calculate(double H_angle, double Distance)
{
	m_PointXYZ point;

	point.x = float(Distance * sin(H_angle * PP / 180));

	point.y = float(Distance * cos(H_angle * PP / 180));

	//point.z = float(Distance);

	return point;
}


#pragma region //设置雷达参数 
bool GetLidarData_M10::setLidarRotateState(int RotateState)	//1停止，0旋转
{
	if (RotateState == 0)
		isMotorRuning = true;
	else
		isMotorRuning = false;
		
	if (m_SerialP)
	{
		unsigned char Rest_UCWP_buff[188];

		Rest_UCWP_buff[0] = 0xA5;
		Rest_UCWP_buff[1] = 0x5A;
		Rest_UCWP_buff[2] = 0x55;
		for (int i = 3; i < 184; i++)
		{
			Rest_UCWP_buff[i] = 0x00;
		}
		if (RotateState == 1)
		{
			Rest_UCWP_buff[184] = 0x03;
			Rest_UCWP_buff[185] = 0x00;
		}
		if (RotateState == 0)
		{
			Rest_UCWP_buff[184] = 0x01;
			Rest_UCWP_buff[185] = 0x01;
		}
		Rest_UCWP_buff[186] = 0xFA;
		Rest_UCWP_buff[187] = 0xFB;
		int sd = 0;
		sd = send((char*)Rest_UCWP_buff, 188);
		if (sd != -1)
		{
			std::cout << "Successfully sent, number of characters sent:" << sd << std::endl;
		}
		else
		{
			std::cout << "Send error" << std::endl;
			return false;
		}

	}
	else
	{
		#ifdef LINUX
		#else
			WSAStartup(MAKEWORD(2, 2), &wsaData);
		#endif
		
		// 创建套接字
		m_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		// 对 sockaddr_in 结构体填充地址、端口等信息
		struct sockaddr_in ServerAddr;
		ServerAddr.sin_family = AF_INET;
		ServerAddr.sin_port = htons(dataPort);
		inet_pton(AF_INET, cpt_ip.c_str(), &ServerAddr.sin_addr);
		int retVal = bind(m_sock, (struct sockaddr*)&ServerAddr, sizeof(ServerAddr));

		unsigned char Rest_UCWP_buff[188];

		Rest_UCWP_buff[0] = 0xA5;
		Rest_UCWP_buff[1] = 0x5A;
		Rest_UCWP_buff[2] = 0x55;
		for (int i = 3; i < 184; i++)
		{
			Rest_UCWP_buff[i] = 0x00;
		}
		if (RotateState == 1)
		{
			Rest_UCWP_buff[184] = 0x03;
			Rest_UCWP_buff[185] = 0x00;
		}
		if (RotateState == 0)
		{
			Rest_UCWP_buff[184] = 0x01;
			Rest_UCWP_buff[185] = 0x01;
		}
		Rest_UCWP_buff[186] = 0xFA;
		Rest_UCWP_buff[187] = 0xFB;

		struct sockaddr_in ServerAddr_send;
		ServerAddr_send.sin_family = AF_INET;
		ServerAddr_send.sin_port = htons(lidarPort);
		inet_pton(AF_INET, cpt_ip_1.c_str(), &ServerAddr_send.sin_addr);
		int sd;
		for (int i = 0; i < 3; i++)
		{
			sd = sendto(m_sock, (const char*)Rest_UCWP_buff, 188, 0, (struct sockaddr*)&ServerAddr_send, sizeof(ServerAddr_send));

		}

		if (sd != -1)
		{
			std::cout << "Successfully sent, number of characters sent:" << sd << std::endl;
			#ifdef LINUX
				usleep(200000);
				close(m_sock);
			#else
				Sleep(200);
				closesocket(m_sock);
			#endif
			
		}
		else
		{
			std::cout << "Send error" << std::endl;
			return false;
		}
	}
	return true;
}

//设置雷达滤波
bool GetLidarData_M10::setLidarFilter(int nFilter)	//0:不滤波,1:正常滤波,2:三米内滤波
{
	LidarStop();
	if (m_SerialP)
	{
		unsigned char Rest_UCWP_buff[188];

		Rest_UCWP_buff[0] = 0xA5;
		Rest_UCWP_buff[1] = 0x5A;
		Rest_UCWP_buff[2] = 0x01;
		for (int i = 3; i < 184; i++)
		{
			Rest_UCWP_buff[i] = 0x00;
		}
		if (nFilter == 0)
			Rest_UCWP_buff[181] = 0x0A;
		else if (nFilter == 1)
			Rest_UCWP_buff[181] = 0x0B;
		else if (nFilter == 2)
			Rest_UCWP_buff[181] = 0x0C;
		if (isMotorRuning == false)
		{
			Rest_UCWP_buff[184] = 0x06;
			Rest_UCWP_buff[185] = 0x00;
		}
		else
		{
			Rest_UCWP_buff[184] = 0x06;
			Rest_UCWP_buff[185] = 0x01;
		}
		Rest_UCWP_buff[186] = 0xFA;
		Rest_UCWP_buff[187] = 0xFB;
		int sd = 0;
		sd = send((char*)Rest_UCWP_buff, 188);
		if (sd != -1)
		{
			std::cout << "Successfully sent, number of characters sent:" << sd << std::endl;
		}
		else
		{
			std::cout << "Send error" << std::endl;
			return false;
		}

	}
	else
	{
		#ifdef LINUX
		#else
			WSAStartup(MAKEWORD(2, 2), &wsaData);
		#endif
		
		// 创建套接字
		m_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		// 对 sockaddr_in 结构体填充地址、端口等信息
		struct sockaddr_in ServerAddr;
		ServerAddr.sin_family = AF_INET;
		ServerAddr.sin_port = htons(dataPort);
		inet_pton(AF_INET, cpt_ip.c_str(), &ServerAddr.sin_addr);
		int retVal = bind(m_sock, (struct sockaddr*)&ServerAddr, sizeof(ServerAddr));

		unsigned char Rest_UCWP_buff[188];

		Rest_UCWP_buff[0] = 0xA5;
		Rest_UCWP_buff[1] = 0x5A;
		Rest_UCWP_buff[2] = 0x55;
		for (int i = 3; i < 184; i++)
		{
			Rest_UCWP_buff[i] = 0x00;
		}
		if (nFilter == 0)
			Rest_UCWP_buff[181] = 0x0A;
		else if (nFilter == 1)
			Rest_UCWP_buff[181] = 0x0B;
		else if (nFilter == 2)
			Rest_UCWP_buff[181] = 0x0C;
		if (isMotorRuning == false)
		{
			Rest_UCWP_buff[184] = 0x06;
			Rest_UCWP_buff[185] = 0x00;
		}
		else
		{
			Rest_UCWP_buff[184] = 0x06;
			Rest_UCWP_buff[185] = 0x01;
		}
		Rest_UCWP_buff[186] = 0xFA;
		Rest_UCWP_buff[187] = 0xFB;

		struct sockaddr_in ServerAddr_send;
		ServerAddr_send.sin_family = AF_INET;
		ServerAddr_send.sin_port = htons(lidarPort);
		inet_pton(AF_INET, cpt_ip_1.c_str(), &ServerAddr_send.sin_addr);
		int sd;
		
		sd = sendto(m_sock, (const char*)Rest_UCWP_buff, 188, 0, (struct sockaddr*)&ServerAddr_send, sizeof(ServerAddr_send));

		if (sd != -1)
		{
			std::cout << "Successfully sent, number of characters sent:" << sd << std::endl;
			#ifdef LINUX
				usleep(200000);
				close(m_sock);
			#else
				Sleep(200);
				closesocket(m_sock);
			#endif
			
		}
		else
		{
			std::cout << "Send error" << std::endl;
			return false;
		}
	}
	LidarStar();
	return true;
}

#pragma endregion 

std::vector<MuchLidarData> GetLidarData_M10::getLidarPerFrameDate()
{
	isFrameOK = false;
	return LidarPerFrameDate;
}

void GetLidarData_M10::sendLidarData()
{
	if (Model == 0)
	{
		all_lidardata.insert(all_lidardata.end(), all_lidardata_d.begin(), all_lidardata_d.end());
		if (callback)
		{
			(*callback)(all_lidardata, 0);
		}
		LidarPerFrameDate = all_lidardata;
	}
	else if (Model == 1)
	{
		if (callback)
		{
			(*callback)(all_lidardata, 0);
		}
		LidarPerFrameDate = all_lidardata;
	}
	else if (Model == 2)
	{
		if (callback)
		{
			(*callback)(all_lidardata_d, 0);
		}
		LidarPerFrameDate = all_lidardata_d;
	}
	isFrameOK = true;
	all_lidardata.clear();
	all_lidardata_d.clear();

}


