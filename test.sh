#! /bin/bash
echo "export LD_LIBRARY_PATH"
export PPWD=$(cd "$(dirname $0)";pwd)
echo "PPWD $PPWD"


if [ ! -d $PPWD/log/image/ ];then
  echo "image文件夹不存在"
else
  rm -rf $PPWD/log/image/*.png
fi

#sleep 1

if [ ! -d $PPWD/log/temp_left_side_image/ ];then
  echo "temp_left_side_image文件夹不存在"
else
  rm -rf $PPWD/log/temp_left_side_image/*.png
fi


if [ ! -d $PPWD/log/temp_right_side_image/ ];then
  echo "temp_right_side_image文件夹不存在"
else
  rm -rf $PPWD/log/temp_right_side_image/*.png
fi

#export LD_LIBRARY_PATH=$(pwd)/extract_libs:$LD_LIBRARY_PATH

export Thirdparty=/home/zyl/3rdparty/echiev/linux-x86-64
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
#/lib:\
#/usr/local/lib:\
#$Thirdparty/opencv/opencv_2.4.13/lib:\



echo "start application......"
chmod +x $PPWD/build/lidar_curb_detection
sudo $PPWD/build/lidar_curb_detection
