#! /bin/bash
echo "export LD_LIBRARY_PATH"
export PPWD=$(cd "$(dirname $0)";pwd)
echo "PPWD $PPWD"


if [ ! -d $PPWD/log/image/ ];then
  echo "image文件夹不存在"
else
  rm -rf $PPWD/log/image/*.png
fi


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


if [ ! -f $PPWD/log/ulog/ulog.log ];then
  echo "ulog.log文件不存在"
else
  rm -rf $PPWD/log/ulog/ulog.log
fi
touch $PPWD/log/ulog/ulog.log

#if [ ! -d $PPWD/log/pcd/ ];then
#  echo "pcd文件夹不存在"
#else
#  rm -rf $PPWD/log/pcd/*.pcd
#fi


if [ ! -f "$PPWD/log/left_side_points.csv" ];then
  echo "left_side_points.csv文件不存在"
else
  rm -rf $PPWD/log/left_side_points.csv
fi
touch $PPWD/log/left_side_points.csv


if [ ! -f "$PPWD/log/right_side_points.csv" ];then
  echo "right_side_points.csv文件不存在"
else
  rm -rf $PPWD/log/right_side_points.csv
fi
touch $PPWD/log/right_side_points.csv


#export LD_LIBRARY_PATH=$(pwd)/extract_libs:$LD_LIBRARY_PATH

export Thirdparty=/home/zyl/3rdparty/echiev/linux-x86-64
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
#/lib:\
#/usr/local/lib:\
#$Thirdparty/opencv/opencv_2.4.13/lib:\

sleep 1

echo "start application......"
chmod +x $PPWD/build/lidar_curb_detection
sudo $PPWD/build/lidar_curb_detection
