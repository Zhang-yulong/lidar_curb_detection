# Lidar detects the edge of the road or the wall

## Building
* 1.根据test.sh和CMakeLists.txt的内容创建对应文件夹

## 

* 1.最后还是使用正交最小二乘法。y=kx+b对于垂直线效果不太好。

### 2025-2-8
1. 上一版本平均滤波遇到的bug，查出来是 放在每段`if块开头的pResult->points.clear();`引起的。

   原因：

   1.（初步猜想）应该是上一段`pResult = m_pCurveFitting->CurveFittingStart(pLastestLeftCluster,ullTime,1);`和`m_qLeftLineInfo.push(left_line_running_info);`和下一段的`pResult->points.clear();`公用一块内存地址？导致pResult的值并没有push进队列。解决方法：将下一段的`pResult->points.clear();`屏蔽。

   2.发现在滤波程序中`pResult = m_pCurveFitting->CurveFittingStart(pLastestLeftCluster,ullTime,1);`和`pResult = SlideWindowProcess(m_qLeftLineInfo);`被两次赋值，可能会导致队列中数据丢失。解决方法：每段if中，添加一个中间变量保存`CurveFittingStart`函数的结果，再给`SlideWindowProcess`函数使用。**（本版本使用这个方法）**

   

### 2025-1-5
1. 该版本可通过yaml文件调整聚类时的投影矩阵
2. 该版本目前还存在使用滤波时，左右两边输出的横向值有问题，取决于滤波过程中，哪边道路的数据后被使用，那么先前一边的数据就会有问题。有待下一个版本解决