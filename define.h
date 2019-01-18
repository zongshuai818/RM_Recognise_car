#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace cv;

#define CONTOURS_MIN 4			//轮廓最小值
#define CONTOURS_MAX 25			//轮廓最大值
#define CONTOURS_RAD_MAX 9		//轮廓最大长宽比
#define CONTOURS_RAD_MIN 2		//轮廓最小长宽比
#define CONTOURS_WIDTH_MIN 20	//装甲灯条宽度最小值
#define CONTOURS_WIDTH_MAX 80	//装甲灯条宽度最大值
#define CONTOURS_HEIGHT_DIF 15	//装甲灯条高度差
#define CONTOURS_ANGLE_DIF 8	//装甲灯条角度差
#define CONTOURS_ANGLE 15		//轮廓最大旋转角
#define HISTORY_NUM 5			//用于预测的历史容量

//灯条连线结构
struct targetLine
{
	Point one;
	Point two;
	int distanceX;
	int distanceY;
	double difAngle;
};