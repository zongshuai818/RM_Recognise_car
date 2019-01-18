#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace cv;

#define CONTOURS_MIN 4			//������Сֵ
#define CONTOURS_MAX 25			//�������ֵ
#define CONTOURS_RAD_MAX 9		//������󳤿��
#define CONTOURS_RAD_MIN 2		//������С�����
#define CONTOURS_WIDTH_MIN 20	//װ�׵��������Сֵ
#define CONTOURS_WIDTH_MAX 80	//װ�׵���������ֵ
#define CONTOURS_HEIGHT_DIF 15	//װ�׵����߶Ȳ�
#define CONTOURS_ANGLE_DIF 8	//װ�׵����ǶȲ�
#define CONTOURS_ANGLE 15		//���������ת��
#define HISTORY_NUM 5			//����Ԥ�����ʷ����

//�������߽ṹ
struct targetLine
{
	Point one;
	Point two;
	int distanceX;
	int distanceY;
	double difAngle;
};