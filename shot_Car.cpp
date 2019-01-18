#运行环境：opencv3

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <iostream>
#include "define.h"

using namespace std;
using namespace cv;

#ifndef DEBUG_LINE
#define DEBUG_LINE
#endif

#ifndef DEBUG_IMAGE
//#define DEBUG_IMAGE
#endif

#ifndef DEBUG_TEXT
//#define DEBUG_TEXT
#endif

//平均预测方法
cv::Point mine_predict(vector<Point> input);
//最近预测方法
cv::Point near_predict(vector<Point> input);

int main(int argc, const char *argv[])
{
	VideoCapture capture("V23..avi");
	//检测摄像头是否打开成功
	if (!capture.isOpened()){cout << "读摄像头有误" << endl;return -1;}

	vector<Point> history;

	while (true)
	{
		Mat src;
		capture>>src;
		//src=imread("bugimg/233.jpg");
		Mat hsv_img,gray_img;
		cvtColor(src,gray_img,CV_BGR2GRAY);
		threshold(gray_img,gray_img,100,255,CV_THRESH_BINARY);

#ifdef DEBUG_IMAGE
		imshow("第一步二值化",gray_img);
#endif // DEBUG

		Mat element0 = getStructuringElement(MORPH_RECT, Size(3, 3));
		morphologyEx(gray_img,gray_img,MORPH_CLOSE, element0);

#ifdef DEBUG_IMAGE
		imshow("闭运算",gray_img);
#endif // DEBUG
		GaussianBlur(gray_img,gray_img,Size(5,5),1);
		threshold(gray_img,gray_img,220,255,CV_THRESH_BINARY);
		dilate(gray_img,gray_img,element0);

#ifdef DEBUG_IMAGE
		imshow("高斯滤波+二值化+膨胀",gray_img);
#endif // DEBUG

		Mat working;
		gray_img.copyTo(working);

		//提取轮廓
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(working, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		//寻找合适的轮廓
		RotatedRect tmp;
		vector<RotatedRect> target;
		for(int i=0;i<contours.size();i++)
		{
			/*/debug counter
			tmp=minAreaRect(contours[i]);
			Point2f vertices2[4]; 
			tmp.points(vertices2);  
			for (int k = 0; k < 4; k++)  
				line(src, vertices2[k], vertices2[(k+1)%4], Scalar(255,0,255),2);
			/*/

			if(contours[i].size()<CONTOURS_MIN||contours[i].size()>CONTOURS_MAX)
				continue;
			tmp=minAreaRect(contours[i]);
			//长宽比
			Size2f s=tmp.size;
			float s_h_tmp= s.height/s.width;
			s_h_tmp=s_h_tmp < 1 ? 1/s_h_tmp : s_h_tmp;
			if (s_h_tmp<CONTOURS_RAD_MIN||s_h_tmp>CONTOURS_RAD_MAX)
				continue;
			//角度计算,有点小难搞,这个角度横竖都可能是-90左右
			double angle = tmp.angle;
			if (angle==0)
				angle=90;
			bool fu= angle<0;
			bool fuMax= (abs(angle)<(90+CONTOURS_ANGLE)) 
				&& (abs(angle)>(90-CONTOURS_ANGLE));
			bool fuMin= abs(angle)<CONTOURS_ANGLE;
			//角度筛选
			if(!fuMax && !fuMin)
				continue;
			//保留到最后的进入候选矩形
			target.push_back(tmp);
#ifdef DEBUG_TEXT
			Scalar color(rand() & 255, rand() & 255, rand() & 255);
			char slope_str[15];
			//sprintf(slope_str, "%.1f", angle);
			putText(src, slope_str, contours[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 1.5, color, 1);
			Point2f vertices[4];  
			tmp.points(vertices);  
			for (int j = 0; j < 4; j++)  
				line(src, vertices[j], vertices[(j+1)%4], Scalar(255,0,255),2);
#endif//画出轮廓和对应的角度,画出满足条件的矩形
		}
		//利用目标矩形连成线,得到候选线结构
		//这个方法复杂度为n^2，如果前期筛选不好会比较耗时
		vector<struct targetLine> targetline;
		targetLine tL;
		for(int i=0;i<target.size();i++)
		{
			for(int j=i+1;j<target.size();j++)
			{
				tL.one=target[i].center;
				tL.two=target[j].center;
				tL.difAngle = abs(target[i].angle-target[j].angle);
				tL.difAngle = cv::min(abs(tL.difAngle-90),tL.difAngle);
				tL.distanceX= abs(target[i].center.x-target[j].center.x);
				tL.distanceY= abs(target[i].center.y-target[j].center.y);
				targetline.push_back(tL);
			}
		}
		//得到真正的目标，这里的方法还可增加模板匹配装甲，不然精度不够
		Point aim(0,0);
		for(int i=0;i<targetline.size();i++)
		{
			if(targetline[i].distanceY<CONTOURS_HEIGHT_DIF
				&& targetline[i].distanceX<CONTOURS_WIDTH_MAX
				&& targetline[i].distanceX>CONTOURS_WIDTH_MIN
				&& targetline[i].difAngle<CONTOURS_ANGLE_DIF)
			{
				aim.x=(targetline[i].one.x+targetline[i].two.x)/2;
				aim.y=(targetline[i].one.y+targetline[i].two.y)/2;
				break;
			}
		}

#ifdef DEBUG_LINE
		line(src,Point(0,aim.y),Point(640,aim.y),Scalar(255,0,0),2);
		line(src,Point(aim.x,0),Point(aim.x,480),Scalar(255,0,0),2);
#endif//画出目标标线

		//如果没有目标或者目标太跳跃
		//用过去5次的目标位置，二次函数拟合预测下一次的目标位置
		//保证容器中有5个过去的位置
		history.push_back(aim);
		if(history.size()>HISTORY_NUM)
		{
			vector<Point>::iterator k = history.begin();
			history.erase(k);//删除第一个元素
		}
		//预测
		if(aim.x==0)
			;
			//aim=near_predict(history);
		//没有目标
		if(aim.x==0)
		{
			string slope_str="NONE AIM";
			putText(src, slope_str, Point(20,40), CV_FONT_HERSHEY_COMPLEX_SMALL, 1.5, Scalar(0,0,255), 1);
			//保存没有识别到的图片
			char name[10];
			string format=".jpg";
			string doc="bugimg/";
			static int i=0;
			//sprintf(name,"%d",i);
			//string imgname=doc+name+format;
			//imwrite(imgname,src);
			//i++;
		}
		else
			circle(src,aim,6,Scalar(0,0,255),-1);
		imshow("src",src);
		waitKey(30);
	}

	return 0;
}

//================================================
//	Predictor
//================================================
//平均预测方法
cv::Point mine_predict(vector<Point> input)
{
	vector<Point>::iterator k = input.begin();
	Point out(0,0);
	int num=0;
	for(;k!=input.end()&&(*k).x!=0;k++)
	{
		out.x+=(*k).x;
		out.y+=(*k).y;
		num++;
	}
	num = num==0 ? 1 : num;
	out.x/=num;
	out.y/=num;
	return out;
}

//最近预测方法
cv::Point near_predict(vector<Point> input)
{
	vector<Point>::iterator k = input.begin();
	Point out(0,0);
	for(;k!=input.end()&&(*k).x!=0;k++)
	{
		out.x=(*k).x;
		out.y=(*k).y;
	}
	return out;
}


//录视频

//int main()
//{
//	VideoCapture capture(0);//如果是笔记本，0打开的是自带的摄像头，1 打开外接的相机
//	double rate = 25.0;//视频的帧率
//	Size videoSize(640, 480);
//	VideoWriter writer("V23..avi", CV_FOURCC('M', 'J', 'P', 'G'), rate, videoSize);
//	Mat frame;
//
//	while (capture.isOpened())
//	{
//		capture >> frame;
//		writer << frame;
//		imshow("video", frame);
//		if (waitKey(20) == 27)//27是键盘摁下esc时，计算机接收到的ascii码值
//		{
//			break;
//		}
//	}
//	return 0;
//}
