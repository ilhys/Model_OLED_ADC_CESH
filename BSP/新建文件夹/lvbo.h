#ifndef __LVBO_H__
#define __LVBO_H__

//中值滤波算法
int middleValueFilter(int N);

//算术平均值滤波
//int averageFilter(int N);
float averageFilter(float eff);

//滑动均值滤波
float MovingAverageFilter(float xi);

//卡尔曼滤波
int KalmanFilter(int inData);

//限幅平均滤波
int LAverageFilter(void);



#endif
