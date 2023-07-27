//https://blog.csdn.net/black_sneak/article/details/129629485  滤波来源
#include"main.h"
#include"lvbo.h"
extern uint32_t ADC_Value[ADC_SIZE];

//中值滤波算法
int middleValueFilter(int N)
{
    int value_buf[N];
    int i,j,k,temp;
    for( i = 0; i < N; ++i)
    {
        // value_buf[i] = HAL_ADC_GetValue(&hadc1);	
				
    }
    for(j = 0 ; j < N-1; ++j)
    {
        for(k = 0; k < N-j-1; ++k)
        {
            //从小到大排序，冒泡法排序
            if(value_buf[k] > value_buf[k+1])
            {
                temp = value_buf[k];
                value_buf[k] = value_buf[k+1];
                value_buf[k+1] = temp;
            }
        }
    }
		
    return value_buf[(N-1)/2];
}


////算术平均值滤波
//int averageFilter(int N)
//{
//   int sum = 0;
//   short i;
//   for(i = 0; i < N; ++i)
//   {
//        // sum += HAL_ADC_GetValue(&hadc1);	
//   }
//   return sum/N;
//}

//滑动均值滤波
#define MAF_LENGTH 5 // 滑动平均滤波器长度
float MovingAverageFilter(float xi)
{
  static int8_t MAF_index = -1;
  static float buffer[MAF_LENGTH];
  static float yk_1 = 0;
  float yk = 0;
  uint8_t i = 0;
  if (MAF_index == -1)
  { // 初始化，数据未填满时，前面的数据不滤波
    for (i = 0; i < MAF_LENGTH; i++)
    {
      buffer[i] = xi;
    }
    yk_1 = xi;
    MAF_index = 0;
    yk = yk_1;
  }
  else
  {//滤波
    yk = yk_1 + (xi - buffer[MAF_index]) / MAF_LENGTH;
    buffer[MAF_index] = xi;
    MAF_index++;
    if (MAF_index >= MAF_LENGTH)
    {
      MAF_index = 0;
    }
	yk_1 = yk;
  }
  return yk;
}

//卡尔曼滤波
int KalmanFilter(int inData)
{
		static float prevData = 0;                                 //先前数值
		static float p = 10, q = 0.001, r = 0.001, kGain = 0;      // q控制误差  r控制响应速度 
	
		p = p + q;
		kGain = p / ( p + r );                                     //计算卡尔曼增益
		inData = prevData + ( kGain * ( inData - prevData ) );     //计算本次滤波估计值
		p = ( 1 - kGain ) * p;                                     //更新测量方差
		prevData = inData;
		return inData;                                             //返回滤波值
}

//限幅平均滤波
#define A 50        //限制幅度阈值
#define M 12
int data[M];
int First_flag=0;
int LAverageFilter(void)
{
  int i;
  int temp,sum,flag=0;
  // data[0]=HAL_ADC_GetValue(&hadc1);
	for(i=1;i<M;i++)
	{
		// temp=HAL_ADC_GetValue(&hadc1);
		if((temp-data[i-1])>A||((data[i-1]-temp)>A))
		{
		  i--;flag++;
		}
		else
		{
			data[i]=temp;
		}
	}
  for(i=0;i<M;i++)
  {
    sum+=data[i];
  } 
  return  sum/M;
}

//均值输入缓冲输出 算术平均值滤波
#define averageFilter_size 100 
float averageFilter(float eff)
{
	static uint16_t average_index=0;
	static float average_num=0;
	average_num*=average_index;
	average_index++;
    average_num+=eff;
	average_num/=average_index;
	if(average_index == averageFilter_size )   {average_index=0;}
	return average_num;
}
