#ifndef _PIDZ_H
#define _PIDZ_H

typedef struct
{
  float Setval;
  float Actualval;
	float limit;
  float err;
  float err_next;
  float err_last;	
  float kp,ki,kd;
}PID_InitTypeDef;

void pid_init(void);
float realize(float sv,float pv);

#endif

