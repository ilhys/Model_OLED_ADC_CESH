#include "pidz.h"

PID_InitTypeDef pid;

void pid_init(void)
{
    pid.Setval=0.0;
    pid.Actualval=0;
	  //pid.limit=0;   
    pid.err=0;
	  pid.err_next=0;
	  pid.err_last=0;
    pid.kp=0.25;
    pid.ki=0.015;
    pid.kd=0.2;
}

float realize(float sv,float pv)
{
  pid.Setval=sv;
	pid.Actualval=pv;
	pid.err=pid.Setval-pid.Actualval;
	pid.Actualval+=pid.kp*(pid.err-pid.err_next)+pid.ki*pid.err+pid.kd*(pid.err-2*pid.err_next+pid.err_last);
	pid.err_last=pid.err_next;
	pid.err_next=pid.err;
	return pid.Actualval;
}

