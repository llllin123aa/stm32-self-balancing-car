#ifndef __PID_H
#define __PID_H

typedef struct {
	float Target;
	float Actual;
	float Out;
	float Actual1;//上次的实际值
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float OutMax;
	float OutMin;
	float OutOffset;
	float ErrorIntMax;
	float ErrorIntMin;
} PID_t;

void PID_Update(PID_t *p);
void PID_Init(PID_t *p);
#endif
