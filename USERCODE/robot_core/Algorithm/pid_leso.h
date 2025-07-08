#ifndef _PID_LESO_H
#define _PID_LESO_H
#ifdef __cplusplus
extern "C" {
#endif
#include "pid.h"

typedef struct
{
	float h;
	float beta1,beta2,beta3;
	float a0,b0;
	float af_z1,af_z2,b1;
	/* LESO */
	float z1,z2,z3;
}leso_para_t;	/* Linear ESO */

float pid_leso_dualloop(pid_struct_t pid[2], leso_para_t* leso, float err, float err_out, float feed);

void leso_6020_init(leso_para_t* leso, float h,float b1, float _J);
void update_leso(leso_para_t* leso, float y,float u);
void reset_leso(leso_para_t* leso);
#ifdef __cplusplus
}
#endif
#endif
