#ifndef RAMP_H_
#define RAMP_H_

typedef struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
//斜波函数计算
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

#endif // !RAMP_H_
