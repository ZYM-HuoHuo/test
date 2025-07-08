#ifndef SMC_H_
#define SMC_H_

typedef enum{
    UNI_LAW = 0u,
    EXPON_LAW,
    POW_LAW
}smc_law_t;

typedef struct
{
    float err[2];
    float s;
    float s_dot;
    float u;

    float c;
    
    float epsilon;
    float k;
    float alpha; 
    
}smc_one_t;

float smc_calc(smc_one_t *smc, float ref, float cur, smc_law_t LAW);
float smc_dual_calc(smc_one_t *smc, float delta, float ddelta, smc_law_t LAW);
#endif // !SMC_H_
