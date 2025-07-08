#include "smc.h"
#include <math.h>
#define sgn(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
float smc_calc(smc_one_t *smc, float e, float e_dot, smc_law_t LAW){
    smc->err[0] = e;
    smc->err[1] = e_dot;
    smc->s = smc->c*smc->err[0] + smc->err[1];
    smc->epsilon = smc->epsilon>0?smc->epsilon:0;
    smc->k  = smc->k>0?smc->k:0;
    smc->alpha = smc->alpha > 0?(smc->alpha>1?1:smc->alpha):0;
    switch (LAW){
        case UNI_LAW:
        smc->s_dot = -smc->epsilon * sgn(smc->s);
        break;
        case EXPON_LAW:
        smc->s_dot = -smc->epsilon * sgn(smc->s) - smc->k * smc->s;
        break;
        case POW_LAW:
        smc->s_dot = -smc->k * powf(fabsf(smc->s), smc->alpha);
        break;
        default:
        break;
    }
    smc->u = smc->s_dot - smc->c * smc->err[1];
    return smc->u;
}
float smc_dual_calc(smc_one_t *smc, float delta, float ddelta, smc_law_t LAW);
