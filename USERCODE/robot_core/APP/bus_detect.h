#ifndef _BUS_DETECT_H
#define _BUS_DETECT_H

#include "drv_conf.h"
#include HAL_INCLUDE

#define CHASSIS_INDEX 0
#define GIMBAL_INDEX 1
#define SHOOT_INDEX 2

void detect_loop(void);

void disable_all_it(void);

void mpu_nx_protect(void);

typedef struct
{
    float data_last;
    uint32_t tick_start;
    float refresh_rate;
} rfreq_t;
float update_data_refresh_freq(rfreq_t *s_rf, float data_curr);

#define get_data_refresh_freq(_DAT, freq)              \
    do                                                 \
    {                                                  \
        static rfreq_t rfreq = {0};                    \
        freq = update_data_refresh_freq(&rfreq, _DAT); \
    } while (0)
/*
READ_BIT(RCC->CSR, Flags) == Flags
Flags: 
RCC_CSR_LPWRRSTF   < Low-Power reset flag 
RCC_CSR_PINRSTF    < PIN reset flag 
RCC_CSR_PORRSTF    < POR/PDR reset flag 
RCC_CSR_SFTRSTF    < Software Reset flag 
RCC_CSR_IWDGRSTF   < Independent Watchdog reset flag 
RCC_CSR_WWDGRSTF   < Window watchdog reset flag 
RCC_CSR_BORRSTF    < BOR reset flag (Brown-out reset,Ƿѹ��λ)

*/
inline uint32_t RCC_FLAG(uint32_t flags){
    return (READ_BIT(RCC->CSR, flags) == flags);
}

//#define IS_POR (RCC_FLAG(RCC_CSR_PORRSTF))
//#define IS_IWDG_RST (RCC_FLAG(RCC_CSR_IWDGRSTF))
#define _DEBUG 1

#if _DEBUG == 1
#define FAULT_HANDLER __BKPT(0xbe);
#else
#define FAULT_HANDLER __set_FAULTMASK(1);NVIC_SystemReset();
#endif

#endif

