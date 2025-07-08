#ifndef _IMU_BMI088_H
#define _IMU_BMI088_H

/*bmi088 offical api support: https://github.com/BoschSensortec/BMI08x-Sensor-API */
/*download files and the following change include path*/
#include "Devices/BMI088/drv_bmi088.h"
#include "Devices/BMI088/bmi08x.h"

#include "drv_conf.h"
#include "algorithm/imu_fusion.h"
#include "algorithm/imu_dcm.h"
#include "algorithm/imu_calibrate.h"

#define IMU_RANGE (4096-(-4096))

#define ACCEL_RANGE_G 3
#define GYRO_RANGE_DPS 1000

//import motion fx and motion_fx_manager.c first
#define USE_MOTION_FX 0

#define IMU_DATA_NOT_RDY -53
#define IMU_CONF_ERR -47
#define IMU_ACCEL_ERR -24

int8_t init_bmi08x(void);
int8_t enable_bmi08x_interrupt(void);
int8_t disable_bmi08x_interrupt(void);

int8_t get_imu_att(imu_data_fp_t *_imu_fp, eular_t *eular, float *temperature);
int8_t get_imu_data_raw(imu_data_raw_t* _imu, imu_data_fp_t* _imu_fp);

#if USE_MOTION_FX == 1
int8_t mfx_get_imu_att(imu_data_fp_t* _imu_fp, eular_t* eular);
#endif

#endif

