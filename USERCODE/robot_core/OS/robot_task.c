/**
 * @file robot_task.c
 * @author ttf
 * @brief 操作系统任务调度仪表盘
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "robot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "APP/robot.h"
#include "APP/graphic_draw.h"
#include "self_delay.h"
//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		250
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

#define IMU_TASK_PRIO		1	
#define IMU_STK_SIZE 		250
TaskHandle_t IMUTask_Handler;
void IMU_task(void *pvParameters);

#define H_TIM_TASK_PRIO		2	
#define H_TIM_STK_SIZE 		100
TaskHandle_t H_TIM_Task_Handler;
void H_TIM_task(void *pvParameters);

#define RGB_TASK_PRIO		2	
#define RGB_STK_SIZE 		100
TaskHandle_t RGBTask_Handler;
void RGB_task(void *pvParameters);

#define DETECT_TASK_PRIO		3	
#define DETECT_STK_SIZE 		100
TaskHandle_t DETECTTask_Handler;
void DETECT_task(void *pvParameters);

#define M_TIM_TASK_PRIO		1	
#define M_TIM_STK_SIZE 		200
TaskHandle_t M_TIM_Task_Handler;
void M_TIM_task(void *pvParameters);

#define REFEREE_TASK_PRIO		3	
#define REFEREE_STK_SIZE 		100
TaskHandle_t REFEREETask_Handler;
void REFEREE_task(void *pvParameters);

#define UI_TASK_PRIO		3	
#define UI_STK_SIZE 		100
TaskHandle_t UITask_Handler;
void UI_task(void *pvParameters);

#define CANCOMMUNI_TASK_PRIO		2	
#define CANCOMMUNI_STK_SIZE 		350
TaskHandle_t CANCOMMUNITask_Handler;
void CANCOMMUNI_task(void *pvParameters);

#define NAVI_TASK_PRIO		2	
#define NAVI_STK_SIZE 		350
TaskHandle_t NAVITask_Handler;
void NAVI_task(void *pvParameters);

uint16_t task_stack_available[6] = {0};

void task_create(void){
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}
//开始任务的任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
      
    xTaskCreate((TaskFunction_t )IMU_task,     
                (const char*    )"IMU_task",   
                (uint16_t       )IMU_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )IMU_TASK_PRIO,
                (TaskHandle_t*  )&IMUTask_Handler); 
								
    xTaskCreate((TaskFunction_t )H_TIM_task,     
                (const char*    )"H_TIM_task",   
                (uint16_t       )H_TIM_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )H_TIM_TASK_PRIO,
                (TaskHandle_t*  )&H_TIM_Task_Handler);
								
    xTaskCreate((TaskFunction_t )RGB_task,     
                (const char*    )"RGB_task",   
                (uint16_t       )RGB_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )RGB_TASK_PRIO,
                (TaskHandle_t*  )&RGBTask_Handler);
								
    xTaskCreate((TaskFunction_t )DETECT_task,     
                (const char*    )"DETECT_task",   
                (uint16_t       )DETECT_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )DETECT_TASK_PRIO,
                (TaskHandle_t*  )&DETECTTask_Handler);
								
    xTaskCreate((TaskFunction_t )M_TIM_task,     
                (const char*    )"M_TIM_task",   
                (uint16_t       )M_TIM_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )M_TIM_TASK_PRIO,
                (TaskHandle_t*  )&M_TIM_Task_Handler);
								
//		xTaskCreate((TaskFunction_t )UI_task,     
//                (const char*    )"UI_task",   
//                (uint16_t       )UI_STK_SIZE, 
//                (void*          )NULL,
//                (UBaseType_t    )UI_TASK_PRIO,
//                (TaskHandle_t*  )&UITask_Handler);
								
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区												
								
}

// portTICK_RATE_MS 为1000ms
TickType_t xTimeIncrement_t;
void IMU_task(void *pvParameters){
	
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
    for( ;; ){		
        robot_imu_tim_cycle();//T
			delay_us(800);//1250HZ
     // vTaskDelayUntil(&xLastWakeTime, 1/portTICK_RATE_MS); //1kHz
			task_stack_available[0] = uxTaskGetStackHighWaterMark(IMUTask_Handler);
    }
}

void H_TIM_task(void *pvParameters){
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; ){		
				robot_h_tim_cycle(); //T
        vTaskDelayUntil(&xLastWakeTime, 1/portTICK_RATE_MS); //1000Hz
			task_stack_available[1] = uxTaskGetStackHighWaterMark(H_TIM_Task_Handler);
    }
}

void RGB_task(void *pvParameters){
	
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; ){		
				RGB_loop(8,0.5);//T
        vTaskDelayUntil(&xLastWakeTime, 125/portTICK_RATE_MS); //8Hz
			task_stack_available[2] = uxTaskGetStackHighWaterMark(RGBTask_Handler);
    }
}

void DETECT_task(void *pvParameters){
	
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; ){		
				diag_slow_time_cycle();//T
        vTaskDelayUntil(&xLastWakeTime, 100/portTICK_RATE_MS); //10Hz
			task_stack_available[3] = uxTaskGetStackHighWaterMark(DETECTTask_Handler);
    }
}

void M_TIM_task(void *pvParameters){
	
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; ){		
				robot_m_tim_cycle();//T
        vTaskDelayUntil(&xLastWakeTime, 5/portTICK_RATE_MS); //200Hz
			task_stack_available[4] = uxTaskGetStackHighWaterMark(M_TIM_Task_Handler);
    }
}

extern UART_HandleTypeDef huart6;
void REFEREEask(void *pvParameters){
	
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; ){		
				//send_buy_all_bullet(&ext_sentry_buy_bullet,&huart6);
        vTaskDelayUntil(&xLastWakeTime, 2/portTICK_RATE_MS); //200Hz
    }
}

void UI_task(void *pvParameters){
	
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for( ;; ){		
				UI_update();//T
        vTaskDelayUntil(&xLastWakeTime, 100/portTICK_RATE_MS); //10Hz
			task_stack_available[5] = uxTaskGetStackHighWaterMark(UITask_Handler);
    }
}

//void CANCOMMUNI_task(void *pvParameters){
//	
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
//    for( ;; ){		
//				CAN_ask_cycle();
//        vTaskDelayUntil(&xLastWakeTime, 5/portTICK_RATE_MS); //200Hz
//    }
//}

//void NAVI_task(void *pvParameters){
//	
//    portTickType xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
//    for( ;; ){		
//				NAVI_ask_cycle();
//        vTaskDelayUntil(&xLastWakeTime, 2/portTICK_RATE_MS); //500hz
//    }
//}


