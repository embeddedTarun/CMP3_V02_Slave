/*
 * l6470.h
 *
 *  Created on: Jan 13, 2024
 *      Author: IT
 */

#ifndef INC_L6470_H_
#define INC_L6470_H_
#include "main.h"
#include "stm32f1xx_hal.h"  // Adjust for your STM32 family
///#include "stm32f1xx_hal_spi.h"

extern SPI_HandleTypeDef hspi1;


#define MAX_STEPS 4194175 //4194303
#define MICROSTEPS_PER_STEP 128
//extern uint32_t total_revolutions;
//extern uint32_t total_steps;


//#define SPI1_SS_Pin GPIO_PIN_2
//#define SPI1_SS_GPIO_Port GPIOA

#define SetParam_CMD		(0x00)
#define GetParam_CMD        (0x20)
#define RUN_CMD 		    (0x50)
#define STEPCLOCK_CMD       (0x58)
#define MOVE_CMD 		    (0x40)
#define GO_TO_CMD           (0x60)
#define GO_TO_DIR_CMD 		(0x68)
#define GO_UNTIL_CMD        (0x82)
#define RELEASE_SW_CMD 		(0x92)
#define GO_HOME_CMD         (0x70)
#define GO_MARK_CMD 		(0x78)
#define RESET_POS_CMD       (0xD8)
#define RESET_DEVICE_CMD 	(0xC0)
#define SOFT_STOP_CMD       (0xB0)
#define HARD_STOP_CMD 		(0xB8)
#define SOFT_HIZ_CMD        (0xA0)
#define HARD_HIZ_CMD 		(0xA8)
#define Get_STATUS_CMD      (0xD0)


#define L6470_ABS_POS         (0x01)
#define L6470_EL_POS          (0x02)
#define L6470_MARK            (0x03)
#define L6470_SPEED           (0x04)
#define L6470_ACC             (0x05)
#define L6470_DEC             (0x06)
#define L6470_MAX_SPEED       (0x07)
#define L6470_MIN_SPEED       (0x08)
#define L6470_FS_SPD          (0x15)
#define L6470_KVAL_HOLD       (0x09)
#define L6470_KVAL_RUN        (0x0A)
#define L6470_KVAL_ACC        (0x0B)
#define L6470_KVAL_DEC        (0x0C)
#define L6470_INT_SPD         (0x0D)
#define L6470_ST_SLP          (0x0E)
#define L6470_FN_SLP_ACC      (0x0F)
#define L6470_FN_SLP_DEC      (0x10)
#define L6470_K_THERM         (0x11)
#define L6470_ADC_OUT         (0x12)
#define L6470_OCD_TH          (0x13)
#define L6470_STALL_TH        (0x14)
#define L6470_STEP_MODE       (0x16)
#define L6470_ALARM_EN        (0x17)
#define L6470_CONFIG_A 		  (0x18)
#define L6470_STATUS_A        (0x19)



#define STEP_S_MAX 		(15610)
#define SPEED_MAX 			(0xFE11A)  // (1040666)in step/s

typedef enum resister_length
{
	L6470_ABS_POS_lEN		=	22,
	L6470_EL_POS_LEN		=	9,
	L6470_MARK_LEN 			=	22,
	L6470_SPEED_LEN			=	20,
	L6470_ACC_LEN			=	12,
	L6470_DEC_LEN			=	12,
	L6470_MAX_SPEED_LEN		=	10,
	L6470_MIN_SPEED_LEN		=	13,
	L6470_FS_SPD_LEN		=	10,
	L6470_KVAL_HOLD_LEN		=	8,
	L6470_KVAL_RUN_LEN		=	8,
	L6470_KVAL_ACC_LEN		=	8,
	L6470_KVAL_DEC_LEN		=	8,
	L6470_INT_SPD_LEN		=	14,
	L6470_ST_SLP_LEN		=	8,
	L6470_FN_SLP_ACC_LEN	=	8,
	L6470_FN_SLP_DEC_LEN	=	8,
	L6470_K_THERM_LEN		=	4,
	L6470_ADC_OUT_LEN		=	5,
	L6470_OCD_TH_LEN		=	4,
	L6470_STALL_TH_LEN		=	7,
	L6470_STEP_MODE_LEN		=	8,
	L6470_ALARM_EN_LEN		=	8,
	L6470_CONFIG_A_LEN		=	16,
	L6470_STATUS_A_LEN		=	16,
	L6470_MAX_LEN			=	22
} register_length_t;


typedef enum {BACKWARD_DIR,FORWARD_DIR} motor_direction_t;
typedef enum {ACT0=0,ACT1=8} act_state_t;






void Spi_init();

 void Spi_transmit(uint8_t byte);
 void Spi_recieve(uint8_t byte[],uint8_t size);

 uint32_t Param(uint32_t value,  register_length_t bit_len);
 uint32_t Param_handler(uint32_t param , uint32_t value );


 void Set_param(uint32_t parameter, uint32_t command,register_length_t length);


 uint8_t get_param_handler(uint8_t parameter);
 uint32_t Get_param(uint8_t parameter);


 uint32_t step_s_2_Speed(uint32_t step_s);

 void Run(motor_direction_t dir, uint32_t speed);

 void Move(motor_direction_t dir,uint32_t N_step);

 void GO_TO(uint32_t Abs_Pos);

 void GO_TO_DIR(motor_direction_t dir,uint32_t Abs_Pos);


 void GO_UNTIL(act_state_t act ,motor_direction_t dir,uint32_t speed);

 void GO_HOME(void);

 void GO_MARK(void);

 void RESET_POS(void);

 void RESET_DEVICE(void);

 void SOFT_STOP(void);

 void HARD_STOP(void);

 void SOFT_HIZ(void);

 void HARD_HIZ(void);

 uint32_t GET_STATUS(void);

 unsigned long Read_total_steps();




#endif /* INC_L6470_H_ */
