/*
 * l6470.h
 *
 *  Created on: Jan 13, 2024
 *      Author: Tarun Kaushik
 */

//#include "stm32f1xx_hal.h"

#include "l6470.h"
#include "string.h"

 uint32_t total_revolutions=0;
 uint32_t total_steps=0;

void Spi_init()
{
	 HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, 1);
	 __HAL_SPI_DISABLE(&hspi1);
	 __HAL_SPI_ENABLE(&hspi1);


}
void Spi_transmit(uint8_t byte)
{
//	uint8_t temp[2];
//	temp[0]=byte;temp[1]=0x00;
	HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, 0);
	HAL_SPI_Transmit(&hspi1,&byte, 1, 100);   /*sizeof byte =1 */
//	HAL_SPI_Transmit(&hspi1,temp, sizeof(temp), 100);
	HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, 1);

}


void Spi_recieve(uint8_t buff[],uint8_t size)
{

	HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, 0);
	HAL_SPI_Receive(&hspi1, buff, size, 100);
	HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, 1);

}


uint32_t Param(uint32_t value, register_length_t bit_len)
{
	uint32_t mask = 0xFFFFFFFF >> (32 - bit_len);
	if (value > mask) value = mask;
	return value & mask;
}



uint32_t Param_handler(uint32_t parameter , uint32_t value)
{

	switch (parameter) {

	case L6470_ABS_POS:    return Param(value, L6470_ABS_POS_lEN);

	case L6470_EL_POS:     return Param(value, L6470_EL_POS_LEN);

	case L6470_MARK:       return Param(value, L6470_MARK_LEN);

	case L6470_SPEED:      return Param(0, L6470_SPEED_LEN);

	case L6470_ACC:		   return Param(value, L6470_ACC_LEN);

	case L6470_DEC:        return Param(value, L6470_DEC_LEN);

	case L6470_MAX_SPEED:  return Param(value, L6470_MAX_SPEED_LEN);

	case L6470_MIN_SPEED:  return Param(value, L6470_MIN_SPEED_LEN);

	case L6470_FS_SPD:     return Param(value, L6470_FS_SPD_LEN);

	case L6470_KVAL_HOLD:  return Param(value,L6470_KVAL_HOLD_LEN);

	case L6470_KVAL_RUN:   return Param(value,L6470_KVAL_RUN_LEN);

	case L6470_KVAL_ACC:   return Param(value,L6470_KVAL_ACC_LEN);

	case L6470_KVAL_DEC:   return Param(value,L6470_KVAL_DEC_LEN);

	case L6470_INT_SPD:    return Param(value, L6470_INT_SPD_LEN);

	case L6470_ST_SLP:     return Param(value,L6470_ST_SLP_LEN);

	case L6470_FN_SLP_ACC: return Param(value,L6470_FN_SLP_ACC_LEN);

	case L6470_FN_SLP_DEC: return Param(value,L6470_FN_SLP_DEC_LEN);

	case L6470_K_THERM:    return Param(value,L6470_K_THERM_LEN);

	case L6470_ADC_OUT:    return Param(0,L6470_ADC_OUT_LEN);

	case L6470_OCD_TH:     return Param(value,L6470_OCD_TH_LEN);

	case L6470_STALL_TH:   return Param(value,L6470_STALL_TH_LEN);

	case L6470_STEP_MODE:  return Param(value,L6470_STEP_MODE_LEN);

	case L6470_ALARM_EN:   return Param(value,L6470_ALARM_EN_LEN);

	case L6470_CONFIG_A:   return Param(value, L6470_CONFIG_A_LEN);

	case L6470_STATUS_A:   return Param(value, L6470_STATUS_A_LEN);

	default:   				return Param(value,L6470_MAX_LEN);
	}
	return 0;

}




void Set_param(uint32_t parameter, uint32_t value , register_length_t length_in_bits)
{

	uint32_t return_data =0;
	uint8_t byte_len=0;
	uint8_t temperory_var=0;
	uint8_t temperory_var_2 =0;

	byte_len= (length_in_bits + 7) / 8;


	temperory_var_2= (uint8_t)((parameter)&0x000000ff);
	Spi_transmit(SetParam_CMD|temperory_var_2);

	return_data= Param_handler(parameter , value );
	switch(byte_len)
	{
	case 1:
		temperory_var = (uint8_t)((return_data)&0x000000ff);
		Spi_transmit(temperory_var);
//		Spi_transmit(0x00);
//		Spi_transmit(0x00);
		break;

	case 2:
		temperory_var = (uint8_t)((return_data>>8)&0x0000ff);
		Spi_transmit(temperory_var);
		temperory_var = (uint8_t)((return_data)&0x000000ff);
		Spi_transmit(temperory_var);
//		Spi_transmit(0x00);
		break;


	case 3:
		temperory_var = (uint8_t)((return_data>>16)&0x00ff);
		Spi_transmit(temperory_var);
		temperory_var = (uint8_t)((return_data>>8)&0x0000ff);
		Spi_transmit(temperory_var);
		temperory_var = (uint8_t)((return_data)&0x000000ff);
		Spi_transmit(temperory_var);
		break;

	case 4:
		temperory_var = (uint8_t)((return_data>>24)&0xff);
		Spi_transmit(temperory_var);
		temperory_var = (uint8_t)((return_data>>16)&0x00ff);
		Spi_transmit(temperory_var);
		temperory_var = (uint8_t)((return_data>>8)&0x0000ff);
		Spi_transmit(temperory_var);
		temperory_var = (uint8_t)((return_data)&0x000000ff);
		Spi_transmit(temperory_var);
		break;

	}


}





uint8_t get_param_handler(uint8_t parameter)
{
	switch (parameter)
	{

	case L6470_ABS_POS:    return L6470_ABS_POS_lEN;

	case L6470_EL_POS:     return L6470_EL_POS_LEN;

	case L6470_MARK:       return  L6470_MARK_LEN;

	case L6470_SPEED:      return  L6470_SPEED_LEN;

	case L6470_ACC:		   return  L6470_ACC_LEN;

	case L6470_DEC:        return  L6470_DEC_LEN;

	case L6470_MAX_SPEED:  return  L6470_MAX_SPEED_LEN;

	case L6470_MIN_SPEED:  return  L6470_MIN_SPEED_LEN;

	case L6470_FS_SPD:     return  L6470_FS_SPD_LEN;

	case L6470_KVAL_HOLD:  return  L6470_KVAL_HOLD_LEN;

	case L6470_KVAL_RUN:   return  L6470_KVAL_RUN_LEN;

	case L6470_KVAL_ACC:   return  L6470_KVAL_ACC_LEN;

	case L6470_KVAL_DEC:   return  L6470_KVAL_DEC_LEN;

	case L6470_INT_SPD:    return  L6470_INT_SPD_LEN;

	case L6470_ST_SLP:     return  L6470_ST_SLP_LEN;

	case L6470_FN_SLP_ACC: return  L6470_FN_SLP_ACC_LEN;

	case L6470_FN_SLP_DEC: return  L6470_FN_SLP_DEC_LEN;

	case L6470_K_THERM:    return  L6470_K_THERM_LEN;

	case L6470_ADC_OUT:    return  L6470_ADC_OUT_LEN;

	case L6470_OCD_TH:     return  L6470_OCD_TH_LEN;

	case L6470_STALL_TH:   return  L6470_STALL_TH_LEN;

	case L6470_STEP_MODE:  return  L6470_STEP_MODE_LEN;

	case L6470_ALARM_EN:   return  L6470_ALARM_EN_LEN;

	case L6470_CONFIG_A:   return  L6470_CONFIG_A_LEN;

	case L6470_STATUS_A:   return  L6470_STATUS_A_LEN;

	default:   			   return  L6470_MAX_LEN;

	}
}


uint32_t Get_param(uint8_t parameter)
{

	uint8_t byte_len=0;
	uint8_t length_in_bits=0;

	Spi_transmit(GetParam_CMD|parameter);

	uint8_t rc_data[1]={0};
	uint32_t get_data=0;
	Spi_recieve(rc_data,1);get_data=rc_data[0];rc_data[0]=0;	//memset(rc_data, '\0', sizeof(rc_data));
	Spi_recieve(rc_data,1);get_data =(get_data<<8)|rc_data[0];rc_data[0]=0; //memset(rc_data, '\0', sizeof(rc_data));
	Spi_recieve(rc_data,1);get_data =(get_data<<8)|rc_data[0];rc_data[0]=0; // memset(rc_data, '\0', sizeof(rc_data));



	length_in_bits = get_param_handler(parameter);
	byte_len= (length_in_bits + 7) / 8;
	switch(byte_len)
	{
		case 1:
				return get_data>>16;
		case 2:
				return get_data>>8;
		case 3:
				return get_data;
	}
	return get_data;

}

uint32_t step_s_2_Speed(uint32_t step_s)
{

	if(step_s<=STEP_S_MAX)
	{
		step_s = (uint32_t)(((float)step_s)/0.015);
	}
	else {step_s=0;}
	return step_s;

}

void Run(motor_direction_t dir, uint32_t speed)
{
	if(speed>=SPEED_MAX)
		{
		speed = (uint32_t)(SPEED_MAX);
		}


		Spi_transmit(RUN_CMD|dir);

		Spi_transmit((uint8_t)((speed>>16)&0x0f));
		Spi_transmit((uint8_t)((speed>>8)&0xff));
		Spi_transmit((uint8_t)(speed&0xff));

	}

void Move(motor_direction_t dir,uint32_t N_step)
{

	Spi_transmit(MOVE_CMD|dir);

	Spi_transmit((uint8_t)((N_step>>16)&0x3f));
	Spi_transmit((uint8_t)((N_step>>8)&0xff));
	Spi_transmit((uint8_t)(N_step&0xff));

	}


void GO_TO(uint32_t Abs_Pos)
{
	Spi_transmit(GO_TO_CMD);

	Spi_transmit((uint8_t)((Abs_Pos>>16)&0x3f));
	Spi_transmit((uint8_t)((Abs_Pos>>8)&0xff));
	Spi_transmit((uint8_t)(Abs_Pos&0xff));

}

void GO_TO_DIR(motor_direction_t dir,uint32_t Abs_Pos)
{
	Spi_transmit(GO_TO_CMD|dir);

	Spi_transmit((uint8_t)((Abs_Pos>>16)&0x3f));
	Spi_transmit((uint8_t)((Abs_Pos>>8)&0xff));
	Spi_transmit((uint8_t)(Abs_Pos&0xff));


}

void GO_UNTIL(act_state_t act ,motor_direction_t dir,uint32_t speed)
{

	if(speed>=SPEED_MAX)
		{
		speed = (uint32_t)(SPEED_MAX);
		}


	Spi_transmit(GO_UNTIL_CMD|act|dir);

	Spi_transmit((uint8_t)((speed>>16)&0x0f));
	Spi_transmit((uint8_t)((speed>>8)&0xff));
	Spi_transmit((uint8_t)(speed&0xff));


}




void GO_HOME(void)
{
	Spi_transmit(GO_HOME_CMD);
}

void GO_MARK(void)
{
	Spi_transmit(GO_MARK_CMD);
}

void RESET_POS(void)
{
	Spi_transmit(RESET_POS_CMD);
}


void RESET_DEVICE(void)
{
	Spi_transmit(RESET_DEVICE_CMD);
}
void SOFT_STOP(void)
{
	Spi_transmit(SOFT_STOP_CMD);
}
void HARD_STOP(void)
{
	Spi_transmit(HARD_STOP_CMD);
}
void SOFT_HIZ(void)
{
	Spi_transmit(SOFT_HIZ_CMD);
}
void HARD_HIZ(void)
{
	Spi_transmit(HARD_HIZ_CMD);
}

uint32_t GET_STATUS(void)
{
	Spi_transmit(Get_STATUS_CMD);

	uint8_t rc_data[1]={0};
	uint32_t get_data=0;
	Spi_recieve(rc_data,1);get_data=rc_data[0];rc_data[0]=0;//	memset(rc_data, '\0', sizeof(rc_data));
	Spi_recieve(rc_data,1);get_data =(get_data<<8)|rc_data[0];rc_data[0]=0;// memset(rc_data, '\0', sizeof(rc_data));

	return get_data;

}




unsigned long Read_total_steps()
{
	total_steps = Get_param(L6470_ABS_POS);

   // total_steps++;
    if (total_steps >= MAX_STEPS) {
    	uint32_t x=Get_param(L6470_ABS_POS);
    	RESET_POS();
    	total_steps=x;
        total_steps -= x;
        total_revolutions++;
    }
   return (total_revolutions* MAX_STEPS + total_steps);
}

