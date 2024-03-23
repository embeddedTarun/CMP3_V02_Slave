
#ifndef MY_MACROS_H
#define MY_MACROS_H

#define SQUARE(x) ((x) * (x))


#define MOTOR_DIR 		((GET_STATUS()&(1<<4))>>4)
#define MOTOR_STATUS 	((GET_STATUS()&(3<<5))>>5)
#define MOTOR_BUSY_FLAG 	((GET_STATUS()&(1<<1))>>1)
#define TOTAL_MICROSTEP		Read_total_steps(MOTOR_DIR,MOTOR_STATUS)
#endif
