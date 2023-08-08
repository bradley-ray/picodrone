#ifndef __CONTROL_H__
#define __CONTROL_H__

#define MOTOR_1_GPIO 1
#define MOTOR_2_GPIO 2
#define MOTOR_3_GPIO 3
#define MOTOR_4_GPIO 4

/*
 * throttle 	-- t<8 bit>
 * roll 		-- r<8 bit>
 * pitch 		-- p<8 bit>
 * yaw 			-- y<8 bit>
 * P-gain		-- P<8 bit>
 * I-gain		-- I<8 bit>
 * D-gain		-- D<8 bit>
 */
void parse_cmd(char* cmd);

void motors_init(void);

#endif
