/*
 * m4_rpmsg.h
 *
 *  Created on: Jun 30, 2020
 *      Author: eaxelsso
 */

#ifndef SRC_M4_RPMSG_H_
#define SRC_M4_RPMSG_H_

enum ovRpmsg_mode {
	ovRpmsg_mode_MIN = 0,
	ovRpmsg_mode_QQVGA_160_120 = 0,
	ovRpmsg_mode_QQVGA_128_96 = 1,
	ovRpmsg_mode_MAX = 1
};

#define IR_RESOLUTION_FULL_WIDTH  160
#define IR_RESOLUTION_FULL_HEIGHT 120
#define IR_RESOLUTION_REDUCED_WIDTH  128
#define IR_RESOLUTION_REDUCED_HEIGHT 96
#define IR_RESOLUTION_DEFAULT_WIDTH  IR_RESOLUTION_FULL_WIDTH
#define IR_RESOLUTION_DEFAULT_HEIGHT IR_RESOLUTION_FULL_HEIGHT


#endif /* SRC_M4_RPMSG_H_ */
