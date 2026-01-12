/*
 * encoder.h
 *
 *  Created on: 2024\1\23
 *      Author: Misaka
 */

#ifndef EC11_H_
#define EC11_H_

#include "zf_common_headfile.h"

#define FORDWARD_ROTATION 1 //EC11正转
#define REVERSE_ROTATION -1 //EC11反转
void EC11_Init(void);
char EC11_Scan(void);

#endif /* EC11_H_ */
