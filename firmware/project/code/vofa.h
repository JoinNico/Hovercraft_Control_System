/*
 * vofa.h
 *
 *  Created on: 2023Äê5ÔÂ23ÈÕ
 *      Author: sundm
 */

#ifndef _VOFA_H_
#define _VOFA_H_

#include "zf_common_headfile.h"

typedef union
{
    float fdata;
    unsigned long ldata;
}FloatLongType;

void JustFloat_Send(float * fdata,uint16_t fdata_num);

#endif /* CODE_VOFA_H_ */
