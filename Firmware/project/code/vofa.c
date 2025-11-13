/*
 * vofa.c
 *
 *  Created on: 2023Äê5ÔÂ23ÈÕ
 *      Author: sundm
 */

#include "vofa.h"

const uint8_t tail [4] = {0x00, 0x00, 0x80, 0x7f};
uint8_t c_data[4];
/*JustFloat*/
void float_turn_u8(float f,uint8_t * c){
    uint8_t x;
    FloatLongType data;
    data.fdata=f;

    for(x=0;x<4;x++){
        c[x]=(uint8_t)(data.ldata>>(x*8));
    }
}

//fdata:
void JustFloat_Send(float * fdata,uint16_t fdata_num)
{
        uint16_t x;
//        uint8_t y;
        for(x=0;x<fdata_num;x++)
        {
            float_turn_u8(fdata[x],c_data);
            wireless_uart_send_buffer(c_data, 4);
        }
        wireless_uart_send_buffer(tail, 4);
}





