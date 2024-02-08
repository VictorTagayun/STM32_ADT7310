/*
 * ADT7310.h
 *
 *  Created on: Feb 5, 2024
 *      Author: https://victortagayun.github.io
 */

#ifndef INC_VT_ADT7310_H_
#define INC_VT_ADT7310_H_

#define Status   	0x00 //  8-bit
#define Config 	 	0x01 //  8-bit
#define Temp_val 	0x02 // 16-bit
#define ID 		 	0x03 //  8-bit
#define TCRIT_setpt 0x04 // 16-bit
#define THYST_setpt	0x05 //  8-bit
#define THIGH_setpt	0x06 // 16-bit
#define TLOW_setpt 	0x07 // 16-bit
#define ADT7310_read  0x40
#define ADT7310_write 0x00
#define ADT7310_16bit 0x80

#endif /* INC_VT_ADT7310_H_ */
