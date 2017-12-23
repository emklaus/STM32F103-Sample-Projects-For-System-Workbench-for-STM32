/*
 * noclick.h
 *  two 8 byte sounds to play before & after recorded sounds to prevent
 *  'clicks' from sound files that start with values around 128
 *
 *  Created on: Mar 1, 2016
 *      Author: Eric M. Klaus
 */

#ifndef NOCLICK_H_
#define NOCLICK_H_

const int pre_play_length = 8;
const int post_play_length = 8;

//Ramp sound up
const uint8_t pre_play_data[] = {16,  32, 48,  64,  80,  96, 112, 128};

//Ramp sound down
const uint8_t post_play_data[] = {128, 112, 96,  80,  64,  48,  32,  16};

#endif /* NOCLICK_H_ */
