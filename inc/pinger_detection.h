/*
 * pinger_detection.h
 *
 *  Created on: Aug 3, 2018
 *      Author: Poornachander
 */

#ifndef PINGER_DETECTION_H_
#define PINGER_DETECTION_H_

extern uint16_t energy_threshold;
extern uint16_t detected_data[(4096 * 4) + 1];
extern uint8_t data_ready;

extern void init_pinger_detection();


#endif /* PINGER_DETECTION_H_ */
