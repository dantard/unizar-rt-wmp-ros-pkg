/*
 * timespec_utils.h
 *
 *  Created on: Mar 23, 2015
 *      Author: danilo
 */

#ifndef TIMESPEC_UTILS_H_
#define TIMESPEC_UTILS_H_
#include <time.h>
#include <time.h>

void timespec_addms(struct timespec *ts, long ms);
void timespec_subtract(struct timespec *a, struct timespec *b);
int timespec_milliseconds(struct timespec *a);
void timespec_now(struct timespec *ts);
int timespec_subtract_to_ms(struct timespec *a, struct timespec *b);
int timespec_elapsed_ms(struct timespec *a);

#endif /* TIMESPEC_UTILS_H_ */
