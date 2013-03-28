/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/plugins/long_messages/long_messages.c
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/
#include <stdarg.h>
#include "config/compiler.h"
#include "core/interface/wmp_interface.h"
#include "core/include/global.h"
#include "core/include/frames.h"
#include "include/queue_core.h"
#include "core/include/queues.h"
#include "core/include/wmp_misc.h"
#include "core/include/queue_core.h"

typedef struct {
	long long last_push_ts;
	int period;
	int active;
	int initied;
	int fixed;
	int fixed_period;
	int mm[10];
	int mm_idx;
	int mm_initied;
} task_t;

static task_t tasks[128];

int task_get_next_predicted_push_delay_for_priority(int priority) {
	long long delay, elapsed, period_us;

	if (!tasks[priority].active) {
		return INT_MAX;
	}

	period_us = tasks[priority].period * 1000;
	elapsed = (getRawActualTimeus() - tasks[priority].last_push_ts);

	delay = elapsed > period_us ? 0 : period_us - elapsed;
	delay = delay > (long long) INT_MAX ? INT_MAX : delay;

	return (((int) delay) / 1000);

}

static int task_get_mm(int priority, int last_period){
	int i;
	if (!tasks[priority].mm_initied){
		for (i=0; i<10; i++){
			tasks[priority].mm[i]=last_period;
		}
		tasks[priority].mm_initied = 1;
		return last_period;
	}else{
		int sum = 0;
		tasks[priority].mm_idx++;
		if (tasks[priority].mm_idx >= 10){
			tasks[priority].mm_idx = 0;
		}
		tasks[priority].mm[tasks[priority].mm_idx] = last_period;
		for (i = 0; i < 10; i++) {
			sum += tasks[priority].mm[i];
		}
		return (int)(sum/10);
	}
}

void task_init(void) {
	int i;
	memset(tasks, 0, sizeof(tasks));
	for (i = 0; i < 128; i++) {
		tasks[i].period = INT_MAX;
	}
}

void task_push(int priority) {
	/* Calculate period */
	if (priority >= 0 && priority < 128) {
		if (tasks[priority].initied) {
			int candidate_period = (((int) (getRawActualTimeus() - tasks[priority].last_push_ts)) / 1000);
			if (tasks[priority].fixed && candidate_period < tasks[priority].fixed_period){
				candidate_period = tasks[priority].fixed_period;
			}
			tasks[priority].period = task_get_mm(priority,candidate_period);
			tasks[priority].active = 1;
		} else {
			tasks[priority].initied = 1;
		}
		tasks[priority].last_push_ts = getRawActualTimeus();
	}
}

static void flush_unactive(void) {
	int i;
	for (i = 0; i < 128; i++) {
		if (tasks[i].active) {
			long long elapsed, period_us;

			period_us = tasks[i].period * 1000;
			elapsed = (getRawActualTimeus() - tasks[i].last_push_ts);

			if (elapsed > (long long) 2 * period_us) {
				tasks[i].active = 0;
				tasks[i].initied = 0;
				tasks[i].mm_initied = 0;
			}
		}
	}
}

int task_get_next_predicted_push_delay(void) {
	int i, min = INT_MAX;
	flush_unactive();
	for (i = 0; i < 128; i++) {
		if (tasks[i].active){
			int delay = task_get_next_predicted_push_delay_for_priority(i);
			if (delay < min) {
				min = delay;
			}
		}
	}
	return min;
}

int task_get_next_predicted_push_delay_most_priority_than(int priority) {
	int i, min = INT_MAX;
	flush_unactive();
	for (i = priority; i < 128; i++) {
		if (tasks[i].active){
			int delay = task_get_next_predicted_push_delay_for_priority(i);
			if (delay < min) {
				min = delay;
			}
		}
	}
	return min;
}

int task_get_priority_period(int priority){
	if (priority >=0 && priority < 128){
		return tasks[priority].period;
	}else{
		return -1;
	}
}

int task_set_fixed_period(int priority, int period){
	if (priority >=0 && priority < 128){
		tasks[priority].fixed_period = period;
		tasks[priority].fixed = 1;
		tasks[priority].active = 0;
		return 1;
	}else{
		return 0;
	}
}

int wmpSetTaskMinimumSeparation(int priority, int period){
	return task_set_fixed_period(priority,period);
}
