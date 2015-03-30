#include "timespec_utils.h"
void timespec_addms(struct timespec *ts, long ms) {
	int sec = ms / 1000;
	ms = ms - sec * 1000;
	ts->tv_nsec += ms * 1000000;
	ts->tv_sec += ts->tv_nsec / 1000000000 + sec;
	ts->tv_nsec = ts->tv_nsec % 1000000000;
}

void timespec_subtract(struct timespec *a, struct timespec *b) {
	a->tv_nsec = a->tv_nsec - b->tv_nsec;
	if (a->tv_nsec < 0) {
		// borrow.
		a->tv_nsec += 1000000000;
		a->tv_sec--;
	}
	a->tv_sec = a->tv_sec - b->tv_sec;
}

int timespec_milliseconds(struct timespec *a) {
	return a->tv_sec * 1000 + a->tv_nsec / 1000000;
}

void timespec_now(struct timespec *ts) {
	clock_gettime(CLOCK_REALTIME, ts);
}

int timespec_subtract_to_ms(struct timespec *a, struct timespec *b) {
	timespec_subtract(a, b);
	return timespec_milliseconds(a);
}
int timespec_elapsed_ms(struct timespec *a) {
	struct timespec now;
	timespec_now(&now);
	timespec_subtract(&now, a);
	return timespec_milliseconds(&now);
}
