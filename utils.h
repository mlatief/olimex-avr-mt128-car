/* utils.h - Some useful delay functions */

#ifndef MYUTILS_H
#define MYUTILS_H

/* Implement a delay for one sec since maximum delay of provided _delay_ms is 262.14 ms / F_CPU in MHz*/
void delay_msec(int);
int rrand(int);
#endif