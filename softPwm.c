/*
 * softPwm.c:
 *	Provide many channels of software driven PWM.
 *	Copyright (c) 2012-2017 Gordon Henderson
 *	modified 29 March 20019 by Anıl Aras for FreeBSD
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>

#include <libgpio.h>
#include "softPwm.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <sched.h>


// MAX_PINS:
//	This is more than the number of Pi pins because we can actually softPwm.
//	Once upon a time I let pins on gpio expanders be softPwm'd, but it's really
//	really not a good thing.

#define	MAX_PINS	255

// The PWM Frequency is derived from the "pulse time" below. Essentially,
//	the frequency is a function of the range and this pulse time.
//	The total period will be range * pulse time in µS, so a pulse time
//	of 100 and a range of 100 gives a period of 100 * 100 = 10,000 µS
//	which is a frequency of 100Hz.
//
//	It's possible to get a higher frequency by lowering the pulse time,
//	however CPU uage will skyrocket as wiringPi uses a hard-loop to time
//	periods under 100µS - this is because the Linux timer calls are just
//	not accurate at all, and have an overhead.
//
//	Another way to increase the frequency is to reduce the range - however
//	that reduces the overall output accuracy...

#define	PULSE_TIME	100

#define	LOW     0
#define	HIGH    1

#define	INPUT			 0
#define	OUTPUT			 1

static volatile int marks         [MAX_PINS] ;
static volatile int range         [MAX_PINS] ;
static volatile pthread_t threads [MAX_PINS] ;
static volatile int newPin = -1 ;

static uint64_t epochMilli, epochMicro ;

/*
 * softPwmThread:
 *	Thread to do the actual PWM output
 *********************************************************************************
 */


int piHiPri (const int pri)
{
    struct sched_param sched ;

    memset (&sched, 0, sizeof(sched)) ;

    if (pri > sched_get_priority_max (SCHED_RR))
        sched.sched_priority = sched_get_priority_max (SCHED_RR) ;
    else
        sched.sched_priority = pri ;

    return sched_setscheduler (0, SCHED_RR, &sched) ;
}


void delay (unsigned int howLong)
{
    struct timespec sleeper, dummy ;

    sleeper.tv_sec  = (time_t)(howLong / 1000) ;
    sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

    nanosleep (&sleeper, &dummy) ;
}


/*
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicrosecondsHard (unsigned int howLong)
{
    struct timeval tNow, tLong, tEnd ;

    gettimeofday (&tNow, NULL) ;
    tLong.tv_sec  = howLong / 1000000 ;
    tLong.tv_usec = howLong % 1000000 ;
    timeradd (&tNow, &tLong, &tEnd) ;

    while (timercmp (&tNow, &tEnd, <))
    gettimeofday (&tNow, NULL) ;
}

void delayMicroseconds (unsigned int howLong)
{
    struct timespec sleeper ;
    unsigned int uSecs = howLong % 1000000 ;
    unsigned int wSecs = howLong / 1000000 ;

    /**/ if (howLong ==   0)
        return ;
    else if (howLong  < 100)
        delayMicrosecondsHard (howLong) ;
    else
    {
        sleeper.tv_sec  = wSecs ;
        sleeper.tv_nsec = (long)(uSecs * 1000L) ;
        nanosleep (&sleeper, NULL) ;
    }
}


/*
 * millis:
 *	Return a number of milliseconds as an unsigned int.
 *	Wraps at 49 days.
 *********************************************************************************
 */

unsigned int millis (void)
{
    uint64_t now ;

#ifdef	OLD_WAY
    struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

#else
    struct  timespec ts ;

    clock_gettime (CLOCK_MONOTONIC, &ts) ;
    now  = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000L) ;
#endif

    return (uint32_t)(now - epochMilli) ;
}


/*
 * micros:
 *	Return a number of microseconds as an unsigned int.
 *	Wraps after 71 minutes.
 *********************************************************************************
 */

unsigned int micros (void)
{
    uint64_t now ;
#ifdef	OLD_WAY
    struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ;
#else
    struct  timespec ts ;

    clock_gettime (CLOCK_MONOTONIC, &ts) ;
    now  = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ;
#endif


    return (uint32_t)(now - epochMicro) ;
}

void digitalWrite (int pin, int value){
    int pinVal = 0;
    gpio_handle_t handle;
    handle = gpio_open(0);
    if(value == HIGH){
        pinVal = 1;
    } else{
        pinVal = 0;
    }
    gpio_pin_set(handle,pin,pinVal);
    gpio_close(handle);
}

void pinMode (int pin, int mode) {
    gpio_handle_t handle;
    handle = gpio_open(0);

    if(mode == INPUT){
        gpio_pin_input(handle, pin); //(handle, 127);
    }
    else if (mode == OUTPUT){
        gpio_pin_output(handle, pin); //(handle, 127);
    } else{
    }
    gpio_close(handle);
}
static void *softPwmThread (void *arg)
{
    int pin, mark, space ;
    struct sched_param param ;

    param.sched_priority = sched_get_priority_max (SCHED_RR) ;
    pthread_setschedparam (pthread_self (), SCHED_RR, &param) ;

    pin = *((int *)arg) ;
    free (arg) ;

    pin    = newPin ;
    newPin = -1 ;

    piHiPri (90) ;

    for (;;)
    {
        mark  = marks [pin] ;
        space = range [pin] - mark ;

        if (mark != 0)
            digitalWrite (pin, HIGH) ;
        delayMicroseconds (mark * 100) ;

        if (space != 0)
            digitalWrite (pin, LOW) ;
        delayMicroseconds (space * 100) ;
    }

    return NULL ;
}


/*
 * softPwmWrite:
 *	Write a PWM value to the given pin
 *********************************************************************************
 */

void softPwmWrite (int pin, int value)
{
    if (pin < MAX_PINS)
    {
        /**/ if (value < 0)
            value = 0 ;
        else if (value > range [pin])
            value = range [pin] ;

        marks [pin] = value ;
    }
}


/*
 * softPwmCreate:
 *	Create a new softPWM thread.
 *********************************************************************************
 */

int softPwmCreate (int pin, int initialValue, int pwmRange)
{
    int res ;
    pthread_t myThread ;
    int *passPin ;

    if (pin >= MAX_PINS)
        return -1 ;

    if (range [pin] != 0)	// Already running on this pin
        return -1 ;

    if (pwmRange <= 0)
        return -1 ;

    passPin = malloc (sizeof (*passPin)) ;
    if (passPin == NULL)
        return -1 ;

    digitalWrite (pin, LOW) ;
    pinMode      (pin, OUTPUT) ;

    marks [pin] = initialValue ;
    range [pin] = pwmRange ;

    *passPin = pin ;
    newPin   = pin ;
    res      = pthread_create (&myThread, NULL, softPwmThread, (void *)passPin) ;

    while (newPin != -1)
        delay (1) ;

    threads [pin] = myThread ;

    return res ;
}


/*
 * softPwmStop:
 *	Stop an existing softPWM thread
 *********************************************************************************
 */

void softPwmStop (int pin)
{
    if (pin < MAX_PINS)
    {
        if (range [pin] != 0)
        {
            pthread_cancel (threads [pin]) ;
            pthread_join   (threads [pin], NULL) ;
            range [pin] = 0 ;
            digitalWrite (pin, LOW) ;
        }
    }
}