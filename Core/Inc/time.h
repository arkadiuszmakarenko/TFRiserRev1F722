/*
 * time.h
 * 
 * System-time abstraction over STM32 STK timer.
 * 
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 * 
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

//typedef uint32_t time_t;





void delay_from(time_t t, unsigned int ticks);
time_t time_now(void);


void time_init(void);

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
