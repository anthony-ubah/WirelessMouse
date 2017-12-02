#ifndef _ACC
#define _ACC
#include <cmsis_os.h>
#define ACC_PIN GPIO_PIN_0
void acc_init(void);
void start_acc_thread(void *args);
void acc_thread(void const *args);
float get_pitch(void);
float get_roll(void);
extern osThreadId acc_thread_ID;
extern volatile int acc_interrupt;
#endif
