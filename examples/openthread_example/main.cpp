#include "msg.h"
#include <stdio.h>
#include <openthread.h>
#include "ot.h"
#include "thread.h"
#include "xtimer.h"
#include "msg.h"
#include "random.h"
#include "cpuid.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#include "periph/cpuid.h"


static char buf[100];
char ot_thread_stack[THREAD_STACKSIZE_MAIN];


int main(void)
{
	msg_t msg;
	serial_msg_t sm;
	sm.len = sizeof(buf)+1;
	msg.type = OPENTHREAD_SERIAL_MSG_TYPE_EVENT;
	msg.content.ptr = &sm;

    char *p=buf;
	sm.buf = (uint8_t*) buf;
    sm.len = 0;

#ifdef CPUID_LEN
    char cpu_id[CPUID_LEN];
    cpuid_get(cpu_id);
    int seed = 0;
    for(int i=0;i<(int) CPUID_LEN;i++)
    {
        seed += cpu_id[i];
    }
    random_init(seed);
#else
    random_init(123);
#endif

   kernel_pid_t pid = thread_create(ot_thread_stack, sizeof(ot_thread_stack),
                           THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
			                                ot_thread, NULL, "ot_thread");
    DEBUG("Starting main's loop\n");
#ifdef CPUID_LEN
    char cpu_id[CPUID_LEN];
    cpuid_get(cpu_id);
    printf("The cpu id is: %.*s\n",CPUID_LEN,  cpuid);
#endif
    (void) msg;
    (void) p;
    (void) pid;

/*	while(1)
	{
        *p = getchar();
        if(*p == '\r' || *p == '\n')
        {
            sm.len = p-buf+1; 
            msg_send(&msg, pid);
            p=buf;
        }
        else
        {
            p++;
        }
	}*/
	return 0;
}
