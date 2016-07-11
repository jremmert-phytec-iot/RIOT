#include "msg.h"
#include <stdio.h>
#include <openthread.h>
#include "ot.h"
#include "thread.h"
#include "xtimer.h"
#include "msg.h"
#include "random.h"
#include "string.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include "periph/cpuid.h"


static char buf[100];
static char ot_thread_stack[3*THREAD_STACKSIZE_MAIN];


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

    kernel_pid_t pid = openthread_netdev2_init(ot_thread_stack, sizeof(ot_thread_stack), THREAD_PRIORITY_MAIN - 1,
            "ot_thread");
    DEBUG("Starting main's loop\n");

	(void) p;
	(void) msg;
#if 0
	memcpy(p, "start\n",sizeof("start"));
	sm.len = sizeof("start\n");
	msg_send(&msg, pid);
    printf("Go!\n");
#else
	while(1)
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
	}
#endif
	return 0;
}
