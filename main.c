#include <stdio.h>
#include <stdint.h>

#include "lpms.h"

lpms_t m_lpms;

int main(void)
{
    uint32_t err;

    err=lpms_init("/dev/ttyUSB0",&m_lpms);
    if (err!=LPMS_SUCCESS) {
        printf("oh crap, something bombed\n");
    } else {
        printf("Hello World!\n");
    }

    lpms_stop(&m_lpms);
    return 0;
}
