#include <stdio.h>
#include <stdint.h>

#include "lpms.h"

lpms_t m_lpms;

int main(void)
{
    uint32_t err;
    printf("going in\n");
    err=lpms_init("/dev/cu.usbserial-A4004fsl",&m_lpms);
    if (err!=LPMS_SUCCESS) {
        printf("oh crap, something bombed\n");
    } else {
        printf("Hello World!\n");
    }

    lpms_stop(&m_lpms);
    return 0;
}
