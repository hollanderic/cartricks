#include <stdio.h>
#include <stdint.h>

#include "lpms.h"
#include "gps.h"

gps_t m_gps;
lpms_t m_lpms;

int main(void)
{
    uint32_t err;
    err=gps_init("/dev/ttyACM0",&m_gps);
    if (err != GPS_SUCCESS) {
        printf("crappy\n");
    }
    m_lpms.config_flags = 0;
   /* err=lpms_init("/dev/ttyUSB0",&m_lpms);
    if (err!=LPMS_SUCCESS) {
        printf("oh crap, something bombed\n");
    } else {
        printf("Successfully opened\n");
    }


    lpms_stop(&m_lpms);
   */ gps_stop(&m_gps);
    return 0;
}
