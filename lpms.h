#include <pthread.h>

#ifndef _LPMS_H
#define _LPMS_H 1

#define LPMS_SUCCESS 0

#define LPMS_ERR_PORT 1


typedef struct lpmst
{
    int32_t     fd;
    pthread_t   pt_handle;
} lpms_t;


typedef enum {
    LPMS_STATE_IDLE,
    LPMS_STATE_HEADER,
    LPMS_STATE_PACKET
} lpms_states_t;




uint32_t lpms_stop(lpms_t * p_lpms);

uint32_t lpms_init(const char * portname, lpms_t * p_lpms);


#endif /* lpms.h */
