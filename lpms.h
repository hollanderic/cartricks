#include <pthread.h>

#ifndef _LPMS_H
#define _LPMS_H 1

#define LPMS_SUCCESS 0

#define LPMS_ERR_PORT 1



typedef enum {
    LPMS_STATE_IDLE,
    LPMS_STATE_HEADER,
    LPMS_STATE_PACKET
} lpms_states_t;

typedef struct lpmsheader
{
    uint8_t     start;
    uint16_t    openmat_id;
    uint16_t    command;
    uint16_t    length;
} lpms_header_t;

typedef struct lpmsframe
{
    lpms_header_t   header;
    uint8_t         data[1024];
    uint16_t        crc;
    uint16_t        ender;
} lpms_frame_t;

typedef struct lpmst
{
    int32_t         fd;
    pthread_t       pt_handle;
    lpms_states_t   state;
} lpms_t;

uint32_t lpms_stop(lpms_t * p_lpms);

uint32_t lpms_init(const char * portname, lpms_t * p_lpms);


#endif /* lpms.h */
