#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "lpms.h"

#define BUFFSIZE 1024
#define BUFF_CHUNK 128

uint8_t     buff[BUFFSIZE];
uint32_t    head;
uint32_t    tail;
uint32_t    end;

lpms_frame_t   curr_frame;

lpms_states_t   state=LPMS_STATE_IDLE;

void dump(uint8_t * buff, uint32_t size)
{
    int i;
    printf("%d bytes:",size);
    for ( i=0 ; i<size ; i++) {
        printf("%02x",(uint8_t)buff[i]);
    }
    printf("\n");
}

void dumpheader() {
    printf("start:%x  id:%x    command:%x  length:%x\n",curr_frame.header.start, curr_frame.header.openmat_id,curr_frame.header.command, curr_frame.header.length);


}


uint32_t serial_open(const char * portname, uint32_t baudrate, int * p_fd)
{
    struct termios options;

    *p_fd = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
    if (*p_fd == -1) {
        printf("error opening %s\n",portname);
        return LPMS_ERR_PORT;
    }
    else
    {
        printf("file open at %x\n",*p_fd);
        fcntl(*p_fd, F_SETFL, 0);
    }
    tcgetattr(*p_fd,&options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag = 0;
    options.c_oflag = 0;

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 5;

    options.c_iflag &= ~(IXON | IXOFF | IXANY );
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG );
    options.c_oflag &= ~OPOST;

    tcflush(*p_fd, TCIFLUSH );

    tcsetattr(*p_fd, TCSANOW, &options);

    return LPMS_SUCCESS;
}

static inline uint16_t datacpy(uint8_t * p_buff, lpms_frame_t * p_frame, uint16_t ptr) {
    memcpy(&(p_frame->data),p_buff+ptr,p_frame->header.length);
    ptr = ptr + p_frame->header.length;
    p_frame->crc = buff[ptr] + (buff[ptr+1] << 8) ;
    p_frame->ender = buff[ptr+2] + (buff[ptr+3] << 8);
    return p_frame->header.length + 4 ;
}

static inline uint16_t readheader(uint8_t * p_buff, lpms_frame_t * p_frame, uint16_t ptr) {
    p_frame->header.start = p_buff[ptr];
    p_frame->header.openmat_id = p_buff[ptr+1] + (p_buff[ptr+2] << 8);
    p_frame->header.command = p_buff[ptr+3] + (p_buff[ptr+4] << 8);
    p_frame->header.length = p_buff[ptr+5] + (p_buff[ptr+6] << 8);
    return LPMS_HEADER_SIZE;
}


void * idler(void * p)
{
    int i,ptr;
    lpms_t * p_lpms;
    p_lpms = (lpms_t *)p;
    uint8_t tempbuff[256];

    int count;
    (void)(i);
    while (1) {
        count = read(p_lpms->fd,&buff[tail],BUFF_CHUNK);
        if (count<0) {
            pthread_exit(NULL);
        } else {
            printf("\n\nStarting: read %d bytes.  State:%d\n",count,(int)p_lpms->state);
            tail = tail + count;
            dump(buff,tail);
            switch (p_lpms->state) {

            case LPMS_STATE_IDLE:
                i=0;
                while ((p_lpms->state==LPMS_STATE_IDLE) && (i<tail)) {
                    if ( (buff[i]==0x3A) && (buff[i+1]==0x01)
                                         && (buff[i+2]==0x00)) {
                        printf("Got sync at %d\n",i);
                        if ((tail-i) >= LPMS_HEADER_SIZE) {
                            ptr=readheader( buff, &curr_frame, i);
                            printf("Got full header\n");
                            dump((uint8_t *)&(curr_frame.header),ptr);
                            dumpheader();
                            if ((tail-i) >= ( ptr + curr_frame.header.length+4)) {
                                    ptr=datacpy(buff,&curr_frame, i + LPMS_HEADER_SIZE);
                                    printf("Got Full Packet! %d bytes\n",ptr);

                                    memcpy(buff, &(buff[i+ptr]),tail - i - ptr);
                                    tail = tail - ptr - i;
                                    p_lpms->state = LPMS_STATE_FRAME_COMPLETE;
                            } else {
                                tail = tail - ( i + ptr );
                                memcpy(&buff, &(buff[i + ptr]),tail);
                                p_lpms->state = LPMS_STATE_DATA; //wait for rest of frame
                                printf("moving to LPMS_STATE_FAME\n");
                            }
                        } else {
                            printf("moving to LPMS_STATE_HEADER\n");
                            //wait for rest of header
                            memcpy(buff,&buff[i],tail-1);
                            tail = tail - i;
                            p_lpms->state=LPMS_STATE_HEADER;
                        }
                    } else {
                        i = i + 1;
                    }
                }
                if (p_lpms->state == LPMS_STATE_IDLE) {
                    printf("still looking for sync, trimming buffer\n");
                    if (tail>2) {
                        buff[0] = buff[tail-2];
                        buff[1] = buff[tail-1];
                        tail = 2;
                    }
                }
                break;
            case LPMS_STATE_HEADER:
                if (tail >= LPMS_HEADER_SIZE) {
                    readheader(buff, &curr_frame, 0);
                    dumpheader();
                    if (tail >= LPMS_HEADER_SIZE + curr_frame.header.length+4) {
                        ptr = datacpy( buff, &curr_frame, LPMS_HEADER_SIZE);
                        memcpy(buff, &(buff[ptr]),tail-ptr);
                        tail = tail - ptr;
                        p_lpms->state = LPMS_STATE_FRAME_COMPLETE;
                    } else {
                        p_lpms->state = LPMS_STATE_DATA;
                    }
                }
                break;
            case LPMS_STATE_DATA:
                if (tail >= LPMS_HEADER_SIZE + curr_frame.header.length + 4) {
                    ptr = datacpy(buff,&curr_frame, LPMS_HEADER_SIZE);
                    memcpy(buff, &(buff[ptr]),tail-ptr);
                    tail = tail - ptr;
                    p_lpms->state = LPMS_STATE_FRAME_COMPLETE;
                }
                break;
            default:
                break;




            }
            if (p_lpms->state == LPMS_STATE_FRAME_COMPLETE) {
                printf("Got a whole frame\n");
                p_lpms->state = LPMS_STATE_IDLE;
            }
            printf("Ending buff:");
            dump(buff,tail);
            //printf("Read %d bytes from 0x%x\n",count,fcntl(p_lpms->fd,F_GETFD));
        }
    }
}

uint32_t lpms_init(const char * portname, lpms_t * p_lpms)
{
    uint32_t err;

    head    =   0;
    tail    =   0;
    end     =   0;
    state   =   LPMS_STATE_IDLE;

    err=serial_open(portname,921600,&p_lpms->fd);

    if (err!=LPMS_SUCCESS) {
        return err;
    }

    pthread_create(&p_lpms->pt_handle,NULL, &idler,(void *) p_lpms);

    return LPMS_SUCCESS;
}



uint32_t lpms_stop(lpms_t * p_lpms) {

    pthread_join(p_lpms->pt_handle, NULL);
    printf("Thread done\n");
    return LPMS_SUCCESS;
}







