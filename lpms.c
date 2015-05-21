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
    for ( i=0 ; i<size ; i++) {
        printf("%02x",(uint8_t)buff[i]);
    }
    printf("\n");
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
            tail = tail + count;
            switch (p_lpms->state) {

            case LPMS_STATE_IDLE:
                i=0;
                while ((p_lpms->state==LPMS_STATE_IDLE) && (i<tail)) {
                    if ( (buff[i]==0x3A) && (buff[i+1]==0x01)
                                         && (buff[i+2]==0x00)) {
                        if ((tail-i) >= sizeof(lpms_header_t)) {
                            memcpy(&(curr_frame.header), &buff[i],
                                        sizeof(lpms_header_t));
                            if ((tail-i) >= (sizeof(lpms_header) +current_header.length+4)) {
                                    ptr = i + sizeof(lpms_header_t);
                                    memcpy(&(curr_frame.data),&buff[ptr],curr_frame.header.length);
                                    ptr = ptr + curr_frame.header.length;
                                    curr_frame.crc = buff[ptr] + buff[ptr+1] << 8;
                                    curr_frame.ender = buff[ptr +2] + buff[ptr + 3] << 8;
                                    printf("Got Full Packet!\n");
                                    memcpy(buff, &(buff[ptr+4]),tail-(ptr+4));
                                    tail = tail - (ptr+4);
                            } else {

                                p_lpms->state = LPMS_STATE_FRAME; //wait for rest of frame
                            }
                        } else {
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
                    if (tail>2) {
                        buff[0] = buff[tail-2];
                        buff[1] = buff[tail-1];
                        tail = 2;
                    }
                }
                break;
            case LPMS_STATE_HEADER:
                break;
            default:
                break;




            }




            printf("Read %d bytes from 0x%x\n",count,fcntl(p_lpms->fd,F_GETFD));
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







