#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include "lpms.h"

#define BUFFSIZE 1024
#define BUFF_CHUNK 128

uint8_t     buff[BUFFSIZE];
uint32_t    head;
uint32_t    tail;
uint32_t    end;

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

    *p_fd = open(portname, O_RDWR | O_NONBLOCK | O_NDELAY);
    if (*p_fd == -1) {
        printf("error opening %s\n",portname);
        return LPMS_ERR_PORT;
    }
    else
    {
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

    options.c_cc[VMIN]  = 1;
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
    int i;
    lpms_t * p_lpms;
    p_lpms = (lpms_t *)p;

    int count;
    (void)(i);
    while (1) {
        printf("\nGoing in-----head=%d   end=%d   tail=%d\n",head,end,tail);
        if ((tail+BUFF_CHUNK) >= BUFFSIZE) {     //Read could wrap buffer
            end = tail;
            count = read(p_lpms->fd,&buff[0],BUFF_CHUNK);   // start at beginning
            tail = count;
            if ( tail > head ) {
                head = tail;
            }
        } else {
            count = read(p_lpms->fd,&buff[tail],BUFF_CHUNK);
            if (tail < head) {
                if ( (tail + count) > head) {
                    head = tail + count;
                }
            }
            tail = tail + count;
            end = tail;
        }
        printf("Going out----head=%d   end=%d   tail=%d  count=%d\n",head,end,tail,count);
        
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







