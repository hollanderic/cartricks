#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include "lpms.h"


void dump(uint8_t * buff, uint32_t size)
{
    int i;
    for ( i=0 ; i<size ; i++) {
        printf("%02x",(uint8_t)buff[i]);
    }
    printf("\n");
}


uint32_t serial_open(uint8_t * portname, uint32_t baudrate, int * p_fd)
{
    struct termios options;

    *p_fd = open(portname, O_RDWR | O_NONBLOCK | O_NDELAY);
    if (*p_fd == -1) {
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
    uint8_t buff[100];
    int i;
    lpms_t * p_lpms;
    p_lpms = (lpms_t *)p;

    int count;
    (void)(i);
    while (1) {
        count=read(p_lpms->fd,buff,50);
        dump(buff,count);
    }



}

uint32_t lpms_init(uint8_t * p_portname, lpms_t * p_lpms)
{
    uint32_t err;
    err=serial_open(p_portname,B921600,&p_lpms->fd);

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







