#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

void dump(uint8_t * buff, uint32_t size)
{
    int i;
    for ( i=0 ; i<size ; i++) {
        printf("%02x",(uint8_t)buff[i]);
    }
    printf("\n");
}


void serial_open(uint8_t * portname, uint32_t baudrate, int * p_fd)
{
    struct termios options;
    
    *p_fd = open(portname, O_RDWR | O_NONBLOCK | O_NDELAY);
    if (*p_fd == -1) {
        perror("serial open: unable to open \n");
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

}

typedef struct test
{
    uint32_t    temporary;
    uint32_t    temp2;
} test_t;


test_t temp;
int fd;

void main(void) {
    int n;
    uint8_t buff[10];

    serial_open("/dev/ttyUSB0",B921600,&fd);

    n =  read(fd, &buff, 10);
    printf("%d bytes read\n",n);
    dump(&buff[0],n);
   close(fd);

}
