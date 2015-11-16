#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>
#include "gps.h"

#define BUFFSIZE 1024
#define BUFF_CHUNK 512



uint32_t gps_serial_open(const char * portname, uint32_t baudrate, int * p_fd)
{
    struct termios options;
    memset(&options, 0 ,sizeof options);


    *p_fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (*p_fd == -1) {
        printf("error opening %s\n",portname);
        return GPS_ERR_PORT;
    }
    else
    {
        printf("file open at %x\n",*p_fd);
        fcntl(*p_fd, F_SETFL, 0);
    }




    if (tcgetattr(*p_fd,&options) != 0) {
        printf("error from tcgetattr\n");
    }

    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~(PARENB | PARODD);
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

    return GPS_SUCCESS;
}

void gpsdump(uint8_t * buff, uint16_t size) {
 uint16_t i;


    for (i=0 ; i<size ; i++) {
        if (isprint(buff[i])) {
            printf("%c",buff[i]);
        } else {
            printf("#%02X",(buff[i]));
        }

    }

        printf("\n");
}
void gps_process(uint8_t * inbuff, uint16_t length) {
    uint8_t outstr[256];
    uint8_t timestamp[12],lat[20],lon[20];
    uint8_t ns[2];
    uint8_t ew[2];
    uint8_t messtype[10];
    uint8_t remainder[256];



    if (strncmp(inbuff,"$GPGGA",6)==0) {
        strncpy(outstr,inbuff,length);
        outstr[length]='\0';
        sscanf(outstr,"%[^','],%[^',']f,%[^','],%[^','],%s",messtype,timestamp,lat,ns,lon);
        printf("%s--%s--%s--%s\n",messtype,timestamp,lat,lon);
    }




}


void *  gps_idler(void * p)
{
    uint16_t head,tail,searchptr;
    gps_t * p_gps;

    uint8_t     buff[BUFFSIZE];
    uint8_t     current_line[180];

    uint32_t seq=0;
    uint8_t done=0;
    uint16_t count;
    struct timeval tv;
    p_gps = (gps_t *)p;

    tail=0;

    while (1) {
        seq++;
        head=0;
        count = read(p_gps->fd,&buff[tail],BUFFSIZE-tail);

        done=0;
        if (count<0) {
            pthread_exit(NULL);
        } else {
            tail = tail + count;
            while (done==0) {

                switch (p_gps->state) {

                    case GPS_STATE_IDLE:
                        while ((p_gps->state==GPS_STATE_IDLE) && (head<tail)) {
                            if ( buff[head] == '$' ) {
                                p_gps->state = GPS_STATE_HEADER;
                                searchptr=head;
                                break;
                            }
                            head = head + 1;
                        }
                        if (head == tail) {
                            done = 1;
                        }
                        break;

                    case GPS_STATE_HEADER:
                        while  (searchptr<tail) {
                            if ((buff[searchptr]=='\n') || (buff[searchptr] == '\r')) {
                                p_gps->state = GPS_STATE_FRAME_COMPLETE;
                                break;
                            }
                            searchptr = searchptr + 1;
                        }
                        if (searchptr == tail) {
                            done = 1;
                        }
                        break;

                    case GPS_STATE_FRAME_COMPLETE:
                        gps_process(&buff[head],searchptr - head);  //process the line of text
                        p_gps->state = GPS_STATE_IDLE;
                        head = searchptr;
                        break;
                }
            } // while (done)

            tail = tail - head;
            memcpy(buff,&buff[head], tail);
            head=0;
            searchptr = 0 ;

        } //if count>0
    } //while(1)
}

uint32_t gps_init(const char * portname, gps_t * p_gps)
{
    uint32_t err;
    int i;

    err=gps_serial_open(portname,B115200,&p_gps->fd);
    printf("Serial Open returned %d\n",err);


    pthread_create(&p_gps->pt_handle,NULL, &gps_idler,(void *) p_gps);
    return GPS_SUCCESS;
}




uint32_t gps_stop(gps_t * p_gps) {

    pthread_join(p_gps->pt_handle, NULL);
    printf("Thread done\n");
    return GPS_SUCCESS;
}


