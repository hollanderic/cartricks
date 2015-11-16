#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>
#include "lpms.h"

#define BUFFSIZE 1024
#define BUFF_CHUNK 512


void dump(uint8_t * buff, uint32_t size)
{
    int i;
    printf("%d bytes:",size);
    for ( i=0 ; i<size ; i++) {
        printf("%02x",(uint8_t)buff[i]);
    }
    printf("\n");
}

void dumpheader(lpms_frame_t * p_frame) {
    printf("start:0x%x  id:0x%x    command:0x%x  length:%d\n",p_frame->header.start,
        p_frame->header.openmat_id,p_frame->header.command, p_frame->header.length);
}

uint64_t gettime() {
    struct timeval t;
    uint64_t tt;

    gettimeofday(&t,NULL);
    tt = t.tv_sec*1000000 + t.tv_usec;
    return tt;
}

uint16_t    calc_crc(lpms_frame_t * p_frame) {
    uint16_t crc=1;     //Initialize with openmatid
    uint16_t i;

    crc = crc + p_frame->header.command;
    crc = crc + p_frame->header.length;
    for (i = 0 ; i < p_frame->header.length ; i++ ) {
        crc = crc + p_frame->data[i];
    }

    return crc;
}

uint16_t    frame_to_buff(uint8_t * buff, lpms_frame_t * p_frame) {
    uint16_t crc;

    buff[0] = (p_frame->header.start ) & 0xFF;
    buff[1] = (p_frame->header.openmat_id     ) & 0xFF;
    buff[2] = (p_frame->header.openmat_id >> 8) & 0xFF;
    buff[3] = (p_frame->header.command     ) & 0xFF;
    buff[4] = (p_frame->header.command >> 8) & 0xFF;
    buff[5] = (p_frame->header.length      ) & 0xFF;
    buff[6] = (p_frame->header.length  >> 8) & 0xFF;

    memcpy( &buff[7] , p_frame->data, p_frame->header.length);

    crc     = calc_crc(p_frame);

    buff[ 7 +  p_frame->header.length] = ( crc      ) & 0xFF;
    buff[ 8 +  p_frame->header.length] = ( crc >> 8 ) & 0xFF;
    buff[ 9 +  p_frame->header.length] = 0x0D;
    buff[ 10 + p_frame->header.length] = 0x0A;

    return 10 + p_frame->header.length + 1;
}


uint32_t serial_open(const char * portname, uint32_t baudrate, int * p_fd)
{
    struct termios options;
    memset(&options, 0 ,sizeof options);


    *p_fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (*p_fd == -1) {
        printf("error opening %s\n",portname);
        return LPMS_ERR_PORT;
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

    //tcflush(*p_fd, TCIFLUSH );

    tcsetattr(*p_fd, TCSANOW, &options);

    return LPMS_SUCCESS;
}

static inline uint16_t datacpy(uint8_t * p_buff, lpms_frame_t * p_frame) {

    uint16_t ptr;
    memcpy(&(p_frame->data),p_buff,p_frame->header.length);

    ptr =  p_frame->header.length;
    p_frame->crc = p_buff[ptr] + (p_buff[ptr+1] << 8) ;
    p_frame->ender = p_buff[ptr+2] + (p_buff[ptr+3] << 8);
    return p_frame->header.length + 4 ;
}

static inline uint16_t readheader(uint8_t * p_buff, lpms_frame_t * p_frame) {
    p_frame->header.start = p_buff[0];
    p_frame->header.openmat_id = p_buff[1] + (p_buff[2] << 8);
    p_frame->header.command = p_buff[3] + (p_buff[4] << 8);
    p_frame->header.length = p_buff[5] + (p_buff[6] << 8);
    return LPMS_HEADER_SIZE;
}






void procframe(lpms_t * p_lpms,lpms_frame_t * p_frame) {
    uint32_t data;
    switch (p_frame->header.command) {
        case LPMS_COMMAND_GET_CONFIG:
            memcpy( &data, p_frame->data, 4);
            p_lpms->config_flags = data;
            printf("0x%08X\n",data);
            if (LPMS_PRESSURE_ENABLED & data) { printf("Pressure enabled\n"); }
            if (LPMS_MAGNETOMETER_ENABLED & data) { printf("Magnetometer enabled\n"); }
            if (LPMS_ACCELEROMETER_ENABLED & data) { printf("Accelerometer enabled\n"); }
            if (LPMS_GYROSCOPE_ENABLED & data) { printf("Gyroscope enabled\n"); }
            if (LPMS_TEMPERATURE_ENABLED & data) { printf("Temperature enabled\n"); }
            if (LPMS_HEAVE_MOTION_ENABLED & data) { printf("Heave motion enabled\n"); }
            if (LPMS_ANGULAR_VELOCITY_ENABLED & data) { printf("Angular velocity enabled\n"); }
            if (LPMS_EULER_ANGLE_ENABLED & data) { printf("Euler angle enabled\n"); }
            if (LPMS_ORIENT_QUAT_ENABLED & data) { printf("Orientation quaternion enabled\n"); }
            if (LPMS_ALTITUDE_ENABLED & data) { printf("Altitude enabled\n"); }
            if (LPMS_LINEAR_ACC_ENABLED & data) { printf("Linear acceleration enabled\n"); }
            break;

        case LPMS_COMMAND_GET_SENSOR_DATA:
            if (p_lpms->config_flags) {
                uint16_t i=0;
                uint16_t idx;
                memcpy(&(p_lpms->data.timestamp), &p_frame->data[i], 4);
                i=i+4;
                printf("Timestamp:%f\n",p_lpms->data.timestamp);
                if (LPMS_GYROSCOPE_ENABLED & p_lpms->config_flags) { 
                    memcpy(p_lpms->data.cal_gyro , &p_frame->data[i] , 4*3);
                    i = i + 12;
                    printf("Gyro- %09f  %09f  %09f \n", p_lpms->data.cal_gyro[0],
                                                        p_lpms->data.cal_gyro[1],
                                                        p_lpms->data.cal_gyro[2]); 
                }
                if (LPMS_ACCELEROMETER_ENABLED & p_lpms->config_flags) { 
                    memcpy(p_lpms->data.cal_acc , &p_frame->data[i] , 4*3);
                    i = i + 12;
                    printf("Acc - %09f  %09f  %09f \n", p_lpms->data.cal_acc[0],
                                                        p_lpms->data.cal_acc[1],
                                                        p_lpms->data.cal_acc[2]); 
                }
                if (LPMS_MAGNETOMETER_ENABLED & p_lpms->config_flags) { 
                    memcpy(p_lpms->data.cal_mag , &p_frame->data[i] , 4*3);
                    i = i + 12;
                    printf("Mag - %09f  %09f  %09f \n", p_lpms->data.cal_mag[0],
                                                        p_lpms->data.cal_mag[1],
                                                        p_lpms->data.cal_mag[2]); 
                }
                if (LPMS_ANGULAR_VELOCITY_ENABLED & p_lpms->config_flags) { 
                    memcpy(p_lpms->data.ang_velocity , &p_frame->data[i] , 4*3);
                    i = i + 12;
                    printf("AngV- %09f  %09f  %09f \n", p_lpms->data.ang_velocity[0],
                                                        p_lpms->data.ang_velocity[1],
                                                        p_lpms->data.ang_velocity[2]); 
                }
                if (LPMS_ORIENT_QUAT_ENABLED & p_lpms->config_flags) { 
                     memcpy(p_lpms->data.orient_quat , &p_frame->data[i] , 4*4);
                    i = i + 16;
                    printf("Quat- %09f  %09f  %09f  %09f \n", p_lpms->data.orient_quat[0],
                                                        p_lpms->data.orient_quat[1],
                                                        p_lpms->data.orient_quat[2],
                                                        p_lpms->data.orient_quat[3]); 
                }
                if (LPMS_EULER_ANGLE_ENABLED & p_lpms->config_flags) { 
                    memcpy(p_lpms->data.euler_angle , &p_frame->data[i] , 4*3);
                    i = i + 12;
                    printf("Eulr- %09f  %09f  %09f \n", p_lpms->data.euler_angle[0],
                                                        p_lpms->data.euler_angle[1],
                                                        p_lpms->data.euler_angle[2]); 
                }
                if (LPMS_LINEAR_ACC_ENABLED & p_lpms->config_flags) { 
                    memcpy(p_lpms->data.linear_acc , &p_frame->data[i] , 4*3);
                    i = i + 12;
                    printf("lacc- %09f  %09f  %09f \n", p_lpms->data.linear_acc[0],
                                                        p_lpms->data.linear_acc[1],
                                                        p_lpms->data.linear_acc[2]); 
                }
                if (LPMS_PRESSURE_ENABLED & p_lpms->config_flags) { 
                    memcpy(&(p_lpms->data.baro_press), &p_frame->data[i], 4);
                    i=i+4;
                    printf("Pressure- %09f\n",p_lpms->data.baro_press); 
                }
                if (LPMS_ALTITUDE_ENABLED & p_lpms->config_flags) { 
                    memcpy(&(p_lpms->data.altitude), &p_frame->data[i], 4);
                    i=i+4;
                    printf("Altitude- %09f\n",p_lpms->data.altitude); 
                }
                if (LPMS_TEMPERATURE_ENABLED & p_lpms->config_flags) { 
                    memcpy(&(p_lpms->data.temperature), &p_frame->data[i], 4);
                    i=i+4;
                    printf("Temperature- %09f\n",p_lpms->data.temperature); 
                }
                if (LPMS_HEAVE_MOTION_ENABLED & p_lpms->config_flags) { 
                    memcpy(&(p_lpms->data.heave_motion), &p_frame->data[i], 4);
                    i=i+4;
                    printf("Heave- %09f\n",p_lpms->data.heave_motion); 
                }
             } else {

            }
            break;
        default:
            break;
    } //switch

}

void *  idler(void * p)
{
    uint16_t head,tail;
    lpms_t * p_lpms;

    uint8_t     buff[BUFFSIZE];
    lpms_frame_t   curr_frame;

    uint32_t seq=0;
    uint8_t done=0;
    uint16_t count;

    p_lpms = (lpms_t *)p;

    tail=0;

    while (1) {
        seq++;
        head=0;
        count = read(p_lpms->fd,&buff[tail],BUFFSIZE-tail);
        //printf("read %d bytes\n",count);
        done=0;
        if (count<0) {
            pthread_exit(NULL);
        } else {
            tail = tail + count;
            while (done==0) {
                //printf("State=%d   bufflen=%d\n",p_lpms->state,tail);

                switch (p_lpms->state) {

                    case LPMS_STATE_IDLE:
                        while ((p_lpms->state==LPMS_STATE_IDLE) && (head<tail)) {
                            if ( (buff[head]==0x3A) && (buff[head+1]==0x01)
                                                 && (buff[head+2]==0x00)) {
                                p_lpms->state = LPMS_STATE_HEADER;
                                break;
                            }
                            head = head + 1;
                        }
                        if (p_lpms->state == LPMS_STATE_IDLE) {
                            // no sync found, save last two bytes in case partial sync
                            if (tail>2) {
                                buff[0] = buff[tail-2];
                                buff[1] = buff[tail-1];
                                head=0;
                                tail = 2;
                            }
                            done=1;
                        }
                        break;

                    case LPMS_STATE_HEADER:
                        if ((tail-head) >= LPMS_HEADER_SIZE) {
                            readheader(&buff[head], &curr_frame);
                            p_lpms->state = LPMS_STATE_DATA;
                        } else {
                            done=1;
                        }
                        break;
  
                    case LPMS_STATE_DATA:
                        if ((tail-head) >= LPMS_HEADER_SIZE + curr_frame.header.length + 4) {
                            datacpy(&buff[head + LPMS_HEADER_SIZE],&curr_frame);
                            p_lpms->state = LPMS_STATE_FRAME_COMPLETE;
                        } else {
                            done=1;
                        }
                        break;

                    case LPMS_STATE_FRAME_COMPLETE:
                        printf("%02X:",curr_frame.header.command);
                        //if (curr_frame.header.length > 0 ) {
                        //    dump(curr_frame.data,30);
                        //} else {
                        //    printf("----\n");
                        //}
                        procframe(p_lpms,&curr_frame);
                        p_lpms->state = LPMS_STATE_IDLE;
                        head= head + LPMS_HEADER_SIZE + curr_frame.header.length + 4;
                        break;

                    default:
                        break;
                }
            } // while (done)
            //printf("tail=%d   head=%d  diff=%d\n",tail,head,tail-head);
            tail = tail - head;
            memcpy(buff,&buff[head], tail);
            head=0;
        } //if count>0
    } //while(1)
}

uint32_t lpms_init(const char * portname, lpms_t * p_lpms)
{
    uint32_t err;
    int i;
    lpms_frame_t frame;
    uint8_t buff[1024];
    
    frame.header.start= 0x3A;
    frame.header.openmat_id = 0x0001;
    frame.header.command = LPMS_COMMAND_GOTO_COMMAND_MODE;
    frame.header.length = 0;
    
    i = frame_to_buff(buff,&frame);

    err=serial_open(portname,B921600,&p_lpms->fd);
    printf("Serial Open returned %d\n",err);
    err=write(p_lpms->fd,buff,i);
    tcflush(p_lpms->fd,TCIFLUSH);
    printf("Got return of %d writing %d bytes\n",err,i);
    dump(buff,i);
    
    
    frame.header.command = LPMS_COMMAND_GET_CONFIG;
    i = frame_to_buff(buff,&frame);
    write(p_lpms->fd,buff,i);

    tcflush(p_lpms->fd,TCIFLUSH);
    dump(buff,i);
    frame.header.command = LPMS_COMMAND_GOTO_STREAM_MODE;
    i = frame_to_buff(buff,&frame);
    err=write(p_lpms->fd,buff,i);

    tcflush(p_lpms->fd,TCIFLUSH);
    printf("Got return of %d writing %d bytes\n",err,i);

//    if (err!=LPMS_SUCCESS) {
  //      return err;
 ///   }

    pthread_create(&p_lpms->pt_handle,NULL, &idler,(void *) p_lpms);
    return LPMS_SUCCESS;
}




uint32_t lpms_stop(lpms_t * p_lpms) {

    pthread_join(p_lpms->pt_handle, NULL);
    printf("Thread done\n");
    return LPMS_SUCCESS;
}


