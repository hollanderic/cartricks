#include <pthread.h>

#ifndef _LPMS_H
#define _LPMS_H 1

#define LPMS_SUCCESS 0
#define LPMS_ERR_PORT 1
#define LPMS_HEADER_SIZE 7

#define LPMS_PRESSURE_ENABLED           1 << 9
#define LPMS_MAGNETOMETER_ENABLED       1 << 10
#define LPMS_ACCELEROMETER_ENABLED      1 << 11
#define LPMS_GYROSCOPE_ENABLED          1 << 12
#define LPMS_TEMPERATURE_ENABLED        1 << 13
#define LPMS_HEAVE_MOTION_ENABLED       1 << 14
#define LPMS_ANGULAR_VELOCITY_ENABLED   1 << 16
#define LPMS_EULER_ANGLE_ENABLED        1 << 17
#define LPMS_ORIENT_QUAT_ENABLED        1 << 18
#define LPMS_ALTITUDE_ENABLED           1 << 19
#define LPMS_LINEAR_ACC_ENABLED         1 << 21


#define LPMS_COMMAND_GOTO_COMMAND_MODE  6
#define LPMS_COMMAND_GOTO_STREAM_MODE   7
#define LPMS_COMMAND_GET_CONFIG         4

#define LPMS_COMMAND_GET_SENSOR_DATA    9


typedef enum {
    LPMS_STATE_IDLE,
    LPMS_STATE_HEADER,
    LPMS_STATE_DATA,
    LPMS_STATE_FRAME_COMPLETE,
} lpms_states_t;

typedef struct lpmsdata
{
    uint32_t    flags;
    
    float   timestamp;
    float   cal_gyro[3];
    float   cal_acc[3];
    float   cal_mag[3];
    float   ang_velocity[3];
    float   orient_quat[4];
    float   euler_angle[3];
    float   linear_acc[3];
    float   baro_press;
    float   altitude;
    float   temperature;
    float   heave_motion;
} lpms_data_t;



typedef struct lpmsheader
{
    uint16_t     start;
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
    lpms_data_t     data;           //most recent data
    uint32_t        config_flags;
} lpms_t;

uint32_t lpms_stop(lpms_t * p_lpms);

uint32_t lpms_init(const char * portname, lpms_t * p_lpms);


#endif /* lpms.h */
