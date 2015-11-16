#include <pthread.h>

#ifndef _GPS_H
#define _GPS_H 1

#define GPS_SUCCESS 0
#define GPS_ERR_PORT 1
#define GPS_HEADER_SIZE 7

#define GPS_PRESSURE_ENABLED           1 << 9
#define GPS_MAGNETOMETER_ENABLED       1 << 10
#define GPS_ACCELEROMETER_ENABLED      1 << 11
#define GPS_GYROSCOPE_ENABLED          1 << 12
#define GPS_TEMPERATURE_ENABLED        1 << 13
#define GPS_HEAVE_MOTION_ENABLED       1 << 14
#define GPS_ANGULAR_VELOCITY_ENABLED   1 << 16
#define GPS_EULER_ANGLE_ENABLED        1 << 17
#define GPS_ORIENT_QUAT_ENABLED        1 << 18
#define GPS_ALTITUDE_ENABLED           1 << 19
#define GPS_LINEAR_ACC_ENABLED         1 << 21


#define GPS_COMMAND_GOTO_COMMAND_MODE  6
#define GPS_COMMAND_GOTO_STREAM_MODE   7
#define GPS_COMMAND_GET_CONFIG         4

#define GPS_COMMAND_GET_SENSOR_DATA    9


typedef enum {
    GPS_STATE_IDLE,
    GPS_STATE_HEADER,
    GPS_STATE_DATA,
    GPS_STATE_FRAME_COMPLETE,
} gps_states_t;

typedef struct gpsdata
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
} gps_data_t;



typedef struct gpsheader
{
    uint16_t     start;
    uint16_t    openmat_id;
    uint16_t    command;
    uint16_t    length;
} gps_header_t;

typedef struct gpsframe
{
    gps_header_t   header;
    uint8_t         data[1024];
    uint16_t        crc;
    uint16_t        ender;
} gps_frame_t;

typedef struct gpst
{
    int32_t         fd;
    pthread_t       pt_handle;
    gps_states_t   state;
    gps_data_t     data;           //most recent data
    uint32_t        config_flags;
} gps_t;

uint32_t gps_stop(gps_t * p_gps);

uint32_t gps_init(const char * portname, gps_t * p_gps);


#endif /* gps.h */
