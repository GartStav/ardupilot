/*
 * sensors_Mavlink.h
 *
 *  Created on: Jun 6, 2018
 *      Author: agritsenko
 */

#ifndef ARDUSUB_SENSORS_MAVLINK_H_
#define ARDUSUB_SENSORS_MAVLINK_H_

struct attitude_sensor_t
{
    uint32_t time;
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;
};

struct imu_sensor_t
{

};

struct altitude_sensor_t
{

};

struct gps_sensor_t
{

};

struct sensor_frame_t
{
    attitude_sensor_t attitude;
    imu_sensor_t      imu;
    altitude_sensor_t altitude;
    gps_sensor_t      gps;
};

#endif /* ARDUSUB_SENSORS_MAVLINK_H_ */
