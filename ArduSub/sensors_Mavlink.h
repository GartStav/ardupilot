/*
 * sensors_Mavlink.h
 *
 *  Created on: Jun 6, 2018
 *      Author: agritsenko
 */

#ifndef ARDUSUB_SENSORS_MAVLINK_H_
#define ARDUSUB_SENSORS_MAVLINK_H_

struct sensor_frame_t
{
    mavlink_attitude_t attitude;
    mavlink_raw_imu_t  imu;
    mavlink_altitude_t altitude;
    mavlink_global_position_int_t gps;
};

#endif /* ARDUSUB_SENSORS_MAVLINK_H_ */
