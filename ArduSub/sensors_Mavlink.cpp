#include "Sub.h"

#include <mutex>

// last "synchronized" snapshot of the sensor data
// NOTE it is not really synchronized, as data can have
// different timestamps
// Usage: a) if we only using one type of sensor data we should
// use _last_<sensor> instead of sensor frame b) if we need precisely
// synchronized data sensor frame is of a little use, consumer would
// need to perform synchronization. c) if we need some sort of data
// synch, but OK with some error we can use sensor frame
static sensor_frame_t _last_sensor_frame;

static mavlink_attitude_t _last_attitude;
static mavlink_raw_imu_t  _last_imu;
static mavlink_altitude_t _last_altitude;
static mavlink_global_position_int_t _last_gps;

static std::mutex sf_mutex;

void Sub::sensor_frame_update()
{
    std::lock_guard<std::mutex> guard(sf_mutex);
    _last_sensor_frame.altitude = _last_altitude;
    _last_sensor_frame.attitude = _last_attitude;
    _last_sensor_frame.gps = _last_gps;
    _last_sensor_frame.imu = _last_imu;
}

void Sub::get_last_sensor_frame(sensor_frame_t *sensor_frame) const
{
    std::lock_guard<std::mutex> guard(sf_mutex);
    sensor_frame->altitude = _last_sensor_frame.altitude;
    sensor_frame->attitude = _last_sensor_frame.attitude;
    sensor_frame->gps = _last_sensor_frame.gps;
    sensor_frame->imu = _last_sensor_frame.imu;
}

void Sub::update_altitude(const mavlink_message_t &msg)
{
    mavlink_altitude_t temp_altitude;
    mavlink_msg_altitude_decode(&msg, &temp_altitude);

    std::lock_guard<std::mutex> guard(sf_mutex);
    _last_altitude = temp_altitude;
}

void Sub::update_imu(const mavlink_message_t &msg)
{
    mavlink_raw_imu_t temp_imu;
    mavlink_msg_raw_imu_decode(&msg, &temp_imu);

    std::lock_guard<std::mutex> guard(sf_mutex);
    _last_imu = temp_imu;
}

void Sub::update_attitude(const mavlink_message_t &msg)
{
    mavlink_attitude_t temp_attitude;
    mavlink_msg_attitude_decode(&msg, &temp_attitude);

    std::lock_guard<std::mutex> guard(sf_mutex);
    _last_attitude = temp_attitude;
}

void Sub::update_gps(const mavlink_message_t &msg)
{
    mavlink_global_position_int_t temp_gps;
    mavlink_msg_global_position_int_decode(&msg, &temp_gps);

    std::lock_guard<std::mutex> guard(sf_mutex);
    _last_gps = temp_gps;
}

inline const mavlink_altitude_t Sub::get_altitude() const
{
    std::lock_guard<std::mutex> guard(sf_mutex);
    return _last_altitude;
}

inline const mavlink_attitude_t Sub::get_attitude() const
{
    std::lock_guard<std::mutex> guard(sf_mutex);
    return _last_attitude;
}

inline const mavlink_raw_imu_t Sub::get_imu() const
{
    std::lock_guard<std::mutex> guard(sf_mutex);
    return _last_imu;
}

inline const mavlink_global_position_int_t Sub::get_gps() const
{
    std::lock_guard<std::mutex> guard(sf_mutex);
    return _last_gps;
}
