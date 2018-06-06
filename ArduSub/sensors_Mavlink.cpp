#include <Sub.h>

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

static attitude_sensor_t _last_attitude;
static imu_sensor_t      _last_imu;
static altitude_sensor_t _last_altitude;
static gps_sensor_t      _last_gps;

static std::mutex sf_mutex;

// the idea behind the sensor frame is that it would have
void Sub::sensor_frame_update()
{
    sensor_frame_t
    std::lock_guard<std::mutex> guard(sf_mutex);
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
    _last_attitude = msg;
}

void Sub::update_imu(const mavlink_message_t &msg)
{
    _last_imu = msg;
}

void Sub::update_attitude(const mavlink_message_t &msg)
{
    _last_attitude = msg;
}

void Sub::update_gps(const mavlink_message_t &msg)
{
    _last_gps = msg;
}

inline const altitude_sensor_t Sub::get_altitude() const
{
    return _last_altitude;
}

inline const attitude_sensor_t Sub::get_attitude() const
{
    return _last_attitude;
}

inline const imu_sensor_t Sub::get_imu() const
{
    return _last_imu;
}

inline const gps_sensor_t Sub::get_gps() const
{
    return _last_gps;
}
