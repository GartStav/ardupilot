/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// ArduSub scheduling, originally copied from ArduCopter

#include "Sub.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    SCHED_TASK(fifty_hz_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,  200,    160),
#endif
//    SCHED_TASK(update_batt_compass,   10,    120),
//    SCHED_TASK(read_rangefinder,      20,    100),
//    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(read_mavlink_sensors,  10,    100), // TODO we should run this faster - 20 Hz?
    SCHED_TASK(sensor_frame_update,   10,    100),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(update_turn_counter,   10,     50),
//    SCHED_TASK(compass_accumulate,   100,    100),
//    SCHED_TASK(barometer_accumulate,  50,     90),
    SCHED_TASK(update_notify,         50,     90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK(gcs_check_input,      400,    180),
    SCHED_TASK(gcs_send_heartbeat,     1,    110),
    SCHED_TASK(gcs_send_deferred,     50,    550),
    SCHED_TASK(gcs_data_stream_send,  50,    550),
    SCHED_TASK(update_mount,          50,     75),
#if CAMERA == ENABLED
    SCHED_TASK(update_trigger,        50,     75),
#endif
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK(dataflash_periodic,    400,    300),
    SCHED_TASK(ins_periodic,          400,    50),
    SCHED_TASK(perf_update,           0.1,    75),
#if RPM_ENABLED == ENABLED
    SCHED_TASK(rpm_update,            10,    200),
#endif
//    SCHED_TASK(compass_cal_update,   100,    100),
//    SCHED_TASK(accel_cal_update,      10,    100),
//    SCHED_TASK(terrain_update,        10,    100),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK(gripper_update,            10,     75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
};

void Sub::setup()
{
    printf("Hi, Artem!\n");
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));

    // setup initial performance counters
    perf_info_reset();
    fast_loopTimer = AP_HAL::micros();

    init_mavlink_sensors();
}

/*
  try to accumulate a baro reading
 */
void Sub::barometer_accumulate(void)
{
    barometer.accumulate();
}

void Sub::perf_update(void)
{
    if (should_log(MASK_LOG_PM)) {
        Log_Write_Performance();
    }
    if (scheduler.debug()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "PERF: %u/%u %lu %lu",
                          (unsigned)perf_info_get_num_long_running(),
                          (unsigned)perf_info_get_num_loops(),
                          (unsigned long)perf_info_get_max_time(),
                          (unsigned long)perf_info_get_min_time());
    }
    perf_info_reset();
    pmTest1 = 0;
}

void Sub::loop()
{
    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    // check loop time
    perf_info_check_loop_time(timer - fast_loopTimer);

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.0f;
    fast_loopTimer          = timer;

    // for mainloop failure monitoring
    mainLoop_count++;

    // Execute the fast loop
    // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    const uint32_t loop_us = scheduler.get_loop_period_us();
    const uint32_t time_available = (timer + loop_us) - micros();
    scheduler.run(time_available > loop_us ? 0u : time_available);
}


// Main loop - 400hz
void Sub::fast_loop()
{
    // update INS immediately to get current gyro data populated
    //ins.update();

    // we need to update from mavlink here

    ins.update();

    if (control_mode != MANUAL) { //don't run rate controller in manual mode
        // run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }

    // send outputs to the motors library
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've reached the surface or bottom
    update_surface_and_bottom_detector();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }
}

// 50 Hz tasks
void Sub::fifty_hz_loop()
{
    // check pilot input failsafe
    failsafe_pilot_input_check();

    failsafe_crash_check();

    failsafe_ekf_check();

    failsafe_sensors_check();

    // Update rc input/output
    RC_Channels::set_pwm_all();
    SRV_Channels::output_ch_all();
}

// updates the status of notify
// should be called at 50hz
void Sub::update_notify()
{
    notify.update();
}

// update_mount - update camera mount position
// should be run at 50hz
void Sub::update_mount()
{
#if MOUNT == ENABLED
    // update camera mount's position
    camera_mount.update();
#endif
}

#if CAMERA == ENABLED
// update camera trigger
void Sub::update_trigger(void)
{
    camera.update_trigger();
}
#endif

// update_batt_compass - read battery and compass
// should be called at 10hz
void Sub::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    read_battery();

    if (g.compass_enabled) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
        // log compass information
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info());
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        DataFlash.Log_Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        DataFlash.Log_Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && mode_requires_GPS(control_mode)) {
        Log_Write_Nav_Tuning();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_Vibration(ins);
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control.control_monitor_log();
    }
}

// twentyfive_hz_logging_loop
// should be run at 25hz
void Sub::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        DataFlash.Log_Write_Rate(ahrs, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            DataFlash.Log_Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            DataFlash.Log_Write_PID(LOG_PIDA_MSG, g.pid_accel_z.get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        DataFlash.Log_Write_IMU(ins);
    }
}

void Sub::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

void Sub::ins_periodic()
{
    ins.periodic();
}

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    leak_detector.update();

    failsafe_leak_check();

    failsafe_internal_pressure_check();

    failsafe_internal_temperature_check();

    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

    ServoRelayEvents.update_events();
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    hal.console->print("1 Hz loop passed \n");
    printf("1 Hz loop passed \n");
    bool arm_check = arming.pre_arm_checks(false);
    ap.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_gps_check = position_ok();

    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(DATA_AP_STATE, ap.value);
    }

    if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.set_orientation();

        // set all throttle channel settings
        motors.set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // update position controller alt limits
    update_poscon_alt_max();

    // log terrain data
    terrain_logging();

    Vector3f vec_neu;
    current_loc.get_vector_from_origin_NEU(vec_neu);
    printf("Current location (%f, %f, %f) \n", vec_neu.x, vec_neu.y, vec_neu.z);
}

// called at 50hz
void Sub::update_GPS(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            // log GPS message
            if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
                DataFlash.Log_Write_GPS(gps, i);
            }

            gps_updated = true;
        }
    }

    if (gps_updated) {
        // set system time if necessary
        set_system_time_from_GPS();

#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Sub::read_AHRS(void)
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    // <true> tells AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
    ahrs_view.update(true);
}

// read baro and rangefinder altitude at 10hz
void Sub::update_altitude()
{
    // read in baro altitude
    read_barometer();

    // write altitude info to dataflash logs
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

// Initialize socket to receive sensor data over UDP

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

void Sub::init_mavlink_sensors()
{
    printf("init mavlink sensors \n");

    char target_ip[100];

    _sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    // CSet target ip to localhost
    strcpy(target_ip, "127.0.0.1");

    const int CLIENT_ADDRESS = 14550;
    memset(&_locAddr, 0, sizeof(_locAddr));
    _locAddr.sin_family = AF_INET;
    _locAddr.sin_addr.s_addr = INADDR_ANY;
    _locAddr.sin_port = htons(CLIENT_ADDRESS);

    /* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */
    if (-1 == bind(_sock,(struct sockaddr *)&_locAddr, sizeof(struct sockaddr)))
    {
        perror("error bind failed");
        close(_sock);
        exit(EXIT_FAILURE);
    }

    if (fcntl(_sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
    {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(_sock);
        exit(EXIT_FAILURE);
    }

    const int SERVER_ADDRESS = 14551;
    memset(&_gcAddr, 0, sizeof(_gcAddr));
    _gcAddr.sin_family = AF_INET;
    _gcAddr.sin_addr.s_addr = inet_addr(target_ip);
    _gcAddr.sin_port = htons(SERVER_ADDRESS);
}

void Sub::handle_new_message(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_ALTITUDE:
        printf("received message %d \n", msg.msgid);
        update_altitude(msg);
        break;
    case MAVLINK_MSG_ID_RAW_IMU:
        printf("received message %d \n", msg.msgid);
        update_imu(msg);
        break;
    case MAVLINK_MSG_ID_ATTITUDE:
        printf("received message %d \n", msg.msgid);
        update_attitude(msg);
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        printf("received message %d \n", msg.msgid);
        update_gps(msg);
        break;
    default:
        printf("Unhandled message %d \n", msg.msgid);
        break;
    }
}

// Read mavlink sensor data and cashes them into structure in sensors.cpp
void Sub::read_mavlink_sensors()
{
    ssize_t recsize;
    uint8_t buf[BUFFER_LENGTH];
    // unsigned int temp = 0;

    memset(buf, 0, BUFFER_LENGTH);
    recsize = recvfrom(_sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&_gcAddr, &_fromlen);
    if (recsize > 0)
    {
        // Something received - print out all bytes and parse packet
        mavlink_message_t msg;
        mavlink_status_t status;

        // printf("Bytes Received: %d\nDatagram: ", (int)recsize);
        for (int i = 0; i < recsize; ++i)
        {
            // temp = buf[i];
            // printf("%02x ", (unsigned char)temp);
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
            {
                // Packet received
                // printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                handle_new_message(msg);
            }
        }
        // printf("\n");
    }
    memset(buf, 0, BUFFER_LENGTH);
}

AP_HAL_MAIN_CALLBACKS(&sub);
