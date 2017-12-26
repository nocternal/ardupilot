/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
   Lead developer: Andrew Tridgell
 
   Authors:    Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

   Please contribute your ideas! See http://dev.ardupilot.org for details

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

#include "Plane.h"

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Plane, &plane, func, rate_hz, max_time_micros)


/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in Hz) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
                           // Units:   Hz      us
    SCHED_TASK(ahrs_update,           400,    400),
    SCHED_TASK(read_radio,             50,    100),
    SCHED_TASK(check_short_failsafe,   50,    100),
    SCHED_TASK(update_speed_height,    50,    200),
    SCHED_TASK(update_flight_mode,    400,    100),
    SCHED_TASK(stabilize,             400,    100),
    SCHED_TASK(set_servos,            400,    100),
    SCHED_TASK(read_control_switch,     7,    100),
    SCHED_TASK(gcs_retry_deferred,     50,    500),
    SCHED_TASK(update_GPS_50Hz,        50,    300),
    SCHED_TASK(update_GPS_10Hz,        10,    400),
    SCHED_TASK(navigate,               10,    150),
    SCHED_TASK(update_compass,         10,    200),
    SCHED_TASK(read_airspeed,          10,    100),
    SCHED_TASK(update_alt,             10,    200),
    SCHED_TASK(adjust_altitude_target, 10,    200),
    SCHED_TASK(afs_fs_check,           10,    100),
    SCHED_TASK(gcs_update,             50,    500),
    SCHED_TASK(gcs_data_stream_send,   50,    500),
    SCHED_TASK(update_events,          50,    150),
    SCHED_TASK(check_usb_mux,          10,    100),
    SCHED_TASK(read_battery,           10,    300),
    SCHED_TASK(compass_accumulate,     50,    200),
    SCHED_TASK(barometer_accumulate,   50,    150),
    SCHED_TASK(update_notify,          50,    300),
    SCHED_TASK(read_rangefinder,       50,    100),
    SCHED_TASK(ice_update,             10,    100),
    SCHED_TASK(compass_cal_update,     50,    50),
    SCHED_TASK(accel_cal_update,       10,    50),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,    50,    50),
#endif
    SCHED_TASK(one_second_loop,         1,    400),
    SCHED_TASK(check_long_failsafe,     3,    400),
    SCHED_TASK(read_receiver_rssi,     10,    100),
    SCHED_TASK(rpm_update,             10,    100),
    SCHED_TASK(airspeed_ratio_update,   1,    100),
    SCHED_TASK(update_mount,           50,    100),
    SCHED_TASK(update_trigger,         50,    100),
    SCHED_TASK(log_perf_info,         0.2,    100),
    SCHED_TASK(compass_save,          0.1,    200),
    SCHED_TASK(Log_Write_Fast,         25,    300),
    SCHED_TASK(update_logging1,        10,    300),
    SCHED_TASK(update_logging2,        10,    300),
    SCHED_TASK(parachute_check,        10,    200),
    SCHED_TASK(terrain_update,         10,    200),
    SCHED_TASK(update_is_flying_5Hz,    5,    100),
    SCHED_TASK(dataflash_periodic,     50,    400),
    SCHED_TASK(avoidance_adsb_update,  10,    100),
    SCHED_TASK(button_update,           5,    100),
};

void Plane::setup() 
{
    cliSerial = hal.console;

    // load the default values of variables listed in var_info[]
    AP_Param::setup_sketch_defaults();

    AP_Notify::flags.failsafe_battery = false;

    notify.init(false);

    rssi.init();

    init_ardupilot();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

void Plane::loop()
{
    uint32_t loop_us = 1000000UL / scheduler.get_loop_rate_hz();

    // wait for an INS sample
    ins.wait_for_sample();

    uint32_t timer = micros();

    perf.delta_us_fast_loop  = timer - perf.fast_loopTimer_us;
    G_Dt = perf.delta_us_fast_loop * 1.0e-6f;

    if (perf.delta_us_fast_loop > loop_us + 500) {
        perf.num_long++;
    }

    if (perf.delta_us_fast_loop > perf.G_Dt_max && perf.fast_loopTimer_us != 0) {
        perf.G_Dt_max = perf.delta_us_fast_loop;
    }

    if (perf.delta_us_fast_loop < perf.G_Dt_min || perf.G_Dt_min == 0) {
        perf.G_Dt_min = perf.delta_us_fast_loop;
    }
    perf.fast_loopTimer_us = timer;

    perf.mainLoop_count++;

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all the tasks that are due to run. Note that we only
    // have to call this once per loop, as the tasks are scheduled
    // in multiples of the main loop tick. So if they don't run on
    // the first call to the scheduler they won't run on a later
    // call until scheduler.tick() is called again
    scheduler.run(loop_us);
}

// update AHRS system
void Plane::ahrs_update()
{
    hal.util->set_soft_armed(arming.is_armed() &&
                   hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // update hil before AHRS update
        gcs_update();
    }
#endif

    ahrs.update();

    if (should_log(MASK_LOG_IMU)) {
        Log_Write_IMU();
    }

    // calculate a scaled roll limit based on current pitch
    roll_limit_cd = aparm.roll_limit_cd * cosf(ahrs.pitch);
    pitch_limit_min_cd = aparm.pitch_limit_min_cd * fabsf(cosf(ahrs.roll));

    // updated the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);

    // update inertial_nav for quadplane
    quadplane.inertial_nav.update(G_Dt);
}

/*
  update 50Hz speed/height controller
 */
void Plane::update_speed_height(void)
{
    if (auto_throttle_mode) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // throttle suppressed, as this needs to be running for
	    // takeoff detection
        SpdHgt_Controller->update_50hz();
    }
}


/*
  update camera mount
 */
void Plane::update_mount(void)
{
#if MOUNT == ENABLED
    camera_mount.update();
#endif
}

/*
  update camera trigger
 */
void Plane::update_trigger(void)
{
#if CAMERA == ENABLED
    camera.trigger_pic_cleanup();
    if (camera.check_trigger_pin()) {
        gcs_send_message(MSG_CAMERA_FEEDBACK);
        if (should_log(MASK_LOG_CAMERA)) {
            DataFlash.Log_Write_Camera(ahrs, gps, current_loc);
        }
    }    
#endif
}

/*
  read and update compass
 */
void Plane::update_compass(void)
{
    if (g.compass_enabled && compass.read()) {
        ahrs.set_compass(&compass);
        compass.learn_offsets();
        if (should_log(MASK_LOG_COMPASS) && !ahrs.have_ekf_logging()) {
            DataFlash.Log_Write_Compass(compass);
        }
    } else {
        ahrs.set_compass(NULL);
    }
}

/*
  if the compass is enabled then try to accumulate a reading
 */
void Plane::compass_accumulate(void)
{
    if (g.compass_enabled) {
        compass.accumulate();
    }    
}

/*
  try to accumulate a baro reading
 */
void Plane::barometer_accumulate(void)
{
    barometer.accumulate();
}

/*
  do 10Hz logging
 */
void Plane::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_IMU))
        Log_Write_IMU();
}

/*
  do 10Hz logging - part2
 */
void Plane::update_logging2(void)
{
    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();
    
    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        DataFlash.Log_Write_Vibration(ins);
}


/*
  check for AFS failsafe check
 */
void Plane::afs_fs_check(void)
{
    // perform AFS failsafe checks
    afs.check(failsafe.last_heartbeat_ms, geofence_breached(), failsafe.AFS_last_valid_rc_ms);
}


/*
  update aux servo mappings
 */
void Plane::update_aux(void)
{
    RC_Channel_aux::enable_aux_servos();
}

void Plane::one_second_loop()
{
    /////////
    //////////////////////////// Temporary JU debug message
    /////////
    /*gcs_send_text_fmt(MAV_SEVERITY_INFO, "Hdotc=%.1f ,Vc=%.1f , Phic=%.1f ,rc=%.1f",
                              (double)Ju_Joystick_Hdotc,
                              (double)Ju_Joystick_Vc,
                              (double)(Ju_Joystick_Phic*57.3f),
                              (double)(Ju_Joystick_rc*57.3f));*/

    /*gcs_send_text_fmt(MAV_SEVERITY_INFO, "Hdotc=%.1f ,HdotRM=%.1f",
                              (float)Ju_Joystick_Hdotc,
                              (float)Ju_Ref_Hdot);*/
    /*gcs_send_text_fmt(MAV_SEVERITY_INFO, "dtime=%.6f s",(float)jdelta_time);*/
    /////////
    ////////////////////////////                   
   
    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);

    // make it possible to change control channel ordering at runtime
    set_control_channels();

#if HAVE_PX4_MIXER
    if (!hal.util->get_soft_armed() && (last_mixer_crc == -1)) {
        // if disarmed try to configure the mixer
        setup_failsafe_mixing();
    }
#endif // CONFIG_HAL_BOARD

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    adsb.set_stall_speed_cm(aparm.airspeed_min);

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

    update_aux();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::NO;

#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data(DataFlash);
    }
#endif

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));

    // update home position if soft armed and gps position has
    // changed. Update every 5s at most
    if (!hal.util->get_soft_armed() &&
        gps.last_message_time_ms() - last_home_update_ms > 5000 &&
        gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            last_home_update_ms = gps.last_message_time_ms();
            update_home();
            
            // reset the landing altitude correction
            auto_state.land_alt_offset = 0;
    }
}

void Plane::log_perf_info()
{
    if (scheduler.debug() != 0) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "PERF: %u/%u Dt=%u/%u Log=%u\n",
                          (unsigned)perf.num_long,
                          (unsigned)perf.mainLoop_count,
                          (unsigned)perf.G_Dt_max,
                          (unsigned)perf.G_Dt_min,
                          (unsigned)(DataFlash.num_dropped() - perf.last_log_dropped));
    }

    if (should_log(MASK_LOG_PM)) {
        Log_Write_Performance();
    }

    resetPerfData();
}

void Plane::compass_save()
{
    if (g.compass_enabled &&
        compass.get_learn_type() >= Compass::LEARN_INTERNAL &&
        !hal.util->get_soft_armed()) {
        /*
          only save offsets when disarmed
         */
        compass.save_offsets();
    }
}

void Plane::terrain_update(void)
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();
#endif
}


void Plane::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

/*
  once a second update the airspeed calibration ratio
 */
void Plane::airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max_cd ||
        ahrs.pitch_sensor < pitch_limit_min_cd) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg, aparm.airspeed_max);
    gcs_send_airspeed_calibration(vg);
}


/*
  read the GPS and update position
 */
void Plane::update_GPS_50Hz(void)
{
    // get position from AHRS
    have_position = ahrs.get_position(current_loc);

    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];
    gps.update();

    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                Log_Write_GPS(i);
            }
        }
    }
}

/*
  read update GPS position - 10Hz update
 */
void Plane::update_GPS_10Hz(void)
{
    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0 && current_loc.lng == 0) {
                ground_start_count = 5;

            } else {
                init_home();

                // set system clock for log timestamps
                uint64_t gps_timestamp = gps.time_epoch_usec();
                
                hal.util->set_system_clock(gps_timestamp);

                // update signing timestamp
                GCS_MAVLINK::update_signing_timestamp(gps_timestamp);

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    const Location &loc = gps.location();
                    compass.set_initial_location(loc.lat, loc.lng);
                }
                ground_start_count = 0;
            }
        }

        // see if we've breached the geo-fence
        geofence_check(false);

#if CAMERA == ENABLED
        if (camera.update_location(current_loc, plane.ahrs ) == true) {
            do_take_picture();
        }
#endif        

        // update wind estimate
        ahrs.estimate_wind();
    } else if (gps.status() < AP_GPS::GPS_OK_FIX_3D && ground_start_count != 0) {
        // lost 3D fix, start again
        ground_start_count = 5;
    }

    calc_gndspeed_undershoot();
}

/*
  main handling for AUTO mode
 */
void Plane::handle_auto_mode(void)
{
    uint16_t nav_cmd_id;

    if (mission.state() != AP_Mission::MISSION_RUNNING) {
        // this should never be reached
        set_mode(RTL, MODE_REASON_MISSION_END);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    nav_cmd_id = mission.get_current_nav_cmd().id;

    if (quadplane.in_vtol_auto()) {
        quadplane.control_auto(next_WP_loc);
    } else if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && flight_stage == AP_SpdHgtControl::FLIGHT_LAND_ABORT)) {
        takeoff_calc_roll();
        takeoff_calc_pitch();
        calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        calc_nav_roll();
        calc_nav_pitch();
        
        if (auto_state.land_complete) {
            // during final approach constrain roll to the range
            // allowed for level flight
            nav_roll_cd = constrain_int32(nav_roll_cd, -g.level_roll_limit*100UL, g.level_roll_limit*100UL);
        }
        calc_throttle();
        
        if (auto_state.land_complete) {
            // we are in the final stage of a landing - force
            // zero throttle
            channel_throttle->set_servo_out(0);
        }
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            steer_state.hold_course_cd = -1;
        }
        auto_state.land_complete = false;
        auto_state.land_pre_flare = false;
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
    }
}

/*
  main flight mode dependent update code 
 */
void Plane::update_flight_mode(void)
{   
    enum FlightMode effective_mode = control_mode;
    if (control_mode == AUTO && g.auto_fbw_steer == 42) {
        effective_mode = FLY_BY_WIRE_A;
    }

    if (effective_mode != AUTO) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }

    // ensure we are fly-forward
    if (quadplane.in_vtol_mode()) {
        ahrs.set_fly_forward(false);
    } else {
        ahrs.set_fly_forward(true);
    }

    switch (effective_mode) 
    {
    case AUTO:
        handle_auto_mode();

    /////////////////////////////////////////Temp
        // 计时器，用于积分、淡化等
        jtnow= AP_HAL::millis();
        jdt = jtnow - jlast_t;  // [ms]
        if (jlast_t == 0 || jdt > 1000) {
        jdt = 0;
        }
        jlast_t     = jtnow;  
        jdelta_time = (float)jdt * 0.001f; // [s]
        
        Ju_Sensor_MEAS();  // 传感器估计
        Ju_Joystick_CMD(); // 各操纵杆对应的下沉率、滚转角、偏航角速度、速度指令
        Ju_HdotV_Ctrl();   // 纵向控制器 ,输出de[rad] dthr[%]
        Ju_Phi_Ctrl();     // 横航向控制器，输出da[rad],dr[rad]

        if (jinit_counter <= (g.JU_Init_Transtime*1000)) 
        {
            jinit_counter += jdt;   
        }
    ///////////////////////////////////////////////

        break;

    case AVOID_ADSB:
    case GUIDED:
        if (auto_state.vtol_loiter && quadplane.available()) {
            quadplane.guided_update();
            break;
        }
        // fall through

    case RTL:
    case LOITER:
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
        break;
        
    case TRAINING: {
        training_manual_roll = false;
        training_manual_pitch = false;
        update_load_factor();
        
        // if the roll is past the set roll limit, then
        // we set target roll to the limit
        if (ahrs.roll_sensor >= roll_limit_cd) {
            nav_roll_cd = roll_limit_cd;
        } else if (ahrs.roll_sensor <= -roll_limit_cd) {
            nav_roll_cd = -roll_limit_cd;                
        } else {
            training_manual_roll = true;
            nav_roll_cd = 0;
        }
        
        // if the pitch is past the set pitch limits, then
        // we set target pitch to the limit
        if (ahrs.pitch_sensor >= aparm.pitch_limit_max_cd) {
            nav_pitch_cd = aparm.pitch_limit_max_cd;
        } else if (ahrs.pitch_sensor <= pitch_limit_min_cd) {
            nav_pitch_cd = pitch_limit_min_cd;
        } else {
            training_manual_pitch = true;
            nav_pitch_cd = 0;
        }
        if (fly_inverted()) {
            nav_pitch_cd = -nav_pitch_cd;
        }
        break;
    }

    case ACRO: {
        // handle locked/unlocked control
        if (acro_state.locked_roll) {
            nav_roll_cd = acro_state.locked_roll_err;
        } else {
            nav_roll_cd = ahrs.roll_sensor;
        }
        if (acro_state.locked_pitch) {
            nav_pitch_cd = acro_state.locked_pitch_cd;
        } else {
            nav_pitch_cd = ahrs.pitch_sensor;
        }
        break;
    }

    case AUTOTUNE:
    case FLY_BY_WIRE_A: {
        // set nav_roll and nav_pitch using sticks
        nav_roll_cd  = channel_roll->norm_input() * roll_limit_cd;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        update_load_factor();
        float pitch_input = channel_pitch->norm_input();
        if (pitch_input > 0) {
            nav_pitch_cd = pitch_input * aparm.pitch_limit_max_cd;
        } else {
            nav_pitch_cd = -(pitch_input * pitch_limit_min_cd);
        }
        adjust_nav_pitch_throttle();
        nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
        if (fly_inverted()) {
            nav_pitch_cd = -nav_pitch_cd;
        }
        if (failsafe.ch3_failsafe && g.short_fs_action == 2) {
            // FBWA failsafe glide
            nav_roll_cd = 0;
            nav_pitch_cd = 0;
            channel_throttle->set_servo_out(0);
        }
        if (g.fbwa_tdrag_chan > 0) {
            // check for the user enabling FBWA taildrag takeoff mode
            bool tdrag_mode = (hal.rcin->read(g.fbwa_tdrag_chan-1) > 1700);
            if (tdrag_mode && !auto_state.fbwa_tdrag_takeoff_mode) {
                if (auto_state.highest_airspeed < g.takeoff_tdrag_speed1) {
                    auto_state.fbwa_tdrag_takeoff_mode = true;
                    gcs_send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
                }
            }
        }
        break;
    }

    case FLY_BY_WIRE_B:
        // Thanks to Yury MonZon for the altitude limit code!
        nav_roll_cd = channel_roll->norm_input() * roll_limit_cd;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        update_load_factor();
        update_fbwb_speed_height();
        break;
        
    case CRUISE:
        /*
          in CRUISE mode we use the navigation code to control
          roll when heading is locked. Heading becomes unlocked on
          any aileron or rudder input
        */
        if ((channel_roll->get_control_in() != 0 ||
             rudder_input != 0)) {                
            cruise_state.locked_heading = false;
            cruise_state.lock_timer_ms = 0;
        }                 
        
        if (!cruise_state.locked_heading) {
            nav_roll_cd = channel_roll->norm_input() * roll_limit_cd;
            nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
            update_load_factor();
        } else {
            calc_nav_roll();
        }
        update_fbwb_speed_height();
        ////////////////////////Temporary!!!!!
///////////////////////////////////////////////////////////////////
        jtnow= AP_HAL::millis();
        jdt = jtnow - jlast_t;  // [ms]
        if (jlast_t == 0 || jdt > 1000) {
        jdt = 0;
        }
        jlast_t     = jtnow;  
        jdelta_time = (float)jdt * 0.001f; // [s]
        
        Ju_Sensor_MEAS();  // 传感器估计
        Ju_Joystick_CMD(); // 各操纵杆对应的下沉率、滚转角、偏航角速度、速度指令
        Ju_HdotV_Ctrl();   // 纵向控制器 ,输出de[rad] dthr[%]
        Ju_Phi_Ctrl();     // 横航向控制器，输出da[rad],dr[rad]

        if (jinit_counter <= (g.JU_Init_Transtime*1000)) 
        {
            jinit_counter += jdt;   
        }
////////////////////////////////////////////////////////////////////////
/////////////////////////////////

        break;
        
    case STABILIZE:
        nav_roll_cd        = 0;
        nav_pitch_cd       = 0;
        // throttle is passthrough
        break;
        
    case CIRCLE:
        // we have no GPS installed and have lost radio contact
        // or we just want to fly around in a gentle circle w/o GPS,
        // holding altitude at the altitude we set when we
        // switched into the mode
        nav_roll_cd  = roll_limit_cd / 3;
        update_load_factor();
        calc_nav_pitch();
        calc_throttle();
        break;

    case MANUAL:
        // servo_out is for Sim control only
        // ---------------------------------
        channel_roll->set_servo_out(channel_roll->pwm_to_angle());
        channel_pitch->set_servo_out(channel_pitch->pwm_to_angle());
        steering_control.steering = steering_control.rudder = channel_rudder->pwm_to_angle();
        break;
        //roll: -13788.000,  pitch: -13698.000,   thr: 0.000, rud: -13742.000


    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL: {
        // set nav_roll and nav_pitch using sticks
        int16_t roll_limit = MIN(roll_limit_cd, quadplane.aparm.angle_max);
        nav_roll_cd  = channel_roll->norm_input() * roll_limit;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
        float pitch_input = channel_pitch->norm_input();
        if (pitch_input > 0) {
            nav_pitch_cd = pitch_input * MIN(aparm.pitch_limit_max_cd, quadplane.aparm.angle_max);
        } else {
            nav_pitch_cd = pitch_input * MIN(-pitch_limit_min_cd, quadplane.aparm.angle_max);
        }
        nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
        break;
    }
    case JUHdotVPhi: {   
        // 计时器，用于积分、淡化等
        jtnow= AP_HAL::millis();
        jdt = jtnow - jlast_t;  // [ms]
        if (jlast_t == 0 || jdt > 1000) {
        jdt = 0;
        }
        jlast_t     = jtnow;  
        jdelta_time = (float)jdt * 0.001f; // [s]

        Ju_Sensor_MEAS();  // 传感器估计
        Ju_Joystick_CMD(); // 各操纵杆对应的下沉率、滚转角、偏航角速度、速度指令
        Ju_HdotV_Ctrl();   // 纵向控制器 ,输出de[rad] dthr[%]
        Ju_Phi_Ctrl();     // 横航向控制器，输出da[rad],dr[rad]

        if (jinit_counter <= (g.JU_Init_Transtime*1000)) 
        {
            jinit_counter += jdt;   
        }

        break;

    }


    case INITIALISING:
        // handled elsewhere
        break;
    } 
}

void Plane::update_navigation()
{
    // wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
    // ------------------------------------------------------------------------

    uint16_t radius = 0;
    
    switch(control_mode) {
    case AUTO:
        if (home_is_set != HOME_UNSET) {
            mission.update();
        }
        break;
            
    case RTL:
        if (quadplane.available() && quadplane.rtl_mode == 1 &&
            nav_controller->reached_loiter_target()) {
            set_mode(QRTL, MODE_REASON_UNKNOWN);
            break;
        } else if (g.rtl_autoland == 1 &&
            !auto_state.checked_for_autoland &&
            reached_loiter_target() && 
            labs(altitude_error_cm) < 1000) {
            // we've reached the RTL point, see if we have a landing sequence
            jump_to_landing_sequence();

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            auto_state.checked_for_autoland = true;
        }
        else if (g.rtl_autoland == 2 &&
            !auto_state.checked_for_autoland) {
            // Go directly to the landing sequence
            jump_to_landing_sequence();

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            auto_state.checked_for_autoland = true;
        }
        radius = abs(g.rtl_radius);
        if (radius > 0) {
            loiter.direction = (g.rtl_radius < 0) ? -1 : 1;
        }
        // fall through to LOITER

    case LOITER:
    case AVOID_ADSB:
    case GUIDED:
        update_loiter(radius);
        break;

    case CRUISE:
        update_cruise();
        break;

    case MANUAL:
    case STABILIZE:
    case TRAINING:
    case INITIALISING:
    case ACRO:
    case FLY_BY_WIRE_A:
    case AUTOTUNE:
    case FLY_BY_WIRE_B:
    case CIRCLE:
    case QSTABILIZE:
    case QHOVER:
    case QLOITER:
    case QLAND:
    case QRTL:
        // nothing to do
    case JUHdotVPhi:
        break;
    }
}

/*
  set the flight stage
 */
void Plane::set_flight_stage(AP_SpdHgtControl::FlightStage fs) 
{
    if (fs == flight_stage) {
        return;
    }

    switch (fs) {
    case AP_SpdHgtControl::FLIGHT_LAND_APPROACH:
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Landing approach start at %.1fm", (double)relative_altitude());
        auto_state.land_in_progress = true;
#if GEOFENCE_ENABLED == ENABLED 
        if (g.fence_autoenable == 1) {
            if (! geofence_set_enabled(false, AUTO_TOGGLED)) {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Disable fence failed (autodisable)");
            } else {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Fence disabled (autodisable)");
            }
        } else if (g.fence_autoenable == 2) {
            if (! geofence_set_floor_enabled(false)) {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Disable fence floor failed (autodisable)");
            } else {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Fence floor disabled (auto disable)");
            }
        }
#endif
        break;

    case AP_SpdHgtControl::FLIGHT_LAND_ABORT:
        gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Landing aborted, climbing to %dm", auto_state.takeoff_altitude_rel_cm/100);
        auto_state.land_in_progress = false;
        break;

    case AP_SpdHgtControl::FLIGHT_LAND_PREFLARE:
    case AP_SpdHgtControl::FLIGHT_LAND_FINAL:
        auto_state.land_in_progress = true;
        break;

    case AP_SpdHgtControl::FLIGHT_NORMAL:
    case AP_SpdHgtControl::FLIGHT_VTOL:
    case AP_SpdHgtControl::FLIGHT_TAKEOFF:
        auto_state.land_in_progress = false;
        break;
    }
    

    flight_stage = fs;

    if (should_log(MASK_LOG_MODE)) {
        Log_Write_Status();
    }
}

void Plane::update_alt()
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }

    // calculate the sink rate.
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
    
    geofence_check(true);

    update_flight_stage();

    if (auto_throttle_mode && !throttle_suppressed) {        

        float distance_beyond_land_wp = 0;
        if (auto_state.land_in_progress && location_passed_point(current_loc, prev_WP_loc, next_WP_loc)) {
            distance_beyond_land_wp = get_distance(current_loc, next_WP_loc);
        }

        SpdHgt_Controller->update_pitch_throttle(relative_target_altitude_cm(),
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 auto_state.land_in_progress,
                                                 distance_beyond_land_wp,
                                                 get_takeoff_pitch_min_cd(),
                                                 throttle_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor);
    }
}

/*
  recalculate the flight_stage
 */
void Plane::update_flight_stage(void)
{
    // Update the speed & height controller states
    if (auto_throttle_mode && !throttle_suppressed) {        
        if (control_mode==AUTO) {
            if (quadplane.in_vtol_auto()) {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_VTOL);
            } else if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_TAKEOFF);
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {

                if ((g.land_abort_throttle_enable && channel_throttle->get_control_in() >= 90) ||
                        auto_state.commanded_go_around ||
                        flight_stage == AP_SpdHgtControl::FLIGHT_LAND_ABORT){
                    // abort mode is sticky, it must complete while executing NAV_LAND
                    set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_ABORT);
                } else if (auto_state.land_complete == true) {
                    set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_FINAL);
                } else if (auto_state.land_pre_flare == true) {
                    set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_PREFLARE);
                } else if (flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
                    bool heading_lined_up = abs(nav_controller->bearing_error_cd()) < 1000 && !nav_controller->data_is_stale();
                    bool on_flight_line = abs(nav_controller->crosstrack_error() < 5) && !nav_controller->data_is_stale();
                    bool below_prev_WP = current_loc.alt < prev_WP_loc.alt;
                    if ((mission.get_prev_nav_cmd_id() == MAV_CMD_NAV_LOITER_TO_ALT) ||
                        (auto_state.wp_proportion >= 0 && heading_lined_up && on_flight_line) ||
                        (auto_state.wp_proportion > 0.15f && heading_lined_up && below_prev_WP) ||
                        (auto_state.wp_proportion > 0.5f)) {
                        set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_APPROACH);
                    } else {
                        set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);                        
                    }
                }
            } else if (quadplane.in_assisted_flight()) {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_VTOL);
            } else {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);
            }
        } else {
            // If not in AUTO then assume normal operation for normal TECS operation.
            // This prevents TECS from being stuck in the wrong stage if you switch from
            // AUTO to, say, FBWB during a landing, an aborted landing or takeoff.
            set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);
        }
    } else if (quadplane.in_vtol_mode() ||
               quadplane.in_assisted_flight()) {
        set_flight_stage(AP_SpdHgtControl::FLIGHT_VTOL);
    } else {
        set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);
    }

    // tell AHRS the airspeed to true airspeed ratio
    airspeed.set_EAS2TAS(barometer.get_EAS2TAS());
}




#if OPTFLOW == ENABLED
// called at 50hz
void Plane::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update);
        Log_Write_Optflow();
    }
}
#endif

void Plane::Ju_Joystick_CMD()
{
    // 各操纵杆对应的下沉率、滚转角、偏航角速度、速度指令
         
    // Hdotc [m/s]
    float channel_pitch_norm_input = channel_pitch->norm_input();   // [-1 1]
    if (g.JU_Rev_Gain_Hdotc==-1) {
    channel_pitch_norm_input = -channel_pitch->norm_input();
    }
    if (channel_pitch_norm_input>=0) {
        Ju_Joystick_Hdotc = channel_pitch_norm_input * g.JU_Lim_Hdot_Max;   
    } else {
        Ju_Joystick_Hdotc = channel_pitch_norm_input * ( - g.JU_Lim_Hdot_Min);   
    }
    Ju_Joystick_Hdotc = constrain_float(Ju_Joystick_Hdotc,g.JU_Lim_Hdot_Min,g.JU_Lim_Hdot_Max);

         
    // Vc [m/s]
    float channel_throttle_norm_input = channel_throttle->get_control_in()/100.0f; // [0 1]
    if (g.JU_Rev_Gain_Vc==-1) {
        channel_throttle_norm_input = 1 - channel_throttle->get_control_in()/100.0f;
    }
    Ju_Joystick_Vc    = g.JU_Lim_V_Air_Min + (g.JU_Lim_V_Air_Max - g.JU_Lim_V_Air_Min) * channel_throttle_norm_input;
    Ju_Joystick_Vc    = constrain_float(Ju_Joystick_Vc,g.JU_Lim_V_Air_Min,g.JU_Lim_V_Air_Max);

    // Phic [rad]
    float channel_roll_norm_input = channel_roll->norm_input(); // [-1 1]
    if (g.JU_Rev_Gain_Phic==-1) {
        channel_roll_norm_input = - channel_roll->norm_input();
    }
    Ju_Joystick_Phic  = channel_roll_norm_input * g.JU_Lim_Phi_Max/57.3f;   
    Ju_Joystick_Phic  = constrain_float(Ju_Joystick_Phic, - g.JU_Lim_Phi_Max/57.3f,g.JU_Lim_Phi_Max/57.3f);
         
    // rc [rad/s]
    float channel_rudder_norm_input = channel_rudder->norm_input(); // [-1 1]
    if (g.JU_Rev_Gain_rc==-1) {
        channel_rudder_norm_input = - channel_rudder->norm_input();
    }        
    Ju_Joystick_rc    = channel_rudder_norm_input * g.JU_Lim_r_Air_Max/57.3f;
    Ju_Joystick_rc    = constrain_float(Ju_Joystick_rc, - g.JU_Lim_r_Air_Max/57.3f,g.JU_Lim_r_Air_Max/57.3f);
}

void Plane::Ju_Sensor_MEAS()
{
        // 下沉率估计 从自带的TECS控制处摘过来的代码
        Vector3f vel;
        if(ahrs.get_velocity_NED(vel)) {
        Ju_Hdot_MEAS = vel.z;
        } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        Ju_Hdot_MEAS = gps.velocity().z;
        } else 
        {
            /*
            use a complimentary filter to calculate climb_rate. This is
            designed to minimise lag
            */
            float baro_alt = ahrs.get_baro().get_altitude();
            // Get height acceleration
            float hgt_ddot_mea = -(ahrs.get_accel_ef().z + GRAVITY_MSS);
            // Perform filter calculation using backwards Euler integration
            // Coefficients selected to place all three filter poles at omega
            float _hgtCompFiltOmega = 3;
            float omega2 = _hgtCompFiltOmega*_hgtCompFiltOmega;
            float hgt_err = baro_alt - _height_filter_height;
            float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;
            _height_filter_dd_height += integ1_input * jdelta_time ;
            float integ2_input = _height_filter_dd_height + hgt_ddot_mea + hgt_err * omega2 * 3.0f;
            _jclimb_rate_temp += integ2_input * jdelta_time ;
            Ju_Hdot_MEAS = - _jclimb_rate_temp;
            float integ3_input = _jclimb_rate_temp + hgt_err * _hgtCompFiltOmega * 3.0f;
            // If more than 1 second has elapsed since last update then reset the integrator state
            // to the measured height
            if (jdelta_time > 1.0f) 
            {
            _height_filter_height = height_from_home;
            } 
            else 
            {
            _height_filter_height += integ3_input*jdelta_time;
            }
        }
        
        // 速度估计
        float EAS2TAS = 1; //暂时还是用当量空速EAS好了。。。毕竟地面站/数据记录等都还是记录的当量空速EAS，当然在这个里面控TAS也不影响，因为指令与实际值都乘了EAS2TAS。
        if (!ahrs.airspeed_sensor_enabled() || !ahrs.airspeed_estimate(&jEAS)) {
        // If no airspeed available use average of min and max
        jEAS = 0.5f * (aparm.airspeed_min.get() + (float)aparm.airspeed_max.get());
        }
        Ju_V_A_MEAS = jEAS * EAS2TAS;
        if (g.JU_VAR_V_Smooth == 1) {
        Ju_V_A_MEAS = smoothed_airspeed;    
        }
        
        // 姿态角估计
        Ju_Phi_MEAS   = ahrs.roll_sensor/100.0f/57.3f;  // ahrs.roll_sensor: centidegree   Ju_Phi_MEAS：[rad]
        Ju_Theta_MEAS = ahrs.pitch_sensor/100.0f/57.3f; // ahrs.pitch_sensor: centidegree  Ju_Theta_MEAS：[rad]       
        Ju_Psi_MEAS   = ahrs.yaw_sensor/100.0f/57.3f;   // ahrs.yaw_sensor: centidegree    Ju_Psi_MEAS：[rad]

        // 角速度估计
        Ju_p_MEAS     = ahrs.get_gyro().x; // [rad/s]
        Ju_q_MEAS     = ahrs.get_gyro().y; // [rad/s]
        Ju_r_MEAS     = ahrs.get_gyro().z; // [rad/s]


        // 有些地方将用到除V、Phi的操作，这种地方需要进行防除零保护，另外，插值等也需要限制范围
        Ju_V_Use      = constrain_float(Ju_V_A_MEAS , g.JU_Lim_V_Avd0_Min   , g.JU_Lim_V_Avd0_Max); 
        Ju_Phi_Use    = Ju_Phi_Use_With_Deadzone();
        Ju_Theta_Use  = constrain_float(Ju_Theta_MEAS ,-g.JU_Lim_Theta_Max/57.3f, g.JU_Lim_Theta_Max/57.3f); // [rad]
}

void Plane::Ju_HdotV_Ctrl()
{
    Ju_Ref_Hdot_Mdl();
    Ju_Ref_V_Mdl();

    // Hdot Control

    float delta_Hdotc  = Ju_Ref_Hdot - Ju_Hdot_MEAS;
    float delta_Gammac = delta_Hdotc / Ju_V_Use;
    Ju_Thetac          = delta_Gammac + Ju_Theta_MEAS; // delta_Theta 约等于 delta_Gamma
    Ju_Thetac          = constrain_float(Ju_Thetac , - g.JU_Lim_Theta_Max/57.3f , g.JU_Lim_Theta_Max/57.3f);
    delta_Gammac       = Ju_Thetac - Ju_Theta_MEAS;
    float gammadotc    = delta_Gammac * g.JU_Gain_P_Pgamma;
    float delta_loadfactor = gammadotc * Ju_V_Use / g_acc / cosf(Ju_Phi_Use);
    delta_loadfactor   = constrain_float(delta_loadfactor , - g.JU_Lim_Delta_nz_Max , g.JU_Lim_Delta_nz_Max);
    Ju_qc_FB           = delta_loadfactor * g_acc / Ju_V_Use;
    Ju_qc_RollComp     = Ju_Get_q_Rollcomp();
    Ju_qc              = Ju_qc_FB + Ju_qc_RollComp + Ju_Ref_q;
    Ju_qc              = constrain_float(Ju_qc , - g.JU_Lim_q_Max/57.3f , g.JU_Lim_q_Max/57.3f);
    Ju_dec_FB          = Ju_q_Ctrl();
    Ju_dec_Trim        = Ju_de_Trim();
    Ju_dec             = Ju_dec_FB + Ju_dec_Trim + Ju_Ref_de;
    Ju_dec             = constrain_float(Ju_dec , - g.JU_DEF_de_Max/57.3f , g.JU_DEF_de_Max/57.3f); // [rad] 

    // V Control
    float delta_Vc     = Ju_Ref_V - Ju_V_A_MEAS;
    Ju_V_P             = delta_Vc * g.JU_Gain_P_PV;
    if (jinit_counter == 0) {
        Ju_V_I = 0;
    }
    else 
    {
        if (jdt>0) 
        {
            Ju_V_I = Ju_V_I + delta_Vc * g.JU_Gain_P_IV * jdelta_time;
            Ju_V_I = constrain_float(Ju_V_I , - g.JU_Lim_V_I_Max , g.JU_Lim_V_I_Max);
        }
        else
        {
            Ju_V_I = 0;
        }
    }
    Ju_Hdot2Vdot = Ju_Hdot2Vdot_LeadFilter();
    Ju_Vdotc     = Ju_V_P + Ju_V_I + Ju_Ref_Vdot + Ju_Hdot2Vdot;
    Ju_Thrc_FB   = Ju_Vdotc * g.JU_Gain_P_ThrPerVdot;//[%]
    Ju_Thrc_Trim = linear_interpolate(g.JU_Trim_dthr_Low , g.JU_Trim_dthr_High, Ju_V_A_MEAS, g.JU_Trim_V_Low , g.JU_Trim_V_High);
    Ju_Thrc      = Ju_Thrc_FB + Ju_Thrc_Trim;
    Ju_Thrc      = constrain_float(Ju_Thrc, g.JU_Lim_Thr_Min , g.JU_Lim_Thr_Max); // [%] 0-100
}

void Plane::Ju_Phi_Ctrl()
{
    Ju_Ref_Phi_Mdl();

    // Phi Ctrl
    float delta_Phic   = Ju_Ref_Phi - Ju_Phi_MEAS;
    Ju_Phidotc_FB      = delta_Phic * g.JU_Gain_RY_Pphi;
    Ju_Phidotc         = Ju_Phidotc_FB + Ju_Ref_Phidot;
    Ju_pc              = Ju_Calc_Phidot2p();
    Ju_dac_FB          = Ju_p_Ctrl();
    Ju_dac             = Ju_dac_FB + Ju_Ref_da;
    Ju_dac             = constrain_float(Ju_dac , - g.JU_DEF_da_Max/57.3f , g.JU_DEF_da_Max/57.3f); // [rad] 

    // r Ctrl
    Ju_rc_Coordinate   = Ju_Calc_r_Coordinate();
    float delta_rc     = Ju_rc_Coordinate - Ju_r_MEAS;
    float delta_rcWash = Ju_Calc_rWashFilter(delta_rc); 
    Ju_drc             = - (delta_rcWash + Ju_Joystick_rc) * g.JU_Gain_RY_Pr;
    Ju_drc             = Ju_drc + Ju_dac * g.JU_Gain_RY_ARI;
    Ju_drc             = constrain_float(Ju_drc , - g.JU_DEF_dr_Max/57.3f , g.JU_DEF_dr_Max/57.3f); // [rad] )
}

void Plane::Ju_Ref_Hdot_Mdl() 
{   
    //  计算 Ju_Ref_Hdot Ju_Ref_q Ju_Ref_de 
    if (jinit_counter == 0) 
    {
        Ju_Ref_Hdot    = Ju_Hdot_MEAS;
        Ju_Ref_Hdotdot = 0; // 这个加速度就不跟着一块初始为当前时刻的加速度了，因为当前时刻的加速度计示数可能不太准
        Ju_Ref_q       = 0;
        Ju_Ref_de      = 0;
    }
    else
    {   
        float hdotdot_ref_temp = (Ju_Joystick_Hdotc - Ju_Ref_Hdot) / g.JU_Ref_T_Hdot / cosf(Ju_Phi_Use);
        hdotdot_ref_temp     = constrain_float(hdotdot_ref_temp , - g.JU_Lim_Delta_nz_Max * g_acc , g.JU_Lim_Delta_nz_Max * g_acc);
        Ju_Ref_Hdotdotdot    = (hdotdot_ref_temp - Ju_Ref_Hdotdot) / g.JU_Ref_T_Hdotdot;
        Ju_Ref_de            = Ju_Ref_Hdotdotdot * g.JU_Gain_Ref_FF_de / ( Ju_V_Use * Ju_V_Use * Ju_V_Use ); // [rad] 
        Ju_Ref_Hdotdot       = Ju_Ref_Hdotdot + Ju_Ref_Hdotdotdot * jdelta_time;
        Ju_Ref_Hdotdot       = constrain_float(Ju_Ref_Hdotdot   , - g.JU_Lim_Delta_nz_Max * g_acc , g.JU_Lim_Delta_nz_Max * g_acc);
        Ju_Ref_q             = Ju_Ref_Hdotdot / Ju_V_Use * g.JU_Gain_Ref_FF_q;
        Ju_Ref_Hdotdot       = Ju_Ref_Hdotdot * cosf(Ju_Phi_Use);
        Ju_Ref_Hdot          = Ju_Ref_Hdot + Ju_Ref_Hdotdot * jdelta_time;
        Ju_Ref_Hdot          = constrain_float(Ju_Ref_Hdot , - g.JU_Lim_Hdot_Max , g.JU_Lim_Hdot_Max);
    }
}

void Plane::Ju_Ref_V_Mdl()
{
    //  计算 Ju_Ref_V Ju_Ref_Vdot
    if (jinit_counter == 0) {
        Ju_Ref_V      = Ju_V_A_MEAS;
        Ju_Ref_Vdot   = 0;
    }
    else {
        Ju_Ref_Vdot   = (Ju_Joystick_Vc - Ju_Ref_V) / g.JU_Ref_T_V * g.JU_Gain_Ref_FF_Vdot;
        Ju_Ref_Vdot   = constrain_float(Ju_Ref_Vdot , - g.JU_Lim_Vdot_Max , g.JU_Lim_Vdot_Max);
        Ju_Ref_V      = Ju_Ref_V + Ju_Ref_Vdot * jdelta_time;
        Ju_Ref_V      = constrain_float(Ju_Ref_V , -g.JU_Lim_V_Air_Max,g.JU_Lim_V_Air_Max); //注意，此处其实并不对最小值作约束，因为切换的时候可能低于空中模式所设定的最小值
    }

}

void Plane::Ju_Ref_Phi_Mdl()
{   //  计算 Ju_Ref_V Ju_Ref_Vdot
    if (jinit_counter == 0) {
        Ju_Ref_Phi    = Ju_Phi_MEAS;
        Ju_Ref_Phidot = 0;
        Ju_Ref_da     = 0;
    }
    else {
        float w0square = g.JU_Ref_w0_Phi * g.JU_Ref_w0_Phi;
        Ju_Ref_Phidotdot = (Ju_Joystick_Phic - Ju_Ref_Phi) * w0square - 2 * g.JU_Ref_w0_Phi * g.JU_Ref_Ksi_Phi * Ju_Ref_Phidot;
        Ju_Ref_Phidotdot = constrain_float(Ju_Ref_Phidotdot , - g.JU_Lim_Phidotdot_Max/57.3f, g.JU_Lim_Phidotdot_Max/57.3f);
        Ju_Ref_da        = Ju_Ref_Phidotdot  * g.JU_Gain_Ref_FF_da / ( Ju_V_Use * Ju_V_Use ); // [rad] 注意，这里是V平方
        Ju_Ref_Phidot    = Ju_Ref_Phidot + Ju_Ref_Phidotdot * jdelta_time * g.JU_Gain_Ref_FF_Phidot;
        Ju_Ref_Phidot    = constrain_float(Ju_Ref_Phidot , - g.JU_Lim_Phidot_Max/57.3f, g.JU_Lim_Phidot_Max/57.3f);
        Ju_Ref_Phi       = Ju_Ref_Phi + Ju_Ref_Phidot * jdelta_time;
        Ju_Ref_Phi       = constrain_float(Ju_Ref_Phi , - g.JU_Lim_Phi_Max/57.3f, g.JU_Lim_Phi_Max/57.3f);
    }
}

float Plane::Ju_Get_q_Rollcomp(void)
{  // 计算滚转时的俯仰角速度补偿量 [rad/s]
    float q_roll_comp    =  g_acc / Ju_V_Use * tanf(Ju_Phi_Use) * sinf(Ju_Phi_Use) * cosf(Ju_Theta_Use);
    return q_roll_comp;
}

float Plane::Ju_q_Ctrl(void)
{   // q控制器， 计算升降舵偏量,下偏为正 [rad]
    Ju_de_F =   Ju_qc     * g.JU_Gain_P_Fq;
    Ju_de_P = - Ju_q_MEAS * g.JU_Gain_P_Pq;
    if (jinit_counter == 0) {
        Ju_de_I = 0;
    }
    else 
    {
        if (jdt>0) 
        {
            Ju_de_I = Ju_de_I + (Ju_qc - Ju_q_MEAS) * g.JU_Gain_P_Iq * jdelta_time;
            Ju_de_I = constrain_float(Ju_de_I , - g.JU_Lim_q_I_Max/57.3f , g.JU_Lim_q_I_Max/57.3f);
        }
        else
        {
            Ju_de_I = 0;
        }
    }
    float  dec = - (Ju_de_P + Ju_de_I + Ju_de_F);  
    return dec;
}

float Plane::Ju_p_Ctrl(void)
{   // p控制器， 计算副翼偏量,产生左滚力矩为正 [rad]
    Ju_da_F =   Ju_pc     * g.JU_Gain_RY_Fp;
    Ju_da_P = - Ju_p_MEAS * g.JU_Gain_RY_Pp;
    if (jinit_counter == 0) {
        Ju_da_I = 0;
    }
    else 
    {
        if (jdt>0) 
        {
            Ju_da_I = Ju_da_I + (Ju_pc - Ju_p_MEAS) * g.JU_Gain_RY_Ip * jdelta_time;
            Ju_da_I = constrain_float(Ju_da_I , - g.JU_Lim_p_I_Max/57.3f , g.JU_Lim_p_I_Max/57.3f);
        }
        else
        {
            Ju_da_I = 0;
        }
    }
    float  dac = - (Ju_da_P + Ju_da_I + Ju_da_F);  
    return dac;
}

float Plane::Ju_de_Trim(void)
{
    float  de_trimdeg = linear_interpolate(g.JU_Trim_de_Low , g.JU_Trim_de_High, Ju_V_A_MEAS, g.JU_Trim_V_Low , g.JU_Trim_V_High);
    float  de_trim    = de_trimdeg/57.3f;
    return de_trim;
}

float Plane::Ju_Hdot2Vdot_LeadFilter(void)
{
    float Vdotc = Ju_Ref_Hdot * g_acc / Ju_V_Use;
    float Vdotc_Lead = Vdotc;

    if (g.JU_Gain_P_LeadTzVdot > 0.01)  // 进行超前的情况
    {
        if (jinit_counter == 0)
        {
            Ju_Hdot2Vdotc_Last = 0;
            Ju_Hdot2Vdotc_Lead_Last = 0;     
        }
        float wn   = 1 / g.JU_Gain_P_LeadTzVdot;
        float wd   = 1 / g.JU_Gain_P_LeadTpVdot;
        float Ts   = jdelta_time;
        float c1   = (Ts*wn*wd+2*wd)/(Ts*wn*wd+2*wn);
        float c2   = (Ts*wn*wd-2*wd)/(Ts*wn*wd+2*wn);
        float c3   = (2-Ts*wd)/(2+Ts*wd);
        Vdotc_Lead = c1 * Vdotc + c2 * Ju_Hdot2Vdotc_Last + c3 * Ju_Hdot2Vdotc_Lead_Last;
        Ju_Hdot2Vdotc_Last = Vdotc;
        Ju_Hdot2Vdotc_Lead_Last = Vdotc_Lead;
    }
    Vdotc_Lead = constrain_float(Vdotc_Lead , - g.JU_Lim_Hdot2Vdot_Max , g.JU_Lim_Hdot2Vdot_Max);
    return Vdotc_Lead;
}

float Plane::Ju_Calc_rWashFilter(float rc)
{
    float  rcWash = rc;
    if (g.JU_Gain_RY_rWashTau>=0.1)
    {
        if (jinit_counter == 0)
        {
            rclast = 0;
            rcwashlast = 0;     
        }
        float w    = 1 / g.JU_Gain_RY_rWashTau;
        float Ts   = jdelta_time;
        float c1   = 2 / (2 + w * Ts);
        float c2   = (2 - w * Ts)/(2 + w * Ts);
        rcWash     = c1 * (rc - rclast) + c2 * rcwashlast;
        rclast     = rc;
        rcwashlast = rcWash;
    }
    return rcWash;
}

float Plane::Ju_Calc_Phidot2p(void)
{
    float pc = Ju_Phidotc;
    if (g.JU_VAR_Phidot2p==1) {
        pc   = Ju_Phidotc - g_acc / Ju_V_Use * sinf(Ju_Theta_Use) * tanf(Ju_Phi_Use); 
    }
    pc = constrain_float(pc , - g.JU_Lim_Phidot_Max/57.3f , g.JU_Lim_Phidot_Max/57.3f);
    return pc;
}

float Plane::Ju_Phi_Use_With_Deadzone(void)
{
    float Phi_Use = Ju_Phi_MEAS;
    if (g.JU_DeadZone_Phi_Use>0.5)
    {
        if (fabsf(Ju_Phi_MEAS)<=(g.JU_DeadZone_Phi_Use/57.3f/2.0f)) {
            Phi_Use = 0;
        }
        else
        {
            if(fabsf(Ju_Phi_MEAS)<=(g.JU_DeadZone_Phi_Use/57.3f))
            {
                if (Ju_Phi_MEAS>0)
                {
                    Phi_Use = (Ju_Phi_MEAS - g.JU_DeadZone_Phi_Use/57.3f/2.0f) * 2;
                }
                else 
                {
                    Phi_Use = (Ju_Phi_MEAS + g.JU_DeadZone_Phi_Use/57.3f/2.0f) * 2;
                }
            }
            else
            {
                Phi_Use = Ju_Phi_MEAS;
            }
        }

    }
    Phi_Use    = constrain_float(Phi_Use,-g.JU_Lim_Phi_Max/57.3f,g.JU_Lim_Phi_Max/57.3f); // [rad]
    return Phi_Use;
}

float Plane::Ju_Calc_r_Coordinate(void)
{
    float  rc = sinf(Ju_Phi_Use) * cosf(Ju_Theta_Use) * g_acc / Ju_V_Use;
    return rc;
}


AP_HAL_MAIN_CALLBACKS(&plane);
