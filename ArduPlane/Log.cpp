// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#include "version.h"

#if LOGGING_ENABLED == ENABLED

#if CLI_ENABLED == ENABLED
// Code to Write and Read packets from DataFlash.log memory
// Code to interact with the user to dump or erase logs

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] = {
    {"dump",        MENU_FUNC(dump_log)},
    {"erase",       MENU_FUNC(erase_logs)},
    {"enable",      MENU_FUNC(select_logs)},
    {"disable",     MENU_FUNC(select_logs)}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, FUNCTOR_BIND(&plane, &Plane::print_log_menu, bool));

bool Plane::print_log_menu(void)
{
    cliSerial->println("logs enabled: ");

    if (0 == g.log_bitmask) {
        cliSerial->println("none");
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
 #define PLOG(_s) if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf(" %s", # _s)
        PLOG(ATTITUDE_FAST);
        PLOG(ATTITUDE_MED);
        PLOG(GPS);
        PLOG(PM);
        PLOG(CTUN);
        PLOG(JTH);
        PLOG(JTV);
        PLOG(JTR);
        PLOG(JTY);
        PLOG(NTUN);
        PLOG(MODE);
        PLOG(IMU);
        PLOG(CMD);
        PLOG(CURRENT);
        PLOG(COMPASS);
        PLOG(TECS);
        PLOG(CAMERA);
        PLOG(RC);
        PLOG(SONAR);
 #undef PLOG
    }

    cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);
    return(true);
}

int8_t Plane::dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log_num;
    uint16_t dump_log_start;
    uint16_t dump_log_end;

    // check that the requested log number can be read
    dump_log_num = argv[1].i;

    if (dump_log_num == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log_num <= 0) {
        cliSerial->printf("dumping all\n");
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2) || ((uint16_t)dump_log_num > DataFlash.get_num_logs())) {
        cliSerial->printf("bad log number\n");
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log_num, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log_num, dump_log_start, dump_log_end);
    return 0;
}

int8_t Plane::erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

int8_t Plane::select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint32_t bits;

    if (argc != 2) {
        cliSerial->printf("missing log type\n");
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp(argv[1].str, "all")) {
        bits = 0xFFFFFFFFUL;
    } else {
 #define TARG(_s)        if (!strcasecmp(argv[1].str, # _s)) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(JTH);
        TARG(JTV);
        TARG(JTR);
        TARG(JTY);
        TARG(NTUN);
        TARG(MODE);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(COMPASS);
        TARG(TECS);
        TARG(CAMERA);
        TARG(RC);
        TARG(SONAR);
 #undef TARG
    }

    if (!strcasecmp(argv[0].str, "enable")) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }
    return(0);
}

int8_t Plane::process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}

#endif // CLI_ENABLED == ENABLED

void Plane::do_erase_logs(void)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Erasing logs");
    DataFlash.EraseAll();
    gcs_send_text(MAV_SEVERITY_INFO, "Log erase complete");
}


// Write an attitude packet
void Plane::Log_Write_Attitude(void)
{
    Vector3f targets;       // Package up the targets into a vector for commonality with Copter usage of Log_Wrote_Attitude
    targets.x = nav_roll_cd;
    targets.y = nav_pitch_cd;
    targets.z = 0;          //Plane does not have the concept of navyaw. This is a placeholder.

    DataFlash.Log_Write_Attitude(ahrs, targets);
    if (quadplane.in_vtol_mode()) {
        DataFlash.Log_Write_PID(LOG_PIDR_MSG, quadplane.attitude_control->get_rate_roll_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDP_MSG, quadplane.attitude_control->get_rate_pitch_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDY_MSG, quadplane.attitude_control->get_rate_yaw_pid().get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDA_MSG, quadplane.pid_accel_z.get_pid_info() );
    } else {
        DataFlash.Log_Write_PID(LOG_PIDR_MSG, rollController.get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDP_MSG, pitchController.get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDY_MSG, yawController.get_pid_info());
        DataFlash.Log_Write_PID(LOG_PIDS_MSG, steerController.get_pid_info());
    }

#if AP_AHRS_NAVEKF_AVAILABLE
 #if OPTFLOW == ENABLED
    DataFlash.Log_Write_EKF(ahrs,optflow.enabled());
 #else
    DataFlash.Log_Write_EKF(ahrs,false);
 #endif
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE(&DataFlash);
#endif
    DataFlash.Log_Write_POS(ahrs);
}

// do logging at loop rate
void Plane::Log_Write_Fast(void)
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }
}


struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t num_long;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
    uint32_t g_dt_min;
    uint32_t log_dropped;
};

// Write a performance monitoring packet. Total length : 19 bytes
void Plane::Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        time_us         : AP_HAL::micros64(),
        num_long        : perf.num_long,
        main_loop_count : perf.mainLoop_count,
        g_dt_max        : perf.G_Dt_max,
        g_dt_min        : perf.G_Dt_min,
        log_dropped     : DataFlash.num_dropped() - perf.last_log_dropped
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t startup_type;
    uint16_t command_total;
};

void Plane::Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        time_us         : AP_HAL::micros64(),
        startup_type    : type,
        command_total   : mission.num_commands()
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    /*// Tune Hdot dataset
    float decJ;
    float de_servo_outJ;
    float de_servo_out;
    float de_pwmJ;
    float de_pwm;
    float de_I;
    float de_Ref;
    float q_Ref;*/

    /*// Tune Phi dataset
    float da_servo_outJ;
    float da_servo_out;
    float da_pwmJ;
    float da_pwm;
    float dr_servo_outJ;
    float dr_servo_out;
    float dr_pwmJ;
    float dr_pwm;
    float dlrc;
    float dlrcW;*/

    // Onboard Flight Dataset
    int16_t dac_servo_out;
    int16_t dec_servo_out;
    int16_t dthrc_servo_out;
    int16_t drc_servo_out;
    float   Ioutp;
    float   Ioutq;
    float   IoutV;
    bool    VAR_SteeringCtrl;

    /*int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    int16_t throttle_dem;*/

};




// Write a control tuning packet. Total length : 22 bytes
void Plane::Log_Write_Control_Tuning()
{
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_us         : AP_HAL::micros64(),
        /*// Tune Hdot dataset
        decJ           : Ju_dec*57.3,
        de_servo_outJ  : Ju_de_servo_out,
        de_servo_out    : channel_pitch->get_servo_out(),
        de_pwmJ        :Ju_de_radio_out,
        de_pwm   : channel_pitch->get_radio_out(),
        de_I : Ju_de_I*57.3f,
        de_Ref: Ju_Ref_de*57.3f,
        q_Ref:Ju_Ref_q*57.3f*/

        /*// Tune Phi dataset
        da_servo_outJ:(float)Ju_da_servo_out,
        da_servo_out:(float)channel_roll->get_servo_out(),
        da_pwmJ:(float)Ju_da_radio_out,
        da_pwm:(float)channel_roll->get_radio_out(),
        dr_servo_outJ:(float)Ju_dr_servo_out,
        dr_servo_out:(float)channel_rudder->get_servo_out(),
        dr_pwmJ:(float)Ju_dr_radio_out,
        dr_pwm:(float)channel_rudder->get_radio_out(),
        dlrc:Ju_delta_rc*57.3f,
        dlrcW:Ju_delta_rcWash*57.3f,*/

        // Onboard Flight Dataset
        dac_servo_out       : Ju_da_servo_out,
        dec_servo_out       : Ju_de_servo_out,
        dthrc_servo_out     : Ju_dthr_servo_out,
        drc_servo_out       : Ju_dr_servo_out,
        Ioutp               : Ju_da_I,
        Ioutq               : Ju_de_I,
        IoutV               : Ju_V_I,
        VAR_SteeringCtrl    : steering_control.ground_steering

        /*nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)channel_throttle->get_servo_out(),
        rudder_out      : (int16_t)channel_rudder->get_servo_out(),
        throttle_dem    : (int16_t)SpdHgt_Controller->get_throttle_demand()*/
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Ju_Tuning_Hdot {
    LOG_PACKET_HEADER; 
    uint64_t time_us; 
    float Thetac;
    float qc_FB;
    float qc_Rollcomp;
    float qc_Ref;
    float qPout;
    float qIout;
    float qFout;
    float dec_FB;
    float dec_Trim;
    float dec_Ref;
    float dec;  
};

void Plane::Log_Write_Ju_Tuning_Hdot()
{
    struct log_Ju_Tuning_Hdot pkt = {
        LOG_PACKET_HEADER_INIT(LOG_JTH_MSG),
        time_us     : AP_HAL::micros64(),
        Thetac      : Ju_Thetac * 57.3f,
        qc_FB       : Ju_qc_FB * 57.3f,
        qc_Rollcomp : Ju_qc_RollComp * 57.3f,
        qc_Ref      : Ju_Ref_q * 57.3f,
        qPout       : -Ju_de_P * 57.3f,
        qIout       : -Ju_de_I * 57.3f,
        qFout       : -Ju_de_F * 57.3f,
        dec_FB      : Ju_dec_FB * 57.3f,
        dec_Trim    : Ju_dec_Trim * 57.3f,
        dec_Ref     : Ju_Ref_de * 57.3f,
        dec         : Ju_dec * 57.3f
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Ju_Tuning_V {
    LOG_PACKET_HEADER; 
    uint64_t time_us;
    float VPout; 
    float VIout;
    float Hdot2Vdotc0; // 经过超前滤波器前
    float Hdot2VdotcLead; // 经超前处理之后
    float VdotcRef;
    float dthrc_FB;
    float dthrc_Trim;
    float dthrc;
 
};

void Plane::Log_Write_Ju_Tuning_V()
{
    struct log_Ju_Tuning_V pkt = {
        LOG_PACKET_HEADER_INIT(LOG_JTV_MSG),
        time_us     : AP_HAL::micros64(),
        VPout       : Ju_V_P,
        VIout       : Ju_V_I,
        Hdot2Vdotc0 : Ju_Hdot2Vdot0,
        Hdot2VdotcLead : Ju_Hdot2Vdot,
        VdotcRef    : Ju_Ref_Vdot,
        dthrc_FB    : Ju_Thrc_FB,
        dthrc_Trim  : Ju_Thrc_Trim,
        dthrc       : Ju_Thrc
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Ju_Tuning_Roll {
    LOG_PACKET_HEADER; 
    uint64_t time_us;
    float Phidotc_FB;
    float PhidotcRef;
    float Phidotc;
    float pc;
    float pPout;
    float pIout;
    float pFout;
    float dac_FB;
    float dacRef;
    float dac;

};

void Plane::Log_Write_Ju_Tuning_Roll()
{
    struct log_Ju_Tuning_Roll pkt = {
        LOG_PACKET_HEADER_INIT(LOG_JTR_MSG),
        time_us     : AP_HAL::micros64(),
        Phidotc_FB  : Ju_Phidotc_FB * 57.3f,
        PhidotcRef  : Ju_Ref_PhidotCtrl * 57.3f,
        Phidotc     : Ju_Phidotc * 57.3f,
        pc          : Ju_pc * 57.3f,
        pPout       : -Ju_da_P * 57.3f,
        pIout       : -Ju_da_I * 57.3f,
        pFout       : -Ju_da_F * 57.3f,
        dac_FB      : Ju_dac_FB * 57.3f,
        dacRef      : Ju_Ref_da * 57.3f,
        dac         : Ju_dac * 57.3f
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Ju_Tuning_Yaw {
    LOG_PACKET_HEADER; 
    uint64_t time_us;
    float rcCoord;
    float rcJoystick; 
    float delta_rc;
    float delta_rcWash;
    float drc_FB;
    float drc_ARI;
    float drc;
};

void Plane::Log_Write_Ju_Tuning_Yaw()
{
    struct log_Ju_Tuning_Yaw pkt = {
        LOG_PACKET_HEADER_INIT(LOG_JTY_MSG),
        time_us     : AP_HAL::micros64(),
        rcCoord     : Ju_rc_Coordinate * 57.3f,
        rcJoystick  : Ju_Joystick_rc * 57.3f,
        delta_rc    : Ju_delta_rc * 57.3f,
        delta_rcWash: Ju_delta_rcWash * 57.3f,
        drc_FB      : Ju_drc_FB * 57.3f,
        drc_ARI     : Ju_drc_ARI * 57.3f,
        drc         : Ju_drc * 57.3f
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}



struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    /* // Tune Hdot Dataset
    float Hdot;
    float Hdotc;
    float HdotR;
    float Theta;
    float Thetac;
    float q;
    float qc;
    float dec;*/

    /*// Tune V Dataset
    float V;
    float Vc;
    float VR;
    float VdotR;
    float Vdotc;
    float T_servoJ;
    float T_servo;
    float T_pwmJ;
    float T_pwm;
    float V_I;
    float V_P;
    float Vd_FromHdot;*/

    /*// Tune Phi Dataset
    float phi;
    float phic;
    float phiref;
    float phidotc;
    float pc;
    float p;
    float daref;
    float rc;
    float rpilot;
    float r;*/
    
    // Onboard Flight Dataset
    float V;
    float Vc_Stick;
    float V_Ref;
    float Hdot;
    float Hdot_Stick;
    float Hdot_Ref;
    float q;
    float qc;
    float Phic_Stick;
    float Phi_Ref;
    float p;
    float pc;
    float r;
    float rc;

    /*float wp_distance;
    int16_t target_bearing_cd;
    int16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    float   xtrack_error;
    float   xtrack_error_i;
    float   airspeed_error;*/
};

// Write a navigation tuning packet
void Plane::Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_us    : AP_HAL::micros64(),
        /* // Tune Hdot Dataset
        Hdot:Ju_Hdot_MEAS,
        Hdotc:Ju_Joystick_Hdotc,
        HdotR:Ju_Ref_Hdot,
        Theta:Ju_Theta_MEAS*57.3f,
        Thetac:Ju_Thetac*57.3f, 
        q:Ju_q_MEAS*57.3f,
        qc:Ju_qc*57.3f,
        dec:Ju_dec*57.3f*/

        /*// Tune V Dataset
        V:Ju_V_A_MEAS,
        Vc:Ju_Joystick_Vc,
        VR:Ju_Ref_V,
        VdotR:Ju_Ref_Vdot,
        Vdotc:Ju_Vdotc,
        T_servoJ: (float)Ju_dthr_servo_out,
        T_servo:channel_throttle->get_servo_out(),
        T_pwmJ:(float)Ju_dthr_radio_out,
        T_pwm:channel_throttle->get_radio_out(),
        V_I:Ju_V_I,
        V_P:Ju_V_P,
        Vd_FromHdot:Ju_Hdot2Vdot*/

        /*// Tune Phi Dataset
        phi: Ju_Phi_MEAS*57.3f,
        phic:Ju_Joystick_Phic*57.3f,
        phiref:Ju_Ref_Phi*57.3f,
        phidotc:Ju_Phidotc*57.3f,
        pc:Ju_pc*57.3f,
        p:Ju_p_MEAS*57.3f,
        daref:Ju_Ref_da*57.3f,
        rc:Ju_rc_Coordinate*57.3f,
        rpilot:Ju_Joystick_rc*57.3f,
        r:Ju_r_MEAS*57.3f*/

        // Onboard Flight Dataset
        V          : Ju_V_A_MEAS,
        Vc_Stick   : Ju_Joystick_Vc,
        V_Ref      : Ju_Ref_V,
        Hdot       : Ju_Hdot_MEAS,
        Hdot_Stick : Ju_Joystick_Hdotc,
        Hdot_Ref   : Ju_Ref_Hdot,
        q          : Ju_q_MEAS*57.3f,
        qc         : Ju_qc*57.3f,
        Phic_Stick : Ju_Joystick_Phic*57.3f,
        Phi_Ref    : Ju_Ref_Phi*57.3f,
        p          : Ju_p_MEAS*57.3f,
        pc         : Ju_pc*57.3f,
        r          : Ju_r_MEAS*57.3f,
        rc         : Ju_rc*57.3f
        
/*        wp_distance         : auto_state.wp_distance,
        target_bearing_cd   : (int16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (int16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)altitude_error_cm,
        xtrack_error        : nav_controller->crosstrack_error(),
        xtrack_error_i      : nav_controller->crosstrack_error_integrator(),
        airspeed_error      : airspeed_error*/
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Status {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t is_flying;
    float is_flying_probability;
    uint8_t armed;
    uint8_t safety;
    bool is_crashed;
    bool is_still;
    uint8_t stage;
    bool impact;
};

void Plane::Log_Write_Status()
{
    struct log_Status pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STATUS_MSG)
        ,time_us   : AP_HAL::micros64()
        ,is_flying   : is_flying()
        ,is_flying_probability : isFlyingProbability
        ,armed       : hal.util->get_soft_armed()
        ,safety      : static_cast<uint8_t>(hal.util->safety_switch_state())
        ,is_crashed  : crash_state.is_crashed
        ,is_still    : plane.ins.is_still()
        ,stage       : static_cast<uint8_t>(flight_stage)
        ,impact      : crash_state.impact_detected
        };

    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Sonar {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float distance;
    float voltage;
    uint8_t count;
    float correction;
};

// Write a sonar packet
void Plane::Log_Write_Sonar()
{
#if RANGEFINDER_ENABLED == ENABLED
    uint16_t distance = 0;
    if (rangefinder.status() == RangeFinder::RangeFinder_Good) {
        distance = rangefinder.distance_cm();
    }

    struct log_Sonar pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SONAR_MSG),
        time_us     : AP_HAL::micros64(),
        distance    : (float)distance*0.01f,
        voltage     : rangefinder.voltage_mv()*0.001f,
        count       : rangefinder_state.in_range_count,
        correction  : rangefinder_state.correction
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    DataFlash.Log_Write_RFND(rangefinder);
#endif
}

struct PACKED log_Optflow {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t surface_quality;
    float flow_x;
    float flow_y;
    float body_x;
    float body_y;
};

#if OPTFLOW == ENABLED
// Write an optical flow packet
void Plane::Log_Write_Optflow()
{
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : optflow.quality(),
        flow_x           : flowRate.x,
        flow_y           : flowRate.y,
        body_x           : bodyRate.x,
        body_y           : bodyRate.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}
#endif

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

void Plane::Log_Write_Current()
{
    DataFlash.Log_Write_Current(battery);

    // also write power status
    DataFlash.Log_Write_Power();
}

void Plane::Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_us                 : AP_HAL::micros64(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()      
    };
    DataFlash.WriteCriticalBlock(&pkt, sizeof(pkt));
}

void Plane::Log_Write_GPS(uint8_t instance)
{
    if (!ahrs.have_ekf_logging()) {
        DataFlash.Log_Write_GPS(gps, instance);
    }
}

void Plane::Log_Write_IMU() 
{
    DataFlash.Log_Write_IMU(ins);
}

void Plane::Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
    if (rssi.enabled()) {
        DataFlash.Log_Write_RSSI(rssi);
    }
}

void Plane::Log_Write_Baro(void)
{
    if (!ahrs.have_ekf_logging()) {
        DataFlash.Log_Write_Baro(barometer);
    }
}

// Write a AIRSPEED packet
void Plane::Log_Write_Airspeed(void)
{
    DataFlash.Log_Write_Airspeed(airspeed);
}

// log ahrs home and EKF origin to dataflash
void Plane::Log_Write_Home_And_Origin()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    // log ekf origin if set
    Location ekf_orig;
    if (ahrs.get_origin(ekf_orig)) {
        DataFlash.Log_Write_Origin(LogOriginType::ekf_origin, ekf_orig);
    }
#endif

    // log ahrs home if set
    if (home_is_set != HOME_UNSET) {
        DataFlash.Log_Write_Origin(LogOriginType::ahrs_home, ahrs.get_home());
    }
}

const struct LogStructure Plane::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "QHHIII",  "TimeUS,NLon,NLoop,MaxT,MinT,LogDrop" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "QBH",         "TimeUS,SType,CTot" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning), 
//////// 字符串允许长度有限，尽量简洁着写，但是不要把TimeUS省略成更短的！    
      //"CTUN", "Qcccchhh",    "TimeUS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,ThrDem" },
      //"CTUN", "Qfffffffffff",    "T,Ptch,Ptchc,r,rc,rMan,dac,dec,thrc,drc,daelc,daerc"},  
      //"CTUN", "Qffffffff",    "TimeUS,deJ,deservoJ,deservo,dePWMJ,dePWM,eI,eR,qR"}, // Tune Hdot Dataset
      //"CTUN","Q","TimeUS"},
      //"CTUN", "Qffffffffff", "TimeUS,asJ,as,apJ,ap,rsJ,rs,rpJ,rp,dlrc,dlrcW"}, // Tune Hdot Dataset
        "CTUN", "QhhhhfffB", "TimeUS,daserv,deserv,dthrserv,drserv,Ip,Iq,IV,SteerCtrl"}, // Onboard Flight Dataset
    { LOG_JTH_MSG, sizeof(log_Ju_Tuning_Hdot),
        "JTH" , "Qfffffffffff","TimeUS,Ptchc,qFB,qRl,qRM,qP,qI,qF,deFB,deTrim,deRM,dec" },
    { LOG_JTV_MSG, sizeof(log_Ju_Tuning_V), 
        "JTV" , "Qffffffff","TimeUS,VP,VI,Hd2Vd0,Hd2VdLead,VdRM,dthrFB,dthrTrim,dthrc" },
    { LOG_JTR_MSG, sizeof(log_Ju_Tuning_Roll), 
        "JTR" , "Qffffffffff","TimeUS,PhidFB,PhidRM,Phidc,pc,pP,pI,pF,daFB,daRM,dac" },
    { LOG_JTY_MSG, sizeof(log_Ju_Tuning_Yaw), 
        "JTY" , "Qfffffff","TimeUS,rCoord,rStick,deltarc,deltarcWash,drFB,drARI,drc" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      //"NTUN", "Qfcccfff",  "TimeUS,WpDist,TargBrg,NavBrg,AltErr,XT,XTi,ArspdErr" },
      //"NTUN", "Qfffffffffffff",  "T,Hd,Hdc,HdR,q,qc,V,Vc,VR,Phi,Phic,PhiR,p,pc"},
      //"NTUN", "Qffffffff",  "TimeUS,Hd,Hdc,HdR,Ptch,Ptchc,q,qc,dec"},// Tune Hdot Dataset
      //"NTUN", "Qffffffffffff",  "TimeUS,V,Vc,VR,VdR,Vdc,TservoJ,Tservo,TpwmJ,Tpwm,VI,VP,Vd4H"},// Tune V Dataset
      //"NTUN", "Qffffffffff",  "TimeUS,phi,phic,phiR,phidc,pc,p,daR,rc,rMan,r"},// Tune Phi Dataset
        "NTUN", "Qffffffffffffff",  "TimeUS,V,Vc,VR,Hd,Hdc,HdR,q,qc,Phic,PhiR,p,pc,r,rc"},// Onboard Flight Dataset
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "QffBf",   "TimeUS,Dist,Volt,Cnt,Corr" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "QBH", "TimeUS,ArmState,ArmChecks" },
    { LOG_ATRP_MSG, sizeof(AP_AutoTune::log_ATRP),
      "ATRP", "QBBcfff",  "TimeUS,Type,State,Servo,Demanded,Achieved,P" },
    { LOG_STATUS_MSG, sizeof(log_Status),
      "STAT", "QBfBBBBBB",  "TimeUS,isFlying,isFlyProb,Armed,Safety,Crash,Still,Stage,Hit" },
    { LOG_QTUN_MSG, sizeof(QuadPlane::log_QControl_Tuning),
      "QTUN", "Qffffehhffff", "TimeUS,AngBst,ThrOut,DAlt,Alt,BarAlt,DCRt,CRt,DVx,DVy,DAx,DAy" },
#if OPTFLOW == ENABLED
    { LOG_OPTFLOW_MSG, sizeof(log_Optflow),
      "OF",   "QBffff",   "TimeUS,Qual,flowX,flowY,bodyX,bodyY" },
#endif
};

#if CLI_ENABLED == ENABLED
// Read the DataFlash.log memory : Packet Parser
void Plane::Log_Read(uint16_t list_entry, int16_t start_page, int16_t end_page)
{
    cliSerial->printf("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n",
                        (unsigned)hal.util->available_memory());

    cliSerial->println(HAL_BOARD_NAME);

	DataFlash.LogReadProcess(list_entry, start_page, end_page,
                             FUNCTOR_BIND_MEMBER(&Plane::print_flight_mode, void, AP_HAL::BetterStream *, uint8_t),
                             cliSerial);
}
#endif // CLI_ENABLED

void Plane::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by DataFlash
    Log_Write_Startup(TYPE_GROUNDSTART_MSG);
    DataFlash.Log_Write_Mode(control_mode);
    DataFlash.Log_Write_Rally(rally);
}

// start a new log
void Plane::start_logging() 
{
    DataFlash.set_mission(&mission);
    DataFlash.setVehicle_Startup_Log_Writer(
        FUNCTOR_BIND(&plane, &Plane::Log_Write_Vehicle_Startup_Messages, void)
        );

    DataFlash.StartNewLog();
}

/*
  initialise logging subsystem
 */
void Plane::log_init(void)
{
    DataFlash.Init(log_structure, ARRAY_SIZE(log_structure));
    if (!DataFlash.CardInserted()) {
        gcs_send_text(MAV_SEVERITY_WARNING, "No dataflash card inserted");
        g.log_bitmask.set(0);
    } else if (DataFlash.NeedPrep()) {
        gcs_send_text(MAV_SEVERITY_INFO, "Preparing log system");
        DataFlash.Prep();
        gcs_send_text(MAV_SEVERITY_INFO, "Prepared log system");
        for (uint8_t i=0; i<num_gcs; i++) {
            gcs[i].reset_cli_timeout();
        }
    }

    arming.set_logging_available(DataFlash.CardInserted());
}

#else // LOGGING_ENABLED

 #if CLI_ENABLED == ENABLED
bool Plane::print_log_menu(void) { return true; }
int8_t Plane::dump_log(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Plane::erase_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Plane::select_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
int8_t Plane::process_logs(uint8_t argc, const Menu::arg *argv) { return 0; }
 #endif // CLI_ENABLED == ENABLED

void Plane::do_erase_logs(void) {}
void Plane::Log_Write_Attitude(void) {}
void Plane::Log_Write_Performance() {}
void Plane::Log_Write_Startup(uint8_t type) {}
void Plane::Log_Write_Control_Tuning() {}
void Plane::Log_Write_Ju_Tuning_Hdot() {}
void Plane::Log_Write_Ju_Tuning_V() {}
void Plane::Log_Write_Ju_Tuning_Roll() {}
void Plane::Log_Write_Ju_Tuning_Yaw() {}
void Plane::Log_Write_Nav_Tuning() {}
void Plane::Log_Write_Status() {}
void Plane::Log_Write_Sonar() {}

 #if OPTFLOW == ENABLED
void Plane::Log_Write_Optflow() {}
 #endif

void Plane::Log_Write_Current() {}
void Plane::Log_Arm_Disarm() {}
void Plane::Log_Write_GPS(uint8_t instance) {}
void Plane::Log_Write_IMU() {}
void Plane::Log_Write_RC(void) {}
void Plane::Log_Write_Baro(void) {}
void Plane::Log_Write_Airspeed(void) {}
void Plane::Log_Write_Home_And_Origin() {}

 #if CLI_ENABLED == ENABLED
void Plane::Log_Read(uint16_t log_num, int16_t start_page, int16_t end_page) {}
 #endif // CLI_ENABLED

void Plane::start_logging() {}
void Plane::log_init(void) {}

#endif // LOGGING_ENABLED
