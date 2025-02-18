#include "Copter.h"

#if AP_DDS_ENABLED
#include <AP_DDS/AP_DDS_Client.h>
#endif

#if MODE_LOITER_ASSISTED_ENABLED == ENABLED

// Parameter defaults
#define DEFAULT_VEL_MAX                             100.0f
#define DEFAULT_MIN_OBS_DIST_CM                     250.0f // closest you can get to obstacle in lass
#define DEFAULT_POS_HZ                              2.0f // lower means less trust of new measurements
#define DEFAULT_WV_WIND                             5
#define DEFAULT_WV_THRESH                           0.1f 
#define DEFAULT_DOCK_SPEED_CM_S                     40.0f
#define DEFAULT_CID_CM                              80.0f // coast in distance
#define DEFAULT_COP_DEG                             5.0f
#define DEFAULT_THRO_PITCH_P_GAIN                   0.01f  // initial P gain
#define DEFAULT_THRO_PITCH_I_GAIN                   0.0f // initial I gain
#define DEFAULT_THRO_PITCH_D_GAIN                   0.002f // initial D gain
#define DEFAULT_THRO_PITCH_FF_GAIN                  0.0f  // initial feed-forward (FF)
#define DEFAULT_THRO_PITCH_IMAX                     0.5f // integrator max  is half throttle
#define DEFAULT_THRO_PITCH_ERR_HZ                   200.0f // error filter frequency in Hz
#define DEFAULT_THRO_PITCH_D_HZ                     200.0f  // derivative filter frequency in Hz
#define DEFAULT_STATIONARY_VEL_M_S                  0.05f    // vehicle velocity must be under m/s
#define DEFAULT_WINDDOWN_DECAY_TIME_S               6.0f // time to winddown throttle in seconds 
#define DEFAULT_LOWER_COAST_IN_PITCH_BOUND_DEG      -5.0f
#define DEFAULT_UPPER_COAST_IN_PITCH_BOUND_DEG      -2.0f
#define DEFAULT_MAX_TP_THROTTLE_RATE                0.1f // ten percent throttle per second default (~3-4 sec for hover throttle)
#define DEFAULT_DELTA_WIND_UP_PITCH_DEG             2.0f
#define DEFAULT_HEADING_NORMAL_TOL_DEG              10.0f    // degrees between heading and dock surface normal.
#define DEFAULT_RECOVERY_TOL_CM                     10.0f
#define DEFAULT_WIND_UP_THROTTLE_HOVER_MIN_RATIO    0.8f // we need to be at 80% of the hover throttle threshold to be considered wound up
#define DEFAULT_COAST_OUT_DIST_CM                   40.0f

// Hard coded parameters
#define BACKUP_RECOVERY_DIST_BACK_M        1.5f  // distance back from current pos the recovery target will be set to
#define BACKUP_RECOVERY_DIST_UP_M          0.25f  // distance up from current pos the recovery target will be set to
#define DOCK_COMMS_PERIOD_MS               1500  // when the dock expects an att_st message heartbeat
#define LOITER_ASSISTED_ACCELMAX_CMS       200.0f   
#define LEADUP_ACCELMAX_CMSS               125.0f

// Misc defines
#define DEG_TO_CD                       100.0f // centidegrees per degree
#define M_TO_CM                         100.0f // centimeters per meter


const AP_Param::GroupInfo ModeLoiterAssisted::var_info[] = {
    // @Param: VEL_MAX
    // @DisplayName: Max velocity commandable by sticks cm/s
    // @Description: Max velocity commandable by sticks cm/s
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("VEL_MAX", 1, ModeLoiterAssisted, _vel_max_cm_s, DEFAULT_VEL_MAX),

    // @Param: MIN_DIST
    // @DisplayName: Minimum dist to obstacle cm
    // @Description: Minimum distance that you can get to the obstacle cm
    // @Units: cm
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("MIN_DIST", 2, ModeLoiterAssisted, _min_obs_dist_cm, DEFAULT_MIN_OBS_DIST_CM),

    // @Param: POS_HZ
    // @DisplayName: Pos LPF alpha
    // @Description: This is the alpha for lpf on dist [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POS_HZ", 3, ModeLoiterAssisted, _dock_pos_filt_hz, DEFAULT_POS_HZ),

    // @Param: WV_WIND
    // @DisplayName: Window Var window size
    // @Description: min samples for window var estimator to provide variance estimate
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("WV_WIND", 4, ModeLoiterAssisted, _wv_window_size, DEFAULT_WV_WIND),

    // @Param: WV_THRESH
    // @DisplayName: Window Var threshold for good docking
    // @Description: Variance threshold that has to be met to enable docking
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("WV_THRESH", 5, ModeLoiterAssisted, _wv_thresh, DEFAULT_WV_THRESH),

    // @Param: DOCK_SPD
    // @DisplayName: Dock speed cm/s
    // @Description: speed to impact the target at cm/s
    // @Unit: mps
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("DOCK_SPD", 6, ModeLoiterAssisted, _dock_speed_cm_s, DEFAULT_DOCK_SPEED_CM_S),

    // @Param{Copter}: CID_M
    // @DisplayName: Coast in distance in cm
    // @Description: Coast in distance in cm
    // @Units: cm
    // @User: Advanced
    AP_GROUPINFO("CI_D_CM", 7, ModeLoiterAssisted, _coast_in_dist_cm, DEFAULT_CID_CM),

    // @Param{Copter}: COP_DEG
    // @DisplayName: Wind up pitch angle for wind up to target in degrees
    // @Description: Wind up pitch angle for wind up to target in degrees
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("CO_P_DEG", 8, ModeLoiterAssisted, _coast_out_pitch_deg, DEFAULT_COP_DEG),

    // @Param{Copter}: TP_P
    // @DisplayName: Wind up pitch angle for wind up to target in degrees
    // @Description: Wind up pitch angle for wind up to target in degrees
    // @User: Advanced
    AP_GROUPINFO("TP_P", 9, ModeLoiterAssisted, _thro_pitch_p, DEFAULT_THRO_PITCH_P_GAIN),

    // @Param{Copter}: TP_I
    // @DisplayName: Wind up pitch angle for wind up to target in degrees
    // @Description: Wind up pitch angle for wind up to target in degrees
    // @User: Advanced
    AP_GROUPINFO("TP_I", 10, ModeLoiterAssisted, _thro_pitch_i, DEFAULT_THRO_PITCH_I_GAIN),

    // @Param{Copter}: TP_D
    // @DisplayName: Wind up pitch angle for wind up to target in degrees
    // @Description: Wind up pitch angle for wind up to target in degrees
    // @User: Advanced
    AP_GROUPINFO("TP_D", 11, ModeLoiterAssisted, _thro_pitch_d, DEFAULT_THRO_PITCH_D_GAIN),

    // @Param{Copter}: TP_FF
    // @DisplayName: Wind up pitch angle for wind up to target in degrees
    // @Description: Wind up pitch angle for wind up to target in degrees
    // @User: Advanced
    AP_GROUPINFO("TP_FF", 12, ModeLoiterAssisted, _thro_pitch_ff, DEFAULT_THRO_PITCH_FF_GAIN),

    // @Param{Copter}: TP_IM
    // @DisplayName: Wind up pitch angle for wind up to target in degrees
    // @Description: Wind up pitch angle for wind up to target in degrees
    // @User: Advanced
    AP_GROUPINFO("TP_IM", 13, ModeLoiterAssisted, _thro_pitch_imax, DEFAULT_THRO_PITCH_IMAX),

    // @Param{Copter}: TP_EHZ
    // @DisplayName: Wind up pitch angle for wind up to target in degrees
    // @Description: Wind up pitch angle for wind up to target in degrees
    // @User: Advanced
    AP_GROUPINFO("TP_EHZ", 14, ModeLoiterAssisted, _thro_pitch_err_hz, DEFAULT_THRO_PITCH_ERR_HZ),
    
    // @Param{Copter}: TP_DHZ
    // @DisplayName: Wind up pitch angle for wind up to target in degrees
    // @Description: Wind up pitch angle for wind up to target in degrees
    // @User: Advanced
    AP_GROUPINFO("TP_DHZ", 15, ModeLoiterAssisted, _thro_pitch_d_hz, DEFAULT_THRO_PITCH_D_HZ),

    // @Param{Copter}: STAT_V
    // @DisplayName: Stationary velocity target
    // @Description: Stationary velocity target
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("STAT_V_MS", 16, ModeLoiterAssisted, _stationary_vel_m_s, DEFAULT_STATIONARY_VEL_M_S),

    // @Param{Copter}: WD_S
    // @DisplayName: Seconds spent winding down throttle
    // @Description: Stationary velocity target
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("WD_S", 17, ModeLoiterAssisted, _wind_down_decay_time_s, DEFAULT_WINDDOWN_DECAY_TIME_S),

    // @Param{Copter}: CIP_MIN
    // @DisplayName: Min coast in pitch degrees
    // @Description: Stationary velocity target
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("CI_P_MIN", 18, ModeLoiterAssisted, _lower_coast_in_pitch_bound_deg, DEFAULT_LOWER_COAST_IN_PITCH_BOUND_DEG),

    // @Param{Copter}: CIP_MAX
    // @DisplayName: Max coast in pitch degrees
    // @Description: Stationary velocity target
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("CI_P_MAX", 19, ModeLoiterAssisted, _upper_coast_in_pitch_bound_deg, DEFAULT_UPPER_COAST_IN_PITCH_BOUND_DEG),

    // @Param{Copter}: TP_SLEW
    // @DisplayName: Max rate of change of throttle in 0-1 units per second
    // @Description: Max rate of change of throttle in 0-1 units per second
    // @Units: units/s
    // @User: Advanced
    AP_GROUPINFO("TP_SLEW", 20, ModeLoiterAssisted, _max_TP_throttle_rate, DEFAULT_MAX_TP_THROTTLE_RATE),
    
    // @Param{Copter}: TP_SLEW
    // @DisplayName: Max rate of change of throttle in 0-1 units per second
    // @Description: Max rate of change of throttle in 0-1 units per second
    // @Units: units/s
    // @User: Advanced
    AP_GROUPINFO("WU_DP_DEG", 21, ModeLoiterAssisted, _delta_wind_up_pitch_deg, DEFAULT_DELTA_WIND_UP_PITCH_DEG),

    // @Param{Copter}: HN_TOL
    // @DisplayName: Heading to normal difference tolerance
    // @Description: Heading to normal difference tolerance
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("HN_TOL", 22, ModeLoiterAssisted, _heading_normal_tol_deg, DEFAULT_HEADING_NORMAL_TOL_DEG),

    // @Param{Copter}: REC_TOL
    // @DisplayName: Recovery tolerance before entering lass
    // @Description: Recovery tolerance before entering lass
    // @Units: cm
    // @User: Advanced
    AP_GROUPINFO("R_TOL_CM", 23, ModeLoiterAssisted, _recovery_tolerance_cm, DEFAULT_RECOVERY_TOL_CM),

    // @Param{Copter}: WU_TH_MIN
    // @DisplayName: Min ratio or throttle hover we need to hit 
    // @Description: Heading to normal difference tolerance
    // @User: Advanced
    AP_GROUPINFO("WU_TH_MIN", 24, ModeLoiterAssisted, _windup_throttle_hover_min_ratio, DEFAULT_WIND_UP_THROTTLE_HOVER_MIN_RATIO),

    // @Param{Copter}: CO_D_CM
    // @DisplayName: Coast out distance cm
    // @Description: Coast out distance cm
    // @Units: cm
    // @User: Advanced
    AP_GROUPINFO("CO_D_CM", 25, ModeLoiterAssisted, _coast_out_dist_cm, DEFAULT_COAST_OUT_DIST_CM),
    
    AP_GROUPEND
};

ModeLoiterAssisted::ModeLoiterAssisted(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}


// loiter_init - initialise loiter controller
bool ModeLoiterAssisted::init(bool ignore_checks)
{
    #if AP_DDS_ENABLED
    if (AP_DDS_Client::attached_state != _flags.ATTACHED) {
        set_attached_status(static_cast<float>(AP_DDS_Client::attached_state));
    }  
    #endif

    // Failsafes
    if (copter.failsafe.radio) {return false;}// TODO what failsafe mode should we use?
    if (!ahrs.get_relative_position_NED_origin(_cur_pos_NED_m)) {return false;}
    float target_climb_rate_cm_s = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -get_pilot_speed_dn(), g.pilot_speed_up);
    AltHoldModeState alt_hold_state = get_alt_hold_state(target_climb_rate_cm_s);
    if (!(alt_hold_state == AltHoldModeState::Flying || _flags.ATTACHED)) {return false;}

    // Pos control inits
    if (!pos_control->is_active_z()) {pos_control->init_z_controller();}
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_max_speed_accel_xy(_vel_max_cm_s, LOITER_ASSISTED_ACCELMAX_CMS);
    pos_control->set_correction_speed_accel_xy(_vel_max_cm_s, LOITER_ASSISTED_ACCELMAX_CMS);
    copter.set_simple_mode(Copter::SimpleMode::NONE); // disable simple mode for this mode. TODO: Validate

    // Initialize all filters owned by this mode
    InitFilters();

    // Set State (assuming not attached for now)
    TRAN(&ModeLoiterAssisted::Default);
    (this->*_lass_state)(Event::ENTRY_SIG); // perform entry actions

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode set to Loiter Assisted");
    return true;
}

// should be called at 100hz or more
void ModeLoiterAssisted::run()
{
    if (copter.failsafe.radio) {GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"Radio failsafe");}
    #if AP_DDS_ENABLED
    if (AP_DDS_Client::attached_state != _flags.ATTACHED) {
        set_attached_status(static_cast<float>(AP_DDS_Client::attached_state));
    }  
    #endif

    if (!ahrs.get_relative_position_NED_origin(_cur_pos_NED_m) || !ahrs.get_velocity_NED(_velocity_NED_m)) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"No pos or vel estimate!");
        abortExit();
    }

    checkDockComms();
    updateFilterParams(); // Update filters (simply check for param changes)
    findDockTarget(); // calculate dock's position. compute navigation data. sets dock related flags
    evaluateFlags(); // evaluate some flags
    
    evaluate_transitions(); // evaluate transitions
    runFlightCode(); // run flight code for current state
    
    logLass(); // log everything of interest that isn't already logged elsewhere (i.e. pos and vel)
}

/*---------------------------------------------------------------------------*/
/* Finite State Machine States... */
ModeLoiterAssisted::Status ModeLoiterAssisted::Default(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::Default;
        gcs().send_named_float("lass", float(_lass_state_name));
        _crash_check_enabled = true;
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.DOCK_FOUND) {status = TRAN(&ModeLoiterAssisted::Lass);}
        else if (_flags.ATTACHED) {status = TRAN(&ModeLoiterAssisted::WindDown);}
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        float target_roll, target_pitch;
        float target_yaw_rate = 0.0f;
        float target_climb_rate_cm_s = 0.0f;
        pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        target_climb_rate_cm_s = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -get_pilot_speed_dn(), g.pilot_speed_up);
        loiter_nav->update();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false); // call attitude controller
        target_climb_rate_cm_s = get_avoidance_adjusted_climbrate(target_climb_rate_cm_s); // get avoidance adjusted climb rate
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s); // Send the commanded climb rate to the position controller
        pos_control->update_z_controller();
        
        break;}
    default:{
        break;}
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::Lass(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::Lass;
        gcs().send_named_float("lass", float(_lass_state_name));
        _crash_check_enabled = true;
        Vector3f zero_vel;
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (!_flags.DOCK_FOUND) {status = TRAN(&ModeLoiterAssisted::Default);}
        else if (_flags.ATTACH_BUTTON_PRESSED && _flags.DOCK_STABLE && _flags.VEHICLE_STATIONARY && _flags.HEADING_NORMAL_ALIGNED) {status = TRAN(&ModeLoiterAssisted::LeadUp);}
        else if (_flags.ATTACHED) {status = TRAN(&ModeLoiterAssisted::WindDown);}
        else if (_flags.ATTACH_BUTTON_PRESSED) {
            //Return why we are not switching to lead up
            if (millis()-_last_send_lass > _statustext_period_ms) {
                if (!_flags.DOCK_STABLE) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Variance: %.4fm >= %.4fm", _dock_target_var, _wv_thresh.get());
                    _last_send_lass = millis();
                }
                if (!_flags.VEHICLE_STATIONARY) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Vel: (%.4f, %.4f, %.4f). Len: %.4f > %.4f", 
                        _velocity_NED_m.x, _velocity_NED_m.y, _velocity_NED_m.z, _velocity_NED_m.length(), _stationary_vel_m_s.get());
                    _last_send_lass = millis();
                }
                if (!_flags.HEADING_NORMAL_ALIGNED) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Heading-normal error: %.4f >= %.4f", 
                        _heading_normal_error_deg, _heading_normal_tol_deg.get());
                    _last_send_lass = millis();
                }
            }
        }
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        // Compute ideal position (Along normal vec)
        float filt_heading_cmd_deg = get_bearing_cd(_cur_pos_NED_m.xy(),_filt_dock_xyz_NEU_m.xy())/DEG_TO_CD;
        Vector2f target_xy_NE_cm = _filt_dock_xyz_NEU_m.xy()*100.0f + _filt_dock_normal_NE * _min_obs_dist_cm.get();
        Vector2p pos_xy = target_xy_NE_cm.topostype();
        Vector2f vel_xy;
        Vector2f acc_xy;
        pos_control->input_pos_vel_accel_xy(pos_xy, vel_xy, acc_xy);

        // Yaw controller
        AC_AttitudeControl::HeadingCommand heading_cmd;
        heading_cmd.heading_mode = AC_AttitudeControl::HeadingMode::Angle_Only;
        heading_cmd.yaw_angle_cd = filt_heading_cmd_deg*DEG_TO_CD; 
        heading_cmd.yaw_rate_cds = 0.0f;

        // z controller
        float target_climb_rate_cm_s = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -get_pilot_speed_dn(), g.pilot_speed_up);
        target_climb_rate_cm_s = get_avoidance_adjusted_climbrate(target_climb_rate_cm_s); // get avoidance adjusted climb rate
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s); // Send the commanded climb rate to the position controller

        // run controllers
        pos_control->update_xy_controller();
        pos_control->update_z_controller();
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading_cmd);
        lasmData data;
        data.hdg = filt_heading_cmd_deg;
        data.tpX = target_xy_NE_cm.x/M_TO_CM;
        data.tpY = target_xy_NE_cm.y/M_TO_CM;
        data.tvZ = target_climb_rate_cm_s/M_TO_CM;
        logLasm(data);
        break;}
    default:{
        break;}
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::LeadUp(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::LeadUp;
        gcs().send_named_float("lass", float(_lass_state_name));
        #if AP_DDS_ENABLED
        AP_DDS_Client::need_to_pub_attach_detach = true;
        AP_DDS_Client::desire_attach = true;
        #endif
        gcs().send_named_float("attach", 1.0f);
        _crash_check_enabled = true;
        
        
        float heading_rad = get_bearing_cd(_cur_pos_NED_m.xy(),_filt_dock_xyz_NEU_m.xy())/DEG_TO_CD*DEG_TO_RAD;
        // float heading_rad = atan2f(-_filt_dock_normal_NE.y,-_filt_dock_normal_NE.x);
        _locked_heading_deg = heading_rad*RAD_TO_DEG;
        _locked_vel_NE_cm_s = Vector2f(cosf(heading_rad),sinf(heading_rad))*_dock_speed_cm_s;
        _recovery_position_NE_m = _cur_pos_NED_m.xy();
        _locked_dock_pos_NE_m = _filt_dock_xyz_NEU_m.xy();

        pos_control->set_max_speed_accel_xy(_dock_speed_cm_s, LEADUP_ACCELMAX_CMSS);
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        _locked_dock_pos_NE_m = Vector2f(INFINITY, INFINITY); // get rid of locked dock target because we don't care if we leave leadup
        pos_control->set_max_speed_accel_xy(_vel_max_cm_s, LOITER_ASSISTED_ACCELMAX_CMS);
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.WITHIN_COAST_IN_DIST) {status = TRAN(&ModeLoiterAssisted::CoastIn);}
        else if (_flags.ATTACHED) {status = TRAN(&ModeLoiterAssisted::CoastIn);}
        else if (!_flags.ATTACH_BUTTON_PRESSED) {status = TRAN(&ModeLoiterAssisted::Lass);}
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        AC_AttitudeControl::HeadingCommand heading_cmd;
        heading_cmd.heading_mode = AC_AttitudeControl::HeadingMode::Angle_Only;
        heading_cmd.yaw_angle_cd = _locked_heading_deg*DEG_TO_CD;
        heading_cmd.yaw_rate_cds = 0.0f;

        Vector2p target_pos = (_locked_dock_pos_NE_m*M_TO_CM).topostype();
        Vector2f target_vel = _locked_vel_NE_cm_s;

        pos_control->input_pos_vel_accel_xy(target_pos, target_vel, Vector2f(0.0f,0.0f));
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f); // no climb rate

        pos_control->update_xy_controller();
        pos_control->update_z_controller();
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading_cmd);

        lasmData data;
        data.hdg = _locked_heading_deg;
        data.tpX = _locked_dock_pos_NE_m.x,
        data.tpY = _locked_dock_pos_NE_m.y,
        data.tvX = _locked_vel_NE_cm_s.x/M_TO_CM;
        data.tvY = _locked_vel_NE_cm_s.y/M_TO_CM;
        data.tvZ = 0.0f;
        logLasm(data);
        break;}
    default:{
        break;}
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::CoastIn(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::CoastIn;
        gcs().send_named_float("lass", float(_lass_state_name));
        _crash_check_enabled = false;
        _coast_in_pitch_cd = pos_control->get_pitch_cd();
        _coast_in_pitch_cd = constrain_float(_coast_in_pitch_cd, _lower_coast_in_pitch_bound_deg*DEG_TO_CD, _upper_coast_in_pitch_bound_deg*DEG_TO_CD); // constrain
        motors->disable_throttle_hover_learn(true);
        _thro_pitch_pid.reset_filter();
        _thro_pitch_pid.reset_I();
        break;}
    case Event::EXIT_SIG:{ 
        motors->disable_throttle_hover_learn(false);
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.DETACH_BUTTON_PRESSED) {status = TRAN(&ModeLoiterAssisted::CoastOut);}
        // else if (_flags.ATTACHED) {status = TRAN(&ModeLoiterAssisted::WindDown);}
        else if (_flags.WU_WD_CONFIRMATION && _flags.ATTACHED) {status = TRAN(&ModeLoiterAssisted::WindDown);}
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        if (!_flags.ATTACHED){
            pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
            pos_control->update_z_controller();
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, _coast_in_pitch_cd, 0.0f);
        } 
        else {
            float current_pitch_deg = ahrs.get_pitch()*RAD_TO_DEG;
            float throttle = -_thro_pitch_pid.update_all(_coast_in_pitch_cd/DEG_TO_CD, current_pitch_deg, G_Dt) + throttle_hover();

            throttle = constrain_float(throttle, 0.0f, 1.0f);
            attitude_control->set_throttle_out(throttle, false, g.throttle_filt);
            attitude_control->relax_attitude_controllers();
        }

        lasmData data;
        data.hdg = _locked_heading_deg;
        data.pit = _coast_in_pitch_cd/DEG_TO_CD;
        data.tvZ = 0.0f;
        logLasm(data);
        break;}
    default:{
        break;}
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::WindDown(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::WindDown;
        gcs().send_named_float("lass", float(_lass_state_name));
        _crash_check_enabled = false;
        _wind_down_throttle_start = attitude_control->get_throttle_in();
        _wind_down_start_ms = millis();
        attitude_control->landed_gain_reduction(true);
        _flags.THROTTLE_WOUND_DOWN = false;
        motors->disable_throttle_hover_learn(true);
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        _flags.THROTTLE_WOUND_DOWN = false;
        motors->disable_throttle_hover_learn(false);
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.VEHICLE_STATIONARY && _flags.THROTTLE_WOUND_DOWN) {status = TRAN(&ModeLoiterAssisted::Vegetable);} // are both necessary?
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        // float roll_cmd_cd = ahrs.get_roll()*RAD_TO_DEG*DEG_TO_CD; // TODO: Basically acts like relax roll, need to actively control roll when new dock mechanism is made
        // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_cmd_cd,ahrs.get_pitch()*RAD_TO_DEG*DEG_TO_CD,0.0f);
        float wind_down_throttle = _wind_down_throttle_start * 
            (1.0f - (float(millis() - _wind_down_start_ms)/1000.0f/_wind_down_decay_time_s.get()));
        wind_down_throttle = MAX(wind_down_throttle,0.0f);
        attitude_control->set_throttle_out(wind_down_throttle, false, g.throttle_filt);
        if (wind_down_throttle < FLT_EPSILON) {_flags.THROTTLE_WOUND_DOWN = true;} 
        else {_flags.THROTTLE_WOUND_DOWN = false;}
        // attitude_control->relax_attitude_controllers();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
            ahrs.get_roll()*RAD_TO_DEG*DEG_TO_CD, // current roll
            ahrs.get_pitch()*RAD_TO_DEG*DEG_TO_CD, // current pitch
            0.0f // zero yaw rate
        ); // might not be needed. or might need to change to target a specific yaw...

        lasmData data;
        data.thr = wind_down_throttle;
        data.rol = ahrs.get_roll()*RAD_TO_DEG;
        data.pit = ahrs.get_pitch()*RAD_TO_DEG;
        logLasm(data);
        break;}
    default:{
        break;}
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::Vegetable(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::Vegetable;
        gcs().send_named_float("lass", float(_lass_state_name));
        _crash_check_enabled = false;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        motors->disable_throttle_hover_learn(true);
        // _docked_position_NED_m = _cur_pos_NED_m;
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        motors->disable_throttle_hover_learn(false);
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (!_flags.WU_WD_CONFIRMATION) {status = TRAN(&ModeLoiterAssisted::WindUp);}
        // TODO add falling check
        // else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        lasmData data;
        logLasm(data);
        
        break;}
    default:{
        break;}
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::WindUp(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::WindUp;
        gcs().send_named_float("lass", float(_lass_state_name));
        _crash_check_enabled = false;
        _is_taking_off = true;
        set_land_complete(false);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_target_and_rate();
        attitude_control->set_throttle_out(0.0f, false, g.throttle_filt); // Start at zero throttle
        _thro_pitch_pid.reset_filter();
        _thro_pitch_pid.reset_I();
        _wind_up_throttle = 0.0f;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        motors->disable_throttle_hover_learn(true);
        _wind_up_pitch_deg = ahrs.get_pitch()*RAD_TO_DEG - _delta_wind_up_pitch_deg;
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        _is_taking_off = false;
        motors->disable_throttle_hover_learn(false);
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.AT_WIND_UP_THROTTLE && _flags.DETACH_BUTTON_PRESSED) {status = TRAN(&ModeLoiterAssisted::CoastOut);} 
        else if (_flags.WU_WD_CONFIRMATION) {status = TRAN(&ModeLoiterAssisted::WindDown);} 
        else if (_flags.DETACH_BUTTON_PRESSED){
            if (millis()-_last_send_windup > _statustext_period_ms) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "thro: %.4f", motors->get_throttle_out());
                _last_send_windup = millis();
            }
        }
        else{}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        if (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) { // Motors are spooled up
            // Compute throttle adjustment using PID
            float last_throttle = _wind_up_throttle;
            float current_pitch_deg = ahrs.get_pitch()*RAD_TO_DEG;
            float old_integrator = _thro_pitch_pid.get_i();
            _wind_up_throttle = -_thro_pitch_pid.update_all(_wind_up_pitch_deg, current_pitch_deg, G_Dt) + throttle_hover();
            // --- Slew Rate Limiter ---
            // Calculate the maximum allowed change in throttle for this update cycle.
            // G_Dt is the time step (in seconds) between updates.
            float max_delta = _max_TP_throttle_rate * G_Dt;
            float delta = _wind_up_throttle - last_throttle;
            if (delta > max_delta) {
                _wind_up_throttle = last_throttle + max_delta;
                _thro_pitch_pid.set_integrator(old_integrator);
            } else if (delta < -max_delta) {
                _wind_up_throttle = last_throttle - max_delta;
                _thro_pitch_pid.set_integrator(old_integrator);
            }
            // --- End Slew Rate Limiter ---

            _wind_up_throttle = constrain_float(_wind_up_throttle, 0.0f, 1.0f);
            attitude_control->set_throttle_out(_wind_up_throttle, true, g.throttle_filt);
            attitude_control->relax_attitude_controllers();
        }

        lasmData data;
        data.thr = _wind_up_throttle;
        data.rol = ahrs.get_roll()*RAD_TO_DEG;
        data.pit = ahrs.get_pitch()*RAD_TO_DEG;
        logLasm(data);

        break;}
    default:{
        break;}
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::CoastOut(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::CoastOut;
        gcs().send_named_float("lass", float(_lass_state_name));
        _crash_check_enabled = true;
        
        #if AP_DDS_ENABLED
        // AP_DDS publish detach message
        AP_DDS_Client::need_to_pub_attach_detach = true;
        AP_DDS_Client::desire_attach = false;
        #endif
        gcs().send_named_float("attach", 0.0f);
        pos_control->init_xy_controller();
        pos_control->init_z_controller();   
        _docked_position_NED_m = _cur_pos_NED_m;
        motors->disable_throttle_hover_learn(true);
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        motors->disable_throttle_hover_learn(false);
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.BEYOND_COAST_OUT_DIST) {status = TRAN(&ModeLoiterAssisted::Recover);}
        // else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, _coast_out_pitch_deg*DEG_TO_CD, 0.0f);

        lasmData data;
        data.pit = _coast_out_pitch_deg;
        logLasm(data);

        break;}
    default:{
        break;}
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::Recover(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::Recover;
        gcs().send_named_float("lass", float(_lass_state_name));
        _crash_check_enabled = true;
        // If no recovery pos exists, set one 2 meters back and .5 meter up?
        if (is_zero(_recovery_position_NE_m.x) && is_zero(_recovery_position_NE_m.y)) {
            Vector2f unit_heading_vec{1.0f,0.0f};
            unit_heading_vec.rotate(ahrs.get_yaw());
            _recovery_position_NE_m = _cur_pos_NED_m.xy() - unit_heading_vec * BACKUP_RECOVERY_DIST_BACK_M;
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }
        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.AT_RECOVERY_POSITION) {status = TRAN(&ModeLoiterAssisted::Lass);}
        // else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        Vector2p pos_xy = (_recovery_position_NE_m*M_TO_CM).topostype();
        Vector2f vel_xy;
        Vector2f acc_xy;
        pos_control->input_pos_vel_accel_xy(pos_xy, vel_xy, acc_xy);

        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);

        pos_control->update_xy_controller();
        pos_control->update_z_controller();
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f, true);
        
        lasmData data;
        data.tpX = pos_xy.x,
        data.tpY = pos_xy.y,
        data.tvZ = 0.0f,
        data.hdg = ahrs.get_yaw()*RAD_TO_DEG;
        logLasm(data);

        break;}
    default:{
        break;}
    }
    return status;
}


/*---------------------------------------------------------------------------*/
/* Finite State Machine facilities... */
void ModeLoiterAssisted::evaluate_transitions() {
    Status status = Status::HANDLED_STATUS;
    StateHandler prev_state = _lass_state; /* save for later */

    status = (this->*_lass_state)(Event::EVALUATE_TRANSITIONS);

    if (status == Status::TRAN_STATUS) { /* transition taken? */
        (this->*prev_state)(Event::EXIT_SIG);
        (this->*_lass_state)(Event::ENTRY_SIG);
    }
}

void ModeLoiterAssisted::runFlightCode() {
    Status status = (this->*_lass_state)(Event::RUN_FLIGHT_CODE);
    // We can do something with status if we need to now
    if (status == Status::TRAN_STATUS) {
        //ignore for now
    }
}

void ModeLoiterAssisted::runStateExitCode() {
    Status status = (this->*_lass_state)(Event::EXIT_SIG);
    // We can do something with status if we need to now
    if (status == Status::TRAN_STATUS) {
        //ignore for now
    }
}



/*..........................................................................*/
/* Helper Functions... */
void ModeLoiterAssisted::findDockTarget(){
    Vector2f dock_normal_vec;
    Vector2f cfit_center_xy_m;
    _flags.DOCK_FOUND = false; 

    if (!g2.proximity.curvefit->get_target(dock_normal_vec, cfit_center_xy_m)) {return;} // return if we don't find an obstacle
    _flags.DOCK_FOUND = true;

    if (!is_equal(cfit_center_xy_m.x,_last_cfit_center_xy_m_x) || !is_equal(cfit_center_xy_m.y,_last_cfit_center_xy_m_y)){ // when new lidar data comes in
        _flags.DOCK_STABLE = false;
        _flags.HEADING_NORMAL_ALIGNED = false;
        // Filter and evaluate dock center position
        _last_cfit_center_xy_m_x = cfit_center_xy_m.x;
        _last_cfit_center_xy_m_y = cfit_center_xy_m.y;
        float x = cfit_center_xy_m.x;
        float y = cfit_center_xy_m.y;
        float z = _cur_pos_NED_m.z;
        const Vector3f dock_pos{x,y,z};
        _filt_dock_xyz_NEU_m = _dock_pos_filter.apply(dock_pos); // low pass filter on dock position
        Vector3f dock_target_var_vec;
        if (_dock_target_window_var.apply(dock_pos, dock_target_var_vec)) { // keep track of variance (deviation) of center
            _dock_target_var = dock_target_var_vec.length();
            if (_dock_target_var < _wv_thresh.get()) {
                _flags.DOCK_STABLE = true;
            } else {
                _flags.DOCK_STABLE = false;
            }
        }
        // Filter and evaluate dock normal vector
        _filt_dock_normal_NE = _dock_norm_filter.apply(dock_normal_vec);
        _heading_normal_error_deg = wrap_PI(ahrs.get_yaw() - _filt_dock_normal_NE.angle() + float(M_PI))*RAD_TO_DEG;
        if (abs(_heading_normal_error_deg) < _heading_normal_tol_deg.get()) {
            _flags.HEADING_NORMAL_ALIGNED = true;
        } else {
            _flags.HEADING_NORMAL_ALIGNED = false;
        }
    }
}

void ModeLoiterAssisted::attach() { // init attach engaged via RC_Channel
    _flags.ATTACH_BUTTON_PRESSED = true;
    _flags.DETACH_BUTTON_PRESSED = false; // These are currently mapped to the same button, they could be mapped so seperate buttons in future
}

void ModeLoiterAssisted::detach() { // init detach engaged via RC_Channel
    _flags.ATTACH_BUTTON_PRESSED = false;
    _flags.DETACH_BUTTON_PRESSED = true;
}

void ModeLoiterAssisted::WindDownSafe() { // init attach engaged via RC_Channel
    _flags.WU_WD_CONFIRMATION = true;
}

void ModeLoiterAssisted::WindDownUnSafe() { // init detach engaged via RC_Channel
    _flags.WU_WD_CONFIRMATION = false;
}

void ModeLoiterAssisted::set_attached_status(float att_st) { // start being attached
    _flags.ATTACHED = !is_zero(att_st);
    _last_att_st_time = millis();
    _flags.DOCK_COMMS_HEALTHY = true;
}

void ModeLoiterAssisted::checkDockComms() {
    if (millis() - _last_att_st_time > DOCK_COMMS_PERIOD_MS) {
        _flags.DOCK_COMMS_HEALTHY = false;
    }
}

void ModeLoiterAssisted::logLass() { 
    // convert flags to bitmask
    uint16_t flags_bitmask = 0;
    flags_bitmask |= (_flags.DOCK_FOUND           ? 1 : 0) << 0;
    flags_bitmask |= (_flags.DOCK_STABLE          ? 1 : 0) << 1;
    flags_bitmask |= (_flags.WITHIN_COAST_IN_DIST     ? 1 : 0) << 2;
    flags_bitmask |= (_flags.ATTACHED             ? 1 : 0) << 3;
    flags_bitmask |= (_flags.VEHICLE_STATIONARY     ? 1 : 0) << 4;
    flags_bitmask |= (_flags.ATTACH_BUTTON_PRESSED ? 1 : 0) << 5;
    flags_bitmask |= (_flags.AT_RECOVERY_POSITION ? 1 : 0) << 6;
    flags_bitmask |= (_flags.BEYOND_COAST_OUT_DIST ? 1 : 0) << 7;
    flags_bitmask |= (_flags.THROTTLE_WOUND_DOWN ? 1 : 0) << 8;
    flags_bitmask |= (_flags.AT_WIND_UP_THROTTLE ? 1 : 0) << 9;
    flags_bitmask |= (_flags.HEADING_NORMAL_ALIGNED ? 1 : 0) << 10;
    flags_bitmask |= (_flags.DETACH_BUTTON_PRESSED ? 1 : 0) << 11;
    flags_bitmask |= (_flags.DOCK_COMMS_HEALTHY ? 1 : 0) << 12;
    flags_bitmask |= (_flags.WU_WD_CONFIRMATION ? 1 : 0) << 13;

    if (millis()-_last_lass_log_time > _log_period_ms) {
        AP::logger().Write(
        "LASS", // heading name
        "TimeUS,dockX,dockY,dockN,state,flags,spd", // field labels
        "smmd--n", // units
        "F000--0", // mults
        "QfffBHf", // format
        AP_HAL::micros64(),
        _filt_dock_xyz_NEU_m.x,
        _filt_dock_xyz_NEU_m.y,
        _filt_dock_normal_NE.angle()*RAD_TO_DEG,
        uint8_t(_lass_state_name),
        flags_bitmask,
        _velocity_NED_m.length()
        );
        _last_lass_log_time = millis();
    }
}

void ModeLoiterAssisted::logLasm(const lasmData &data) { 
    if (millis()-_last_lasm_log_time > _log_period_ms) {
        AP::logger().Write(
            "LASM", // heading name
            "TimeUS,tpX,tpY,tpZ,tvX,tvY,tvZ,rol,pit,hdg,thr,dss,ss", // field labels
            "smmmnnnddd---", // units
            "F000000000000", // mults
            "QffffffffffBB", // format
            AP_HAL::micros64(),
            data.tpX, // cmded x position NEU m
            data.tpY, // cmded y position NEU m
            data.tpZ, // cmded z position NEU m
            data.tvX, // cmded x velocity NEU m/s
            data.tvY, // cmded y velocity NEU m/s
            data.tvZ, // cmded z velocity NEU m/s
            data.rol, // cmded roll deg 
            data.pit, // cmded pitch deg
            data.hdg, // cmded yaw deg
            data.thr, // cmded throttle unitless
            uint8_t(motors->get_desired_spool_state()), //desired spool state
            uint8_t(motors->get_spool_state()) //spool state
        );
        _last_lasm_log_time = millis();
    }
}

// Note: This function doesn't evaluate all flags
void ModeLoiterAssisted::evaluateFlags() { // ALL FLAGS MUST BE SET TO FALSE INITIALLY!
    _flags.WITHIN_COAST_IN_DIST = false;
    _flags.AT_RECOVERY_POSITION = false;
    _flags.BEYOND_COAST_OUT_DIST = false;
    _flags.VEHICLE_STATIONARY = false;
    _flags.AT_WIND_UP_THROTTLE = false;

    // Check if we are at the recovery position
    float dist_to_recovery_pos_cm = (_recovery_position_NE_m-_cur_pos_NED_m.xy()).length()*M_TO_CM;
    if (dist_to_recovery_pos_cm < _recovery_tolerance_cm) {
        _flags.AT_RECOVERY_POSITION = true;
    }

    // Check if we are beyond the coast out distance
    float dist_from_docked_pos_cm = (_docked_position_NED_m.xy()-_cur_pos_NED_m.xy()).length()*M_TO_CM;
    if (dist_from_docked_pos_cm > _coast_out_dist_cm.get()) {
        _flags.BEYOND_COAST_OUT_DIST = true;
    }

    // Check if we are within the coast in distance
    if (_flags.DOCK_FOUND && !_locked_dock_pos_NE_m.is_inf()) { 
    // if (_flags.DOCK_FOUND) { 
        // _locked_vel_NE_cm_s/_dock_speed_cm_s // This is the unit vector for LeadUp
        _dist_to_dock_cm = abs((_locked_dock_pos_NE_m - _cur_pos_NED_m.xy()).dot(_locked_vel_NE_cm_s/_dock_speed_cm_s))*M_TO_CM; // This gives the value along that projected vector
        // _dist_to_dock_cm = (_filt_dock_xyz_NEU_m.xy()-_cur_pos_NED_m.xy()).length()*M_TO_CM;
        if (_dist_to_dock_cm < _coast_in_dist_cm.get()) {
            _flags.WITHIN_COAST_IN_DIST = true;
        }
    }
    
    // Check if we are stationary
    _flags.VEHICLE_STATIONARY = (_velocity_NED_m.length() <= _stationary_vel_m_s.get());

    // Check if we are at the wind up pitch
    _flags.AT_WIND_UP_THROTTLE = motors->get_throttle_out() > motors->get_throttle_hover()*_windup_throttle_hover_min_ratio;
}

void ModeLoiterAssisted::abortExit() {
    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"Abort Exiting LASS");
    if (!copter.set_mode(copter.prev_control_mode, ModeReason::UNKNOWN)) {
        // this should never happen but just in case
        copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
    }
}

// send exit lass message
void ModeLoiterAssisted::exit()
{
    pos_control->set_max_speed_accel_xy(POSCONTROL_SPEED, POSCONTROL_ACCEL_XY);
    runStateExitCode();
    gcs().send_named_float("lass", 9);
}

// must be unset on exit of state! Must never be called twice in a row! Don't want to overwrite originals
void ModeLoiterAssisted::set_attitude_control_rate_limits(float limit_deg_s) {
    _original_roll_limit = attitude_control->get_ang_vel_roll_max_degs();
    _original_pitch_limit = attitude_control->get_ang_vel_pitch_max_degs();
    _original_yaw_limit = attitude_control->get_ang_vel_yaw_max_degs();
    attitude_control->set_ang_vel_roll_max_degs(limit_deg_s);
    attitude_control->set_ang_vel_pitch_max_degs(limit_deg_s);
    attitude_control->set_ang_vel_yaw_max_degs(limit_deg_s);
}

void ModeLoiterAssisted::unset_attitude_control_rate_limits() {
    attitude_control->set_ang_vel_roll_max_degs(_original_roll_limit);
    attitude_control->set_ang_vel_pitch_max_degs(_original_pitch_limit);
    attitude_control->set_ang_vel_yaw_max_degs(_original_yaw_limit);
}


void ModeLoiterAssisted::InitFilters() {
    _dock_pos_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _dock_pos_filt_hz.get()); 
    _dock_norm_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _dock_pos_filt_hz.get()); 
    _dock_target_window_var = WindowVar(_wv_window_size.get()); // reinit with new min samples
    _dock_pos_filter.reset();
    _dock_norm_filter.reset();
    _dock_target_window_var.reset();
}

void ModeLoiterAssisted::updateFilterParams() {
    // check for filter change
    if (!is_equal(_dock_pos_filter.get_cutoff_freq(), _dock_pos_filt_hz.get())) { // TODO update to dock_hz
        _dock_pos_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _dock_pos_filt_hz.get());
    }
    if (!is_equal(_dock_norm_filter.get_cutoff_freq(), _dock_pos_filt_hz.get())) { // TODO update to dock_hz
        _dock_norm_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _dock_pos_filt_hz.get());
    }
    if (!is_equal(_dock_target_window_var.get_window_size(), _wv_window_size.get())) {
        _dock_target_window_var.set_new_window_size(_wv_window_size.get());
    }

    _thro_pitch_pid.set_kP(_thro_pitch_p.get());
    _thro_pitch_pid.set_kI(_thro_pitch_i.get());
    _thro_pitch_pid.set_kD(_thro_pitch_d.get());
    _thro_pitch_pid.set_ff(_thro_pitch_ff.get());
    _thro_pitch_pid.set_imax(_thro_pitch_imax.get());
    _thro_pitch_pid.set_filt_E_hz(_thro_pitch_err_hz.get());
    _thro_pitch_pid.set_filt_D_hz(_thro_pitch_d_hz.get());
}

/*..........................................................................*/
/*...........Window Var Functions............................*/

bool ModeLoiterAssisted::WindowVar::apply(Vector3f value, Vector3f &current_variance) {
    // Add the new value to the window
    _values.push_back(value);

    // Update sums
    _sum += value;
    _sum_of_squares += Vector3f(value.x * value.x, value.y * value.y, value.z * value.z);

    // Remove the oldest value if the window size is exceeded
    if (_values.size() > _window_size) {
        Vector3f old_value = _values.front();
        _values.pop_front();
        _sum -= old_value;
        _sum_of_squares -= Vector3f(old_value.x * old_value.x, old_value.y * old_value.y, old_value.z * old_value.z);
    }

    size_t n = _values.size();
    if (n == 0) {
        current_variance = Vector3f(INFINITY,INFINITY,INFINITY); // No variance for an empty window
        return false;
    }

    Vector3f mean = _sum / n;
    current_variance = (_sum_of_squares / n) - Vector3f(mean.x * mean.x, mean.y * mean.y, mean.z * mean.z);

    if (n < _window_size) {
        return false; // No variance for an empty window
    }
    return true;
}

void ModeLoiterAssisted::WindowVar::reset() {
    _sum = Vector3f(0.0f,0.0f,0.0f);
    _sum_of_squares = Vector3f(0.0f,0.0f,0.0f);
    _values = std::deque<Vector3f>();
}
#endif
