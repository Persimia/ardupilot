#include "Copter.h"

#if AP_DDS_ENABLED
#include <AP_DDS/AP_DDS_Client.h>
#endif

#if MODE_LOITER_ASSISTED_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

#define VEL_MAX_DEFAULT                      50
#define MIN_OBS_DIST_CM_DEFAULT              50
#define YAW_HZ_DEFAULT                       100.0
#define DIST_HZ_DEFAULT                      100.0
#define POS_HZ_DEFAULT                       100.0
#define WV_WIND_DEFAULT                      5
#define WV_THRESH_DEFAULT                    0.1 
#define DOCK_SPEED_CMS_DEFAULT               10.0
#define UNDOCK_SPEED_CMS_DEFAULT             50.0

#define LAND_DETECTOR_ACCEL_MAX            1.0f    // vehicle acceleration must be under 1m/s/s
#define WIND_UP_PITCH_TOL                  0.5f   
#define LOWER_COAST_IN_PITCH_BOUND         -2.0f
#define UPPER_COAST_IN_PITCH_BOUND         0.0f
#define RECOVERY_DIST_THRESH_CM            50.0f
#define COAST_OUT_DIST_CM                     20.0f


const AP_Param::GroupInfo ModeLoiterAssisted::var_info[] = {
    // @Param: VEL_MAX
    // @DisplayName: Max velocity commandable by sticks cm/s
    // @Description: Max velocity commandable by sticks cm/s
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("VEL_MAX", 1, ModeLoiterAssisted, _vel_max_cms, VEL_MAX_DEFAULT),

    // @Param: MIN_DIST
    // @DisplayName: Minimum dist to obstacle cm
    // @Description: Minimum distance that you can get to the obstacle cm
    // @Units: cm
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("MIN_DIST", 2, ModeLoiterAssisted, _min_obs_dist_cm, MIN_OBS_DIST_CM_DEFAULT),

    // @Param: YAW_HZ
    // @DisplayName: Yaw LPF alpha
    // @Description: This is the alpha for lpf on yaw [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("YAW_HZ", 3, ModeLoiterAssisted, _yaw_hz, YAW_HZ_DEFAULT),

    // @Param: POS_HZ
    // @DisplayName: Pos LPF alpha
    // @Description: This is the alpha for lpf on dist [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POS_HZ", 4, ModeLoiterAssisted, _pos_filt_hz, POS_HZ_DEFAULT),

    // @Param: DIST_HZ
    // @DisplayName: Dist LPF alpha
    // @Description: This is the alpha for lpf on dist [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DIST_HZ", 5, ModeLoiterAssisted, _dist_filt_hz, DIST_HZ_DEFAULT),

    // @Param: WV_WIND
    // @DisplayName: Window Var window size
    // @Description: min samples for window var estimator to provide variance estimate
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("WV_WIND", 6, ModeLoiterAssisted, _wv_window_size, WV_WIND_DEFAULT),

    // @Param: WV_THRESH
    // @DisplayName: Window Var threshold for good docking
    // @Description: Variance threshold that has to be met to enable docking
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("WV_THRESH", 7, ModeLoiterAssisted, _wv_thresh, WV_THRESH_DEFAULT),

    // @Param: DOCK_SPD
    // @DisplayName: Dock speed cm/s
    // @Description: speed to impact the target at cm/s
    // @Unit: mps
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("DOCK_SPD", 8, ModeLoiterAssisted, _dock_speed_cms, DOCK_SPEED_CMS_DEFAULT),

    // @Param: UNDOCK_SPD
    // @DisplayName: UnDock speed cm/s
    // @Description: speed to escape the target at cm/s
    // @Unit: mps
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("UNDOCK_SPD", 9, ModeLoiterAssisted, _undock_speed_cms, UNDOCK_SPEED_CMS_DEFAULT),

    // @Param{Copter}: _LDR_HZ
    // @DisplayName: Lidar sweep rate in Hz
    // @Description: Lidar sweep rate in Hz
    // @Units: hz
    // @User: Advanced
    AP_GROUPINFO("LDR_HZ", 10, ModeLoiterAssisted, _lidar_sweep_rate_hz, 3.73),

    // @Param{Copter}: CID_M
    // @DisplayName: Coast in distance in cm
    // @Description: Coast in distance in cm
    // @Units: cm
    // @User: Advanced
    AP_GROUPINFO("CID_CM", 11, ModeLoiterAssisted, _coast_in_dist, 100),

    // @Param{Copter}: WUP_DEG
    // @DisplayName: Pitch angle for wind up to target in degrees
    // @Description: Pitch angle for wind up to target in degrees
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("WUP_DEG", 12, ModeLoiterAssisted, _wind_up_pitch_deg, 1),

    AP_GROUPEND
};

ModeLoiterAssisted::ModeLoiterAssisted(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}


// loiter_init - initialise loiter controller
bool ModeLoiterAssisted::init(bool ignore_checks)
{
    // Failsafes
    if (copter.failsafe.radio) {return false;}// TODO what failsafe mode should we use?
    if (!ahrs.get_relative_position_NED_origin(_cur_pos_NED_m)) {return false;}
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    AltHoldModeState alt_hold_state = get_alt_hold_state(target_climb_rate);
    if (!(alt_hold_state == AltHoldModeState::Flying || _flags.ATTACHED)) {return false;}

    // Pos control inits
    if (!pos_control->is_active_z()) {pos_control->init_z_controller();}
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
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
    #if AP_DDS_ENABLED
        if (AP_DDS_Client::attached_state != _flags.ATTACHED) {
            set_attached_status(static_cast<float>(AP_DDS_Client::attached_state));
        }  
    #endif
    // find our current position
    if (!ahrs.get_relative_position_NED_origin(_cur_pos_NED_m)) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"No pos estimate!");
        AbortExit();
    }

    // Update filters (simply check for param changes)
    UpdateFilters();

    // calculate dock's position. compute navigation data
    find_dock_target();

    // evaluate distance flags
    evaluateDistFlags();

    // evaluate rate flags
    evaluateRateFlags();

    // send pilot feedback on flags
    sendFlagFeedback();
    // evaluate transitions
    evaluate_transitions();
    // run flight code for current state
    (this->*_lass_state)(Event::RUN_FLIGHT_CODE);
    // log everything of interest
    logLass();
}

// This function interprets what the pilot is attempting, and returns feedback
void ModeLoiterAssisted::sendFlagFeedback() { 
    if (_flags.ATTACH_BUTTON_PRESSED) { // Pilot wants to engage docking mode
        if (!_flags.DOCK_STABLE) {

        }
    }
}

void ModeLoiterAssisted::logLass() { 
    // convert flags to bitmask
    uint16_t flags_bitmask = 0;
    flags_bitmask |= (_flags.DOCK_FOUND           ? 1 : 0) << 0;
    flags_bitmask |= (_flags.DOCKING_ENGAGED      ? 1 : 0) << 1;
    flags_bitmask |= (_flags.DOCK_STABLE          ? 1 : 0) << 2;
    flags_bitmask |= (_flags.WITHIN_COAST_IN_DIST     ? 1 : 0) << 3;
    flags_bitmask |= (_flags.ATTACHED             ? 1 : 0) << 4;
    flags_bitmask |= (_flags.ACCEL_STATIONARY     ? 1 : 0) << 5;
    flags_bitmask |= (_flags.ATTACH_BUTTON_PRESSED ? 1 : 0) << 6;
    flags_bitmask |= (_flags.STABLE_AT_WIND_UP_PITCH ? 1 : 0) << 7;
    flags_bitmask |= (_flags.AT_RECOVERY_POSITION ? 1 : 0) << 8;
    flags_bitmask |= (_flags.THROTTLE_WOUND_DOWN ? 1 : 0) << 9;
    
    if (millis()-_last_log_time > _log_period_ms) {
        AP::logger().Write(
        "LASS", // heading name
        "TimeUS,dockX,dockY,state,flags", // field labels
        "smm--", // units
        "F00--", // mults
        "QffBH", // format
        AP_HAL::micros64(),
        _filt_dock_xyz_NEU_m.x,
        _filt_dock_xyz_NEU_m.y,
        uint8_t(_lass_state_name),
        flags_bitmask
        );
        _last_log_time = millis();
    }
}

void ModeLoiterAssisted::evaluateDistFlags() { // ALL FLAGS MUST BE SET TO FALSE INITIALLY!
    _flags.WITHIN_COAST_IN_DIST = false;
    _flags.AT_RECOVERY_POSITION = false;
    _flags.BEYOND_COAST_OUT_DIST = false;
    float dist_to_recovery_pos_cm = (_recovery_position_NED_m-_cur_pos_NED_m).length()*100.0f;
    fprintf(stderr, "dist %.2f \n", dist_to_recovery_pos_cm);
    if (dist_to_recovery_pos_cm < RECOVERY_DIST_THRESH_CM) {
        _flags.AT_RECOVERY_POSITION = true;
    }
    float dist_from_docked_pos_cm = (_docked_position_NED_m.xy()-_cur_pos_NED_m.xy()).length()*100.0f;
    if (dist_from_docked_pos_cm > COAST_OUT_DIST_CM) {
        _flags.BEYOND_COAST_OUT_DIST = true;
    }
    if (!_flags.DOCK_FOUND) { return;}
    _dist_to_dock_cm = (_filt_dock_xyz_NEU_m.xy()-_cur_pos_NED_m.xy()).length()*100.0f;
    if (_dist_to_dock_cm < _coast_in_dist) {_flags.WITHIN_COAST_IN_DIST = true;}


}

void ModeLoiterAssisted::evaluateRateFlags() { // ALL FLAGS MUST BE SET TO FALSE INITIALLY!
    _flags.ACCEL_STATIONARY = false;
    _flags.STABLE_AT_WIND_UP_PITCH = false;
    bool accel_stationary = (copter.land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX);
    if (accel_stationary) {_flags.ACCEL_STATIONARY = true;}
    bool at_wind_up_pitch = abs(ahrs.get_pitch()*RAD_TO_DEG - _wind_up_pitch_deg) < WIND_UP_PITCH_TOL;
    if (accel_stationary && at_wind_up_pitch) {
        _flags.STABLE_AT_WIND_UP_PITCH = true;
    }
}

void ModeLoiterAssisted::AbortExit() {
    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"Abort Exiting LASS");
    if (!copter.set_mode(copter.prev_control_mode, ModeReason::UNKNOWN)) {
        // this should never happen but just in case
        copter.set_mode(Mode::Number::STABILIZE, ModeReason::UNKNOWN);
    }
}

void ModeLoiterAssisted::InitFilters() {
    _yaw_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _yaw_hz.get());
    _dist_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _dist_filt_hz.get());
    _dock_pos_filter.set_cutoff_frequency(_lidar_sweep_rate_hz.get(), _pos_filt_hz.get()); 
    _dock_norm_filter.set_cutoff_frequency(_lidar_sweep_rate_hz.get(), _pos_filt_hz.get()); 
    _dock_target_window_var = WindowVar(_wv_window_size.get()); // reinit with new min samples
    _yaw_buf = ModeLoiterAssisted::YawBuffer(); // reinit yaw buffer
}

void ModeLoiterAssisted::UpdateFilters() {
    // check for filter change
    if (!is_equal(_yaw_filter.get_cutoff_freq(), _yaw_hz.get())) {
        _yaw_filter.set_cutoff_frequency(_yaw_hz.get());
    }
    if (!is_equal(_dist_filter.get_cutoff_freq(), _dist_filt_hz.get())) {
        _dist_filter.set_cutoff_frequency(_dist_filt_hz.get());
    }
    if (!is_equal(_dock_pos_filter.get_cutoff_freq(), _pos_filt_hz.get())) { // TODO update to dock_hz
        _dock_pos_filter.set_cutoff_frequency(_pos_filt_hz.get());
    }
    if (!is_equal(_dock_norm_filter.get_cutoff_freq(), _pos_filt_hz.get())) { // TODO update to dock_hz
        _dock_norm_filter.set_cutoff_frequency(_pos_filt_hz.get());
    }
    if (!is_equal(_dock_target_window_var.get_window_size(), _wv_window_size.get())) {
        _dock_target_window_var.set_new_window_size(_wv_window_size.get());
    }
}

void ModeLoiterAssisted::find_dock_target(){
    Vector2f dock_normal_vec;
    Vector2f cfit_center_xy_m;
    _flags.DOCK_FOUND = false; 
    if (!g2.proximity.curvefit->get_target(dock_normal_vec, cfit_center_xy_m)) {return;} // return if we don't find an obstacle
    _flags.DOCK_FOUND = true;

    if (!is_equal(cfit_center_xy_m.x,_last_cfit_center_xy_m_x) || !is_equal(cfit_center_xy_m.y,_last_cfit_center_xy_m_y)){ // when new lidar data comes in
        // Filter and evaluate dock center position
        _last_cfit_center_xy_m_x = cfit_center_xy_m.x;
        _last_cfit_center_xy_m_y = cfit_center_xy_m.y;
        float x = cfit_center_xy_m.x;
        float y = cfit_center_xy_m.y;
        float z = _cur_pos_NED_m.z;
        const Vector3f dock_pos{x,y,z};
        _filt_dock_xyz_NEU_m = _dock_pos_filter.apply(dock_pos); // low pass filter on dock position
        _flags.DOCK_STABLE = false;
        _flags.DOCKING_ENGAGED = false;
        Vector3f dock_target_var;
        if (_dock_target_window_var.apply(dock_pos, dock_target_var)) { // keep track of variance (deviation) of center
            if (dock_target_var.length() < _wv_thresh.get()) {
                _flags.DOCK_STABLE = true;
                if (_flags.ATTACH_BUTTON_PRESSED) {
                    _flags.DOCKING_ENGAGED = true;
                }
            } 
            else {
                _flags.DOCK_STABLE = false;
                if (_flags.ATTACH_BUTTON_PRESSED) {
                    _flags.DOCKING_ENGAGED = false;
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Variance: %.4fm >= %.4fm", dock_target_var.length(), _wv_thresh.get());
                }
            }
        }

        // Filter and evaluate dock normal vector
        _filt_dock_normal_NEU = _dock_norm_filter.apply(dock_normal_vec);
    }
}

// void ModeLoiterAssisted::set_dock_target_state(DockTargetLockState new_state){
//     _docking_target_lock_state = new_state;
// }

/*---------------------------------------------------------------------------*/
/* Finite State Machine States... */
ModeLoiterAssisted::Status ModeLoiterAssisted::Default(const Event e) {
    Status status = Status::HANDLED_STATUS;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:{
        _lass_state_name = StateName::Default;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering Default state");
        _crash_check_enabled = true;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        
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
        float target_climb_rate = 0.0f;
        pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        loiter_nav->update();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false); // call attitude controller
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate); // get avoidance adjusted climb rate
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate); // Send the commanded climb rate to the position controller
        pos_control->update_z_controller();
        
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering Lass state");
        _crash_check_enabled = true;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (!_flags.DOCK_FOUND) {status = TRAN(&ModeLoiterAssisted::Default);}
        else if (_flags.DOCKING_ENGAGED) {status = TRAN(&ModeLoiterAssisted::LeadUp);}
        else if (_flags.ATTACHED) {status = TRAN(&ModeLoiterAssisted::WindDown);}
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        // xy controller... TODO Change to velocity control!
        float filt_heading_cmd_deg = get_bearing_cd(_cur_pos_NED_m.xy(),_filt_dock_xyz_NEU_m.xy())/100.0f;
        // float filt_heading_cmd_deg = atan2f(-_filt_dock_normal_NEU.y,-_filt_dock_normal_NEU.x)*RAD_TO_DEG;
        Vector2f target_xy_body_vel_cms = get_pilot_desired_velocity_xy(_vel_max_cms.get());
        Vector2f target_xy_NEU_vel_cms = target_xy_body_vel_cms;
        target_xy_NEU_vel_cms.rotate(filt_heading_cmd_deg * DEG_TO_RAD);
        Vector2f target_xy_NEU_cm = pos_control->get_pos_target_cm().tofloat().xy() + target_xy_NEU_vel_cms*G_Dt;
        Vector2f target_to_dock_vec_cm = _filt_dock_xyz_NEU_m.xy()*100 - target_xy_NEU_cm;
        float target_to_dock_dist_cm = target_to_dock_vec_cm.length();
        if (target_to_dock_dist_cm < _min_obs_dist_cm.get()) {
            // if too close, adjust to nearest pos that fits min distance condition
            Vector2f min_dist_correction_vec_cm = target_to_dock_vec_cm.normalized()*abs(target_to_dock_dist_cm-_min_obs_dist_cm.get());
            target_xy_NEU_cm -= min_dist_correction_vec_cm;
        }
        pos_control->set_pos_target_xy_cm(target_xy_NEU_cm.x, target_xy_NEU_cm.y);

        // Yaw controller
        AC_AttitudeControl::HeadingCommand heading_cmd;
        heading_cmd.heading_mode = AC_AttitudeControl::HeadingMode::Angle_Only;
        heading_cmd.yaw_angle_cd = filt_heading_cmd_deg*100.0f;
        heading_cmd.yaw_rate_cds = 0.0;

        // z controller
        float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate); // get avoidance adjusted climb rate
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate); // Send the commanded climb rate to the position controller

        // run controllers
        pos_control->update_xy_controller();
        pos_control->update_z_controller();
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading_cmd);
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering LeadUp state");
        #if AP_DDS_ENABLED
        AP_DDS_Client::need_to_pub_attach_detach = true;
        AP_DDS_Client::desire_attach = true;
        #endif
        gcs().send_named_float("attach", 1.0f);
        _crash_check_enabled = true;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        
        float heading_rad = get_bearing_cd(_cur_pos_NED_m.xy(),_filt_dock_xyz_NEU_m.xy())/100.0f*DEG_TO_RAD;
        // float heading_rad = atan2f(-_filt_dock_normal_NEU.y,-_filt_dock_normal_NEU.x);
        _locked_heading_deg = heading_rad*RAD_TO_DEG;
        _locked_vel_NE_cms = Vector2f(cosf(heading_rad),sinf(heading_rad))*_dock_speed_cms;
        _recovery_position_NED_m = _cur_pos_NED_m;
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.WITHIN_COAST_IN_DIST) {status = TRAN(&ModeLoiterAssisted::CoastIn);}
        else if (_flags.ATTACHED) {status = TRAN(&ModeLoiterAssisted::WindDown);}
        else if (!_flags.ATTACH_BUTTON_PRESSED) {status = TRAN(&ModeLoiterAssisted::Recover);}
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        AC_AttitudeControl::HeadingCommand heading_cmd;
        heading_cmd.heading_mode = AC_AttitudeControl::HeadingMode::Angle_Only;
        heading_cmd.yaw_angle_cd = _locked_heading_deg*100.0f;
        heading_cmd.yaw_rate_cds = 0.0;

        pos_control->input_vel_accel_xy(_locked_vel_NE_cms, Vector2f(0,0));
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f); // no climb rate

        pos_control->update_xy_controller();
        pos_control->update_z_controller();
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading_cmd);
        _coast_in_pitch_cd = pos_control->get_pitch_cd();
        _coast_in_pitch_cd = constrain_float(_coast_in_pitch_cd, LOWER_COAST_IN_PITCH_BOUND, UPPER_COAST_IN_PITCH_BOUND); // constrain
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering CoastIn state");
        _crash_check_enabled = false;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.ATTACHED) {status = TRAN(&ModeLoiterAssisted::WindDown);}
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, _coast_in_pitch_cd, 0);
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering WindDown state");
        _crash_check_enabled = false;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        _wind_down_throttle_start = attitude_control->get_throttle_in();
        _wind_down_start_ms = millis();
        
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.ACCEL_STATIONARY && _flags.THROTTLE_WOUND_DOWN) {status = TRAN(&ModeLoiterAssisted::Vegetable);} // are both necessary?
        else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        attitude_control->input_euler_rate_roll_pitch_yaw(0, 0, 0);
        float wind_down_throttle = _wind_down_throttle_start * 
            (1 - (float(millis() - _wind_down_start_ms)/_wind_down_decay_time_s));
        wind_down_throttle = MAX(wind_down_throttle,0.0f);
        attitude_control->set_throttle_out(wind_down_throttle, true, g.throttle_filt);
        if (wind_down_throttle < 0.0001f) {
            _flags.THROTTLE_WOUND_DOWN = true;
        }
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering Vegetable state");
        _crash_check_enabled = false;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        _docked_position_NED_m = _cur_pos_NED_m;
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (!_flags.ATTACH_BUTTON_PRESSED) {status = TRAN(&ModeLoiterAssisted::WindUp);}
        // TODO add falling check
        // else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering WindUp state");
        _crash_check_enabled = false;
        _is_taking_off = true;
        set_land_complete(false);
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        if (abs(_wind_down_throttle_start-0.0f)<0.0001f) {
            _wind_down_throttle_start = motors->get_throttle_hover(); // This could be problematic!
        }
        
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        _is_taking_off = false;
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.STABLE_AT_WIND_UP_PITCH) {status = TRAN(&ModeLoiterAssisted::CoastOut);} 
        else {
            float accel_mss = copter.land_accel_ef_filter.get().length();
            float pitch_deg = ahrs.get_pitch()*RAD_TO_DEG;
            if (millis()-_last_send_windup > 100) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "accel m/s/s: %.2f pitch deg: %.2f", accel_mss, pitch_deg); // TODO remove or rate limit
                _last_send_windup = millis();
            }
            
        }
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        // float roll = ahrs.get_roll()*RAD_TO_DEG*100.0f; // if we are roll constrained, it might be useful to not push it to hard
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, _wind_up_pitch_deg*100.0f, 0.0f);
        attitude_control->set_throttle_out(_wind_down_throttle_start, true, g.throttle_filt); // monitor wind up?
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering CoastOut state");
        _crash_check_enabled = true;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        #if AP_DDS_ENABLED
        // AP_DDS publish detach message
        AP_DDS_Client::need_to_pub_attach_detach = true;
        AP_DDS_Client::desire_attach = false;
        #endif
        gcs().send_named_float("attach", 0.0f);
        
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.BEYOND_COAST_OUT_DIST) {status = TRAN(&ModeLoiterAssisted::Recover);}
        // else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
        pos_control->update_z_controller();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, -_wind_up_pitch_deg*100.0f, 0);
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering Recover state");
        _crash_check_enabled = true;
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        
        break;}
    case Event::EXIT_SIG:{ // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        
        break;}
    case Event::EVALUATE_TRANSITIONS:{
        if (_flags.AT_RECOVERY_POSITION) {status = TRAN(&ModeLoiterAssisted::Lass);}
        // else {}
        break;}
    case Event::RUN_FLIGHT_CODE:{
        // Flight Code
        pos_control->set_pos_target_xy_cm(_recovery_position_NED_m.x*100.0f, _recovery_position_NED_m.y*100.0f);
        pos_control->set_pos_target_z_cm(-_recovery_position_NED_m.z*100.0f);

        // run position controllers
        pos_control->update_xy_controller();
        pos_control->update_z_controller();

        // call attitude controller with auto yaw
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), ahrs.get_yaw()*RAD_TO_DEG*100.0f);
        break;}
    default:{
        fprintf(stderr, "I am in hell");
        
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
void ModeLoiterAssisted::run_flight_code() {
    Status status = (this->*_lass_state)(Event::RUN_FLIGHT_CODE);
    if (status == Status::TRAN_STATUS) {
        //ignore for now
    }
}




/*..........................................................................*/

void ModeLoiterAssisted::attach() { // init attach engaged via RC_Channel
    _flags.ATTACH_BUTTON_PRESSED = true;
}

void ModeLoiterAssisted::detach() { // init detach engaged via RC_Channel
    _flags.ATTACH_BUTTON_PRESSED = false;
}

void ModeLoiterAssisted::set_attached_status(float att_st) { // start being attached
    _flags.ATTACHED = (abs(att_st) > 0.01);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Attached Status: %d\n", _flags.ATTACHED);
}



uint32_t ModeLoiterAssisted::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeLoiterAssisted::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

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
