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
    TRAN(ModeLoiterAssisted::Default);
    (this->*_lass_state)(Event::ENTRY_SIG); // perform entry actions

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode set to Loiter Assisted");
    return true;
}

// should be called at 100hz or more
void ModeLoiterAssisted::run()
{
    // find our current position
    if (!ahrs.get_relative_position_NED_origin(_cur_pos_NED_m)) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"No pos estimate!");
        AbortExit();
    }

    // Update filters (simply check for param changes)
    UpdateFilters();

    

    // calculate dock's position. compute navigation data
    find_dock_target();

    // evaluate transitions
    evaluate_transitions();

    // run flight code for current state
    (this->*_lass_state)(Event::RUN_FLIGHT_CODE);

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
        Vector3f dock_target_var;
        if (_dock_target_window_var.apply(dock_pos, dock_target_var)) { // keep track of variance (deviation) of center
            if (dock_target_var.length() < _wv_thresh.get()) {
                _flags.DOCK_STABLE = true;
            }
        }

        // Filter and evaluate dock normal vector
        _filt_dock_normal_NEU = _dock_norm_filter.apply(dock_normal_vec);

    }
    // _filt_heading_cmd_deg = get_bearing_cd(_cur_pos_NED_m.xy(),_filt_dock_xyz_NEU_m.xy())/100.0f;
    _filt_heading_cmd_deg = atan2f(dock_normal_vec.y,dock_normal_vec.x);
}

// void ModeLoiterAssisted::set_dock_target_state(DockTargetLockState new_state){
//     _docking_target_lock_state = new_state;
// }

/*---------------------------------------------------------------------------*/
/* Finite State Machine States... */
ModeLoiterAssisted::Status ModeLoiterAssisted::Default(const Event e) {
    Status status;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:
        _lass_state_name = StateName::Default;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering Default state");
        _crash_check_enabled = true;
        status = Status::HANDLED_STATUS;
        break;
    case Event::EXIT_SIG: // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        status = Status::HANDLED_STATUS;
        break;
    case Event::EVALUATE_TRANSITIONS:
        if (_flags.DOCK_FOUND) {status = TRAN(ModeLoiterAssisted::Lass);}
        else {status = Status::HANDLED_STATUS;}
        break;
    case Event::RUN_FLIGHT_CODE:
        // Flight Code
        loiter_nav->update();
        float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false); // call attitude controller
        float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate); // get avoidance adjusted climb rate
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate); // Send the commanded climb rate to the position controller
        pos_control->update_z_controller();
        status = Status::HANDLED_STATUS;
        break;
    default:
        fprintf(stderr, "I am in hell");
        status = Status::HANDLED_STATUS;
        break;
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::Lass(const Event e) {
    Status status;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:
        _lass_state_name = StateName::Lass;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering Lass state");
        _crash_check_enabled = true;
        status = Status::HANDLED_STATUS;
        break;
    case Event::EXIT_SIG: // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        status = Status::HANDLED_STATUS;
        break;
    case Event::EVALUATE_TRANSITIONS:
        if (!_flags.DOCK_FOUND) {status = TRAN(ModeLoiterAssisted::Default);}
        else if (_flags.DOCKING_ENGAGED) {status = TRAN(ModeLoiterAssisted::LeadUp);}
        else {status = Status::HANDLED_STATUS;}
        break;
    case Event::RUN_FLIGHT_CODE:
        // Flight Code
        // xy controller... TODO Change to velocity control!
        Vector2f target_xy_body_vel_cms = get_pilot_desired_velocity_xy(_vel_max_cms.get());
        Vector2f target_xy_NEU_vel_cms = target_xy_body_vel_cms;
        target_xy_NEU_vel_cms.rotate(_filt_heading_cmd_deg * DEG_TO_RAD);
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
        heading_cmd.yaw_angle_cd = _filt_heading_cmd_deg*100.0f;
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
        
        break;
    default:
        fprintf(stderr, "I am in hell");
        status = Status::HANDLED_STATUS;
        break;
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::LeadUp(const Event e) {
    Status status;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:
        _lass_state_name = StateName::LeadUp;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering LeadUp state");
        #if AP_DDS_ENABLED
        AP_DDS_Client::need_to_pub_attach_detach = true;
        AP_DDS_Client::desire_attach = true;
        #endif
        gcs().send_named_float("attach", 1.0f);
        _crash_check_enabled = true;
        status = Status::HANDLED_STATUS;
        break;
    case Event::EXIT_SIG: // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        status = Status::HANDLED_STATUS;
        break;
    case Event::EVALUATE_TRANSITIONS:
        if (_flags.AT_COAST_IN_DIST) {status = TRAN(ModeLoiterAssisted::CoastIn);}
        else {status = Status::HANDLED_STATUS;}
        break;
    case Event::RUN_FLIGHT_CODE:
        // Flight Code
        pos_control->input_pos_vel_accel_xy(_xy_vel_cms, _xy_accel);
        // // pos_control->input_vel_accel_z(_z_vel, _z_accel);
        // pos_control->set_vel_desired_xy_cms(_xy_vel_cms);
        // // pos_control->set_accel_desired_xy_cmss(_xy_accel);
        // pos_control->set_vel_desired_z_cms(_z_vel);
        filt_heading_cmd_deg = wrap_180(_bearing_cd/100.0f);
        // integrate current pos to new target pos
        Vector2f target_xy_NEU_cm = pos_control->get_pos_target_cm().tofloat().xy() + _xy_vel_cms*dt;
        pos_control->set_pos_target_xy_cm(target_xy_NEU_cm.x, target_xy_NEU_cm.y);

        break;
    default:
        fprintf(stderr, "I am in hell");
        status = Status::HANDLED_STATUS;
        break;
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::CoastIn(const Event e) {
    Status status;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:
        _lass_state_name = StateName::CoastIn;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering CoastIn state");
        _crash_check_enabled = false;
        status = Status::HANDLED_STATUS;
        break;
    case Event::EXIT_SIG: // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        status = Status::HANDLED_STATUS;
        break;
    case Event::EVALUATE_TRANSITIONS:
        if (_flags.ATTACHED) {status = TRAN(ModeLoiterAssisted::WindDown);}
        else {status = Status::HANDLED_STATUS;}
        break;
    case Event::RUN_FLIGHT_CODE:
        // Flight Code
        
        break;
    default:
        fprintf(stderr, "I am in hell");
        status = Status::HANDLED_STATUS;
        break;
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::WindDown(const Event e) {
    Status status;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:
        _lass_state_name = StateName::WindDown;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering WindDown state");
        _crash_check_enabled = false;
        status = Status::HANDLED_STATUS;
        break;
    case Event::EXIT_SIG: // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        status = Status::HANDLED_STATUS;
        break;
    case Event::EVALUATE_TRANSITIONS:
        if (_flags.WINDED_DOWN) {status = TRAN(ModeLoiterAssisted::Vegetable);}
        else {status = Status::HANDLED_STATUS;}
        break;
    case Event::RUN_FLIGHT_CODE:
        // Flight Code
        
        break;
    default:
        fprintf(stderr, "I am in hell");
        status = Status::HANDLED_STATUS;
        break;
    }
    return status;
}

ModeLoiterAssisted::Status ModeLoiterAssisted::Vegetable(const Event e) {
    Status status;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:
        _lass_state_name = StateName::Vegetable;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LASS: Entering Vegetable state");
        _crash_check_enabled = false;
        status = Status::HANDLED_STATUS;
        break;
    case Event::EXIT_SIG: // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        status = Status::HANDLED_STATUS;
        break;
    case Event::EVALUATE_TRANSITIONS:
        // if (_flags.WINDED_DOWN) {status = TRAN(ModeLoiterAssisted::Vegetable);}
        // else {status = Status::HANDLED_STATUS;}
        break;
    case Event::RUN_FLIGHT_CODE:
        // Flight Code
        
        break;
    default:
        fprintf(stderr, "I am in hell");
        status = Status::HANDLED_STATUS;
        break;
    }
    return status;
}


/*---------------------------------------------------------------------------*/
/* Finite State Machine facilities... */
void ModeLoiterAssisted::evaluate_transitions() {
    Status status;
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
    // if (_flags.ATTACH_BUTTON_PRESSED) {
    //     return true;
    // }
    // if (!_ready_to_dock) {
    //     GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Variance too high, not ready to dock");
    //     return false;
    // }

    // _docking_state = DockingState::ATTACH_MANEUVER;
    // _lock_commands = true;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Attach primed");
    // return true;
}

void ModeLoiterAssisted::detach() { // init detach engaged via RC_Channel
    _flags.ATTACH_BUTTON_PRESSED = false;
    // if (_docking_state == DockingState::DETACH_MANEUVER) {
    //     return true;
    // }
    // if (_docking_state == DockingState::NOT_DOCKING) { // only detach when we are attaching or attached
    //     return false;
    // }
    // #if AP_DDS_ENABLED
    // // AP_DDS publish detach message
    // AP_DDS_Client::need_to_pub_attach_detach = true;
    // AP_DDS_Client::desire_attach = false;
    // #endif
    // gcs().send_named_float("attach", 0.0f);


    // _docking_state = DockingState::DETACH_MANEUVER;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Detach primed");
    // return true;
}

void ModeLoiterAssisted::set_attached_status(float att_st) { // start being attached
    _flags.ATTACHED = (abs(att_st) > 0.01);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Attached Status: %d\n", _flags.ATTACHED);
    // if (_docking_state == DockingState::ATTACHED) {
    //     return true;
    // }   

    // _docking_state = DockingState::ATTACHED;
    // _attached = true;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Attached!");
    // return true;
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
