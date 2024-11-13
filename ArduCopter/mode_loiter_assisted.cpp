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

bool ModeLoiterAssisted::attached_state = false;  // Initialization

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
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);
    if (!(loiter_state == AltHoldModeState::Flying || _attached)) {return false;}

    // Pos control inits
    if (!pos_control->is_active_z()) {pos_control->init_z_controller();}
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    copter.set_simple_mode(Copter::SimpleMode::NONE); // disable simple mode for this mode. TODO: Validate

    // LASS Inits
    _yaw_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _yaw_hz.get());
    _dist_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _dist_filt_hz.get());
    _dock_pos_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _pos_filt_hz.get()); // TODO: use custom dock hz param
    _dock_target_window_var = WindowVar(_wv_window_size.get()); // reinit with new min samples
    _yaw_buf = ModeLoiterAssisted::YawBuffer(); // reinit yaw buffer

    // Set State (assuming not attached for now)
    set_dock_target_state(DockTargetLockState::NOT_FOUND);
    _external_event = Event::ENTRY_SIG; // set external event to entry so entry is performed on the state

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode set to Loiter Assisted");
    return true;
}

// should be called at 100hz or more
void ModeLoiterAssisted::run()
{
    evaluate_transitions(_external_event); /* NO BLOCKING! */
}

void ModeLoiterAssisted::set_dock_target_state(DockTargetLockState new_state){
    _docking_target_lock_state = new_state;
}

/*---------------------------------------------------------------------------*/
/* Finite State Machine States... */
ModeLoiterAssisted::Status ModeLoiterAssisted::Default(const Event e) {
    Status status;
    // Transitions
    switch (e) {
    case Event::ENTRY_SIG:
        _crash_check_enabled = true;
        status = Status::HANDLED_STATUS;
        break;
    case Event::EXIT_SIG: // exit must return so flight code doesn't get run (maybe split into run transitions and run actions?)
        status = Status::HANDLED_STATUS;
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

/*---------------------------------------------------------------------------*/
/* Finite State Machine facilities... */
void ModeLoiterAssisted::evaluate_transitions(const Event e) {
    Status status;
    StateHandler prev_state = _lass_state; /* save for later */

    status = (this->*_lass_state)(e);

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

bool ModeLoiterAssisted::attach() { // init attach engaged via RC_Channel
    if (_docking_state == DockingState::ATTACH_MANEUVER) {
        return true;
    }
    if (!_ready_to_dock) {
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Variance too high, not ready to dock");
        return false;
    }
    #if AP_DDS_ENABLED
    // AP_DDS publish attach message... should change to state setup?
    AP_DDS_Client::need_to_pub_attach_detach = true;
    AP_DDS_Client::desire_attach = true;
    #endif
    gcs().send_named_float("attach", 1.0f);
    _crash_check_enabled = false;
    _docking_state = DockingState::ATTACH_MANEUVER;
    _lock_commands = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Attach primed");
    return true;
}

bool ModeLoiterAssisted::detach() { // init detach engaged via RC_Channel
    if (_docking_state == DockingState::DETACH_MANEUVER) {
        return true;
    }
    if (_docking_state == DockingState::NOT_DOCKING) { // only detach when we are attaching or attached
        return false;
    }
    #if AP_DDS_ENABLED
    // AP_DDS publish detach message
    AP_DDS_Client::need_to_pub_attach_detach = true;
    AP_DDS_Client::desire_attach = false;
    #endif
    gcs().send_named_float("attach", 0.0f);


    _docking_state = DockingState::DETACH_MANEUVER;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Detach primed");
    return true;
}

bool ModeLoiterAssisted::attached() { // start being attached
    if (_docking_state == DockingState::ATTACHED) {
        return true;
    }

    _docking_state = DockingState::ATTACHED;
    _attached = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Attached!");
    return true;
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
