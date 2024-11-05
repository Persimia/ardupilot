#include "Copter.h"

#if AP_DDS_ENABLED
#include <AP_DDS/AP_DDS_Client.h>
#endif

#if MODE_LOITER_ASSISTED_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

#define PITCH_TO_FW_VEL_GAIN_DEFAULT         0.1
#define ROLL_TO_RT_VEL_GAIN_DEFAULT          0.1
#define MIN_OBS_DIST_CM_DEFAULT              100
#define YAW_HZ_DEFAULT                       100.0
#define DIST_HZ_DEFAULT                      100.0
#define POS_HZ_DEFAULT                       100.0
#define WV_WIND_DEFAULT                      5
#define WV_THRESH_DEFAULT                    0.1 
#define DOCK_SPEED_MPS_DEFAULT               0.1
#define UNDOCK_SPEED_MPS_DEFAULT             0.5

bool ModeLoiterAssisted::attached_state = false;  // Initialization

const AP_Param::GroupInfo ModeLoiterAssisted::var_info[] = {
    // @Param: P_2_FW_VEL
    // @DisplayName: Pitch to perpendicular vel
    // @Description: Gain from pitch angle to perpendicular velocity
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("P_2_FW_VEL", 1, ModeLoiterAssisted, _pitch_to_fw_vel_gain, PITCH_TO_FW_VEL_GAIN_DEFAULT),

    // @Param: R_2_RT_VEL
    // @DisplayName: Roll to lateral vel
    // @Description: Gain from roll angle to lateral velocity
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("R_2_RT_VEL", 2, ModeLoiterAssisted, _roll_to_rt_vel_gain, ROLL_TO_RT_VEL_GAIN_DEFAULT),

    // @Param: MIN_DIST
    // @DisplayName: Minimum dist to obstacle
    // @Description: Minimum distance that you can get to the obstacle
    // @Units: cm
    // @Range: 0 5000
    // @User: Advanced
    AP_GROUPINFO("MIN_DIST", 3, ModeLoiterAssisted, _min_obs_dist_cm, MIN_OBS_DIST_CM_DEFAULT),

    // @Param: YAW_HZ
    // @DisplayName: Yaw LPF alpha
    // @Description: This is the alpha for lpf on yaw [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("YAW_HZ", 4, ModeLoiterAssisted, _yaw_hz, YAW_HZ_DEFAULT),

    // @Param: POS_HZ
    // @DisplayName: Pos LPF alpha
    // @Description: This is the alpha for lpf on dist [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POS_HZ", 5, ModeLoiterAssisted, _pos_filt_hz, POS_HZ_DEFAULT),

    // @Param: DIST_HZ
    // @DisplayName: Dist LPF alpha
    // @Description: This is the alpha for lpf on dist [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DIST_HZ", 6, ModeLoiterAssisted, _dist_filt_hz, DIST_HZ_DEFAULT),

    // @Param: WV_WIND
    // @DisplayName: Window Var window size
    // @Description: min samples for window var estimator to provide variance estimate
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("WV_WIND", 7, ModeLoiterAssisted, _wv_window_size, WV_WIND_DEFAULT),

    // @Param: WV_THRESH
    // @DisplayName: Window Var threshold for good docking
    // @Description: Variance threshold that has to be met to enable docking
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("WV_THRESH", 8, ModeLoiterAssisted, _wv_thresh, WV_THRESH_DEFAULT),

    // @Param: DOCK_SPD
    // @DisplayName: Dock speed
    // @Description: speed to impact the target at
    // @Unit: mps
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("DOCK_SPD", 9, ModeLoiterAssisted, _dock_speed_mps, DOCK_SPEED_MPS_DEFAULT),

    // @Param: UNDOCK_SPD
    // @DisplayName: UnDock speed
    // @Description: speed to escape the target at
    // @Unit: mps
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("UNDOCK_SPD", 10, ModeLoiterAssisted, _undock_speed_mps, UNDOCK_SPEED_MPS_DEFAULT),

    AP_GROUPEND
};

ModeLoiterAssisted::ModeLoiterAssisted(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}


// loiter_init - initialise loiter controller
bool ModeLoiterAssisted::init(bool ignore_checks)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mode set to Loiter Assisted");

    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    _target_acquired = false;

    copter.set_simple_mode(Copter::SimpleMode::NONE); // disable simple mode for this mode. TODO: Validate

    _crash_check_enabled = true;
    _docking_state = DockingState::NOT_DOCKING;
    _yaw_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _yaw_hz.get());
    _dist_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _dist_filt_hz.get());
    _dock_target_pos_filter.set_cutoff_frequency(copter.scheduler.get_loop_rate_hz(), _pos_filt_hz.get()); // TODO: use custom dock hz param
    _dock_target_window_var = WindowVar(_wv_window_size.get()); // reinit with new min samples
    _yaw_buf = ModeLoiterAssisted::YawBuffer(); // reinit yaw buffer
    _time_since_last_yaw = millis();
    _init_time = millis();
    _lock_commands = false;
#if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
#endif

    return true;
}

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
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Attached!");
    return true;
}


#if AC_PRECLAND_ENABLED
bool ModeLoiterAssisted::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeLoiterAssisted::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    pos_control->update_xy_controller();
}
#endif

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeLoiterAssisted::run()
{
    // float delete_me_1;
    // float delete_me_2;
    // Vector2f delete_me_3; 
    // Vector2f delete_me_4;
    // Vector2f delete_me_5;
    // bool test = g2.proximity.curvefit->get_target(delete_me_1, delete_me_2, delete_me_3, delete_me_4, delete_me_5);
    // if (!test){
    //     // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "No Obs")
    // }


    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // check for filter change
    if (!is_equal(_yaw_filter.get_cutoff_freq(), _yaw_hz.get())) {
        _yaw_filter.set_cutoff_frequency(_yaw_hz.get());
    }
    if (!is_equal(_dist_filter.get_cutoff_freq(), _dist_filt_hz.get())) {
        _dist_filter.set_cutoff_frequency(_dist_filt_hz.get());
    }
    if (!is_equal(_dock_target_pos_filter.get_cutoff_freq(), _pos_filt_hz.get())) { // TODO update to dock_hz
        _dock_target_pos_filter.set_cutoff_frequency(_pos_filt_hz.get());
    }
    if (!is_equal(_dock_target_window_var.get_window_size(), _wv_window_size.get())) {
        _dock_target_window_var.set_new_window_size(_wv_window_size.get());
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        // update_simple_mode(); //Disable simple mode

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {
        case AltHoldModeState::MotorStopped:
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->reset_yaw_target_and_rate();
            pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
            loiter_nav->init_target();
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            _target_acquired = false;
            break;

        case AltHoldModeState::Landed_Ground_Idle:
            attitude_control->reset_yaw_target_and_rate();
            FALLTHROUGH;

        case AltHoldModeState::Landed_Pre_Takeoff:
            attitude_control->reset_rate_controller_I_terms_smoothly();
            loiter_nav->init_target();
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
            _target_acquired = false;
            break;

        case AltHoldModeState::Takeoff:
            // initiate take-off
            if (!takeoff.running()) {
                takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            }
            // get avoidance adjusted climb rate
            target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
            // set position controller targets adjusted for pilot input
            takeoff.do_pilot_takeoff(target_climb_rate);
            // run loiter controller
            loiter_nav->update();
            // call attitude controller
            attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
            _target_acquired = false;
            break;

        case AltHoldModeState::Flying: {
            // set motors to full range
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

            // Get Yaw angle and distance to closest object from AP_Proximity
            float surface_heading_rad;
            float surface_distance_m;
            Vector2f surface_tangent_vec; 
            Vector2f surface_normal_vec;
            Vector2f surface_center_coords;

            bool found_obstacle = g2.proximity.curvefit->get_target(surface_heading_rad, surface_distance_m, surface_tangent_vec, surface_normal_vec, surface_center_coords);

            if (found_obstacle) { // only perform obstacle stuff when obstacles are in, otherwise do regular loiter
                float heading_to_obs_deg = wrap_180(surface_heading_rad*RAD_TO_DEG);
                float dist_to_obs_m = surface_distance_m;

                if(!_target_acquired){// Reacquired target so reset yaw. TODO manage this state better
                    _distance_target_cm = dist_to_obs_m * 100.0;
                    _target_acquired = true; //TODO control this more clearly
                    _yaw_filter.reset();
                    _dist_filter.reset();
                    _dock_target_pos_filter.reset();
                    _dock_target_window_var.reset();
                }

                if (!ahrs.get_relative_position_NED_origin(_current_vehicle_position)){
                    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY,"No pos estimate!");
                    return; // TODO: What do we even do here? Switch to stabilize?
                }

                if (!is_equal(surface_center_coords.x,_last_surface_center_coords_x) || !is_equal(surface_center_coords.y,_last_surface_center_coords_y)){ // when new lidar data comes in
                    _last_surface_center_coords_x = surface_center_coords.x;
                    _last_surface_center_coords_y = surface_center_coords.y;

                    // // Start tracking global position of nearest point
                    float x = surface_center_coords.x;
                    float y = surface_center_coords.y;
                    float z = _current_vehicle_position.z;
                    const Vector3f dock_target_pos{x,y,z};

                    _filt_dock_target_pos = _dock_target_pos_filter.apply(dock_target_pos); // this is currently filtering something that is already filtered by curvefit (TODO Fix)
                    
                    _ready_to_dock = false;
                    Vector3f dock_target_var;
                    if (_dock_target_window_var.apply(dock_target_pos, dock_target_var)) {
                        if (dock_target_var.length() < _wv_thresh) {
                            _ready_to_dock = true;
                        }
                    }
                }

                float filt_heading_cmd_deg = _yaw_filter.apply(heading_to_obs_deg);
                float filt_dist_to_obs_m = _dist_filter.apply(dist_to_obs_m);
                
                float cos_heading_obs = cosf(filt_heading_cmd_deg * DEG_TO_RAD);
                float sin_heading_obs = sinf(filt_heading_cmd_deg * DEG_TO_RAD);

                Vector2f vec_to_target_2d_m = _filt_dock_target_pos.xy() - _current_vehicle_position.xy();

                // Update target information for attach()
                if (!_lock_commands){
                    _xy_vel = vec_to_target_2d_m.normalized()*_dock_speed_mps.get()*100.0f;
                    _bearing_cd = get_bearing_cd(_current_vehicle_position.xy(), _filt_dock_target_pos.xy());
                }
                Vector2f reverse_vel = _xy_vel.normalized()*-_undock_speed_mps.get()*100.0f; // convert m/s to cm/s
                float reverse_vel_z = ((0.0<_z_vel)-(_z_vel<0.0))*-_undock_speed_mps.get()*100.0f;

                // Docking state controller
                switch (_docking_state){
                    case DockingState::ATTACH_MANEUVER: 
                        pos_control->input_vel_accel_xy(_xy_vel, _xy_accel);
                        pos_control->input_vel_accel_z(_z_vel, _z_accel);
                        filt_heading_cmd_deg = wrap_180(_bearing_cd/100.0f);
                        if(ModeLoiterAssisted::attached_state) { //signal from sensor
                            attached();
                        }
                        break;

                    case DockingState::DETACH_MANEUVER:
                        pos_control->input_vel_accel_xy(reverse_vel, _xy_accel); // back up the way we came in
                        pos_control->input_vel_accel_z(reverse_vel_z, _z_accel);
                        if (filt_dist_to_obs_m * 100.0 >= _min_obs_dist_cm) {
                            init(false); //reinitialize loiter controller
                        }
                        break;

                    case DockingState::ATTACHED:
                        if(!ModeLoiterAssisted::attached_state) { //this is bad news if we become unattached without manually unattaching!
                            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Unexpected detach!");
                            _docking_state = DockingState::DETACH_MANEUVER;
                        }
                        pos_control->relax_velocity_controller_xy();
                        pos_control->relax_z_controller(0.0f);
                        return;
                        break;

                    case DockingState::NOT_DOCKING:
                        FALLTHROUGH;

                    default:
                        float vel_fw = -_pitch_to_fw_vel_gain * target_pitch; //Forward Velocity Command 
                        float vel_rt =  _roll_to_rt_vel_gain * target_roll; //Right Velocity Command 
                        _distance_target_cm -= vel_fw*pos_control->get_dt();
                        if (_distance_target_cm < _min_obs_dist_cm) 
                        {
                            _distance_target_cm = _min_obs_dist_cm;
                        }

                        float distance_err_cm = filt_dist_to_obs_m * 100.0 - _distance_target_cm;
                        Vector2p dist_correction(
                            distance_err_cm*cos_heading_obs,
                            distance_err_cm*sin_heading_obs
                        );
                        Vector2p target_pos = pos_control->get_pos_target_cm().xy();
                        target_pos += dist_correction;
                        Vector2f target_vel(
                            -vel_rt*sin_heading_obs,
                            vel_rt*cos_heading_obs
                        );
                        pos_control->input_pos_vel_accel_xy(target_pos, target_vel, Vector2f(0,0)); // input pos and vel targets
                        break;
                }

                // YAW CONTROLLER //
                AC_AttitudeControl::HeadingCommand heading_cmd;
                heading_cmd.heading_mode = AC_AttitudeControl::HeadingMode::Angle_Only;
                heading_cmd.yaw_angle_cd = filt_heading_cmd_deg*100.0f;
                heading_cmd.yaw_rate_cds = 0.0;
                // END YAW CONTROLLER //

                pos_control->update_xy_controller(); // run pos controller
                // AUGMENT CONTROL //
                // attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
                attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), heading_cmd);
                // get avoidance adjusted climb rate
                #if AP_RANGEFINDER_ENABLED
                // update the vertical offset based on the surface measurement
                copter.surface_tracking.update_surface_offset();
                #endif
                // Send the commanded climb rate to the position controller
                pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
                // END AUGMENT CONTROL //

            } else { //default to normal loiter when no obstacles are detected
                _target_acquired = false; // no target found
                #if AC_PRECLAND_ENABLED
                bool precision_loiter_old_state = _precision_loiter_active;
                if (do_precision_loiter()) {
                    precision_loiter_xy();
                    _precision_loiter_active = true;
                } else {
                    _precision_loiter_active = false;
                }
                if (precision_loiter_old_state && !_precision_loiter_active) {
                    // prec loiter was active, not any more, let's init again as user takes control
                    loiter_nav->init_target();
                }
                // run loiter controller if we are not doing prec loiter
                if (!_precision_loiter_active) {
                    loiter_nav->update();
                }
                #else
                loiter_nav->update();
                #endif

                // call attitude controller
                attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

                // get avoidance adjusted climb rate
                target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

                #if AP_RANGEFINDER_ENABLED
                // update the vertical offset based on the surface measurement
                copter.surface_tracking.update_surface_offset();
                #endif

                // Send the commanded climb rate to the position controller
                pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
            }
            break;}
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"Undefined Loiter State");
            break;
    }
    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
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
