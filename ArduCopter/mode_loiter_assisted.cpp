#include "Copter.h"

#if AP_DDS_ENABLED
#include <AP_DDS/AP_DDS_Client.h>
#endif

#if MODE_LOITER_ASSISTED_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

#define PITCH_TO_FW_VEL_GAIN_DEFAULT         0.2
#define ROLL_TO_RT_VEL_GAIN_DEFAULT          0.2
#define MIN_OBS_DIST_CM_DEFAULT              200
#define YAW_HZ_DEFAULT                       400.0
#define DIST_HZ_DEFAULT                      400.0
#define POS_HZ_DEFAULT                       400.0
#define YAW_DEADZONE_DEFAULT                 10.0
#define WV_WIND_DEFAULT                      5
#define WV_THRESH_DEFAULT                    0.1 
#define DOCK_SPEED_MPS_DEFAULT               0.2
#define UNDOCK_SPEED_MPS_DEFAULT             1.0
#define MTN_CMP_MS_DEFAULT                   50


#define DOCK_TARGET_DIST_CM                  0.0

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

    // @Param: YAW_DZ
    // @DisplayName: Yaw controller deadzone
    // @Description: Deadzone in degrees where the yaw controller won't update
    // @Units: deg
    // @Range: 0 180
    // @User: Advanced
    AP_GROUPINFO("YAW_DZ", 5, ModeLoiterAssisted, _yaw_dz, YAW_DEADZONE_DEFAULT),

    // @Param: POS_HZ
    // @DisplayName: Pos LPF alpha
    // @Description: This is the alpha for lpf on dist [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("POS_HZ", 6, ModeLoiterAssisted, _pos_filt_hz, POS_HZ_DEFAULT),

    // @Param: DIST_HZ
    // @DisplayName: Dist LPF alpha
    // @Description: This is the alpha for lpf on dist [x*a + y*(a-1)]
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("DIST_HZ", 7, ModeLoiterAssisted, _dist_filt_hz, DIST_HZ_DEFAULT),

    // @Param: WV_WIND
    // @DisplayName: Window Var window size
    // @Description: min samples for window var estimator to provide variance estimate
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("WV_WIND", 8, ModeLoiterAssisted, _wv_window_size, WV_WIND_DEFAULT),

    // @Param: WV_THRESH
    // @DisplayName: Window Var threshold for good docking
    // @Description: Variance threshold that has to be met to enable docking
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("WV_THRESH", 9, ModeLoiterAssisted, _wv_thresh, WV_THRESH_DEFAULT),

    // @Param: DOCK_SPD
    // @DisplayName: Dock speed
    // @Description: speed to impact the target at
    // @Unit: mps
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("DOCK_SPD", 10, ModeLoiterAssisted, _dock_speed_mps, DOCK_SPEED_MPS_DEFAULT),

    // @Param: UNDOCK_SPD
    // @DisplayName: UnDock speed
    // @Description: speed to escape the target at
    // @Unit: mps
    // @Range: 0 1000
    // @User: Advanced
    AP_GROUPINFO("UNDOCK_SPD", 11, ModeLoiterAssisted, _undock_speed_mps, UNDOCK_SPEED_MPS_DEFAULT),

    // @Param: MTN_CMP_MS
    // @DisplayName: Motion compensation delay
    // @Description: ms delay between when lidar arrives and when heading is computed
    // @Unit: ms
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("MTN_CMP_MS", 12, ModeLoiterAssisted, _mtn_cmp_ms, MTN_CMP_MS_DEFAULT),

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
    // // Set auto yaw... auto yaw is used in auto modes normally, so we shouldn't do that here
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    _last_yaw_deg = ahrs.get_yaw()*RAD_TO_DEG;

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
    // // TODO REMOVE
    // float surface_heading_rad;
    // float surface_distance_m;
    // Vector2f surface_tangent_vec; 
    // Vector2f surface_normal_vec;
    // Vector2f surface_center_coords;

    // bool test = g2.proximity.curvefit->get_target(surface_heading_rad, surface_distance_m, surface_tangent_vec, surface_normal_vec, surface_center_coords);
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
            
            // // Fallback to prox library get_closest_object
            // if (!found_obstacle) {
            //     found_obstacle = g2.proximity.get_closest_object(yaw_to_obs_deg, dist_to_obs_m);
            // } else {
            //     yaw_to_obs_deg = (targ_heading_rad - ahrs.get_yaw())*RAD_TO_DEG;
            // }

            // !!!!!!!!!!!!!!!!!!! TODO FIX FOR AHRS.GET_YAW() MISHAP !!!!!!!!!!!!!!!!!
            // hgjfkghjdfkghdfk
            // if (millis()-_time_since_last_yaw > 1) {
            //     _yaw_buf.addYaw(ahrs.get_yaw()); // add the current yaw to the buffer
            //     _time_since_last_yaw = millis();
            // }
            // float delayed_yaw = 0.0f; 
            // if (_yaw_buf.getDelayedYaw(_mtn_cmp_ms.get(), delayed_yaw)) {
            //     yaw_to_obs_deg += (delayed_yaw - ahrs.get_yaw()); // get the adjusted offset
            // }
            // !!!!!!!!!!!!!!!!!!! TODO FIX FOR AHRS.GET_YAW() MISHAP !!!!!!!!!!!!!!!!!

            float yaw_to_obs_deg = wrap_180((surface_heading_rad-ahrs.get_yaw())*RAD_TO_DEG);
            float dist_to_obs_m = surface_distance_m;
            
            if (found_obstacle) { // only perform obstacle stuff when obstacles are in, otherwise do regular loiter
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
                // TODO Reevaluate if this is necessary ===================================
                // propogate vehicle movements to yaw estimate
                float delta_yaw = ahrs.get_yaw()*RAD_TO_DEG - _last_yaw_deg;
                float filt_yaw_cmd_deg = _last_yaw_cmd_deg - delta_yaw; // subtract change in yaw from vehicle motion

                // TODO Reevaluate if this is necessary ===================================
                // propogate vehicle movements to dist estimate
                float heading_obs_rad = ahrs.get_yaw() + filt_yaw_cmd_deg*DEG_TO_RAD;
                float cos_yaw_obs = cosf(heading_obs_rad);
                float sin_yaw_obs = sinf(heading_obs_rad);
                Vector2f delta_pos = _current_vehicle_position.xy() - _last_vehicle_pos;
                float delta_dist = delta_pos.dot(Vector2f(cos_yaw_obs,sin_yaw_obs));
                float filt_dist_to_obs_m = _last_dist_to_obs_m - delta_dist; // subtract change in motion from vehicle
                // ::fprintf(stderr,"yaw: %f\tdyaw: %f\tdist: %f\t ddsit: %f\n",filt_yaw_cmd_deg,delta_yaw,filt_dist_to_obs_m,delta_dist);
                
                // TODO Reevaluate if this is necessary ===================================
                // new data in, compute absolute position of obstacle
                if (!is_equal(yaw_to_obs_deg,_last_yaw_to_obs_deg) || !is_equal(dist_to_obs_m,_last_dist_to_obs_m)){ // when new lidar data comes in
                    _last_yaw_to_obs_deg = yaw_to_obs_deg;
                    _last_dist_to_obs_m = dist_to_obs_m;

                    // // Start tracking global position of nearest point
                    // float pitch_rad = ahrs.get_pitch(); // replace with pitch of sensor
                    // float yaw_rad = yaw_to_obs_deg*DEG_TO_RAD;

                    // // Calculate x, y, z using spherical to Cartesian conversion
                    // float x = dist_to_obs_m * cosf(pitch_rad) * cosf(yaw_rad);
                    // float y = dist_to_obs_m * cosf(pitch_rad) * sinf(yaw_rad);
                    // float z = dist_to_obs_m * sinf(pitch_rad);
                    // const Vector3f dock_target_vec{x,y,z};
                    float x = surface_center_coords.x;
                    float y = surface_center_coords.y;
                    float z = _current_vehicle_position.z;
                    const Vector3f dock_target_vec{x,y,z};

                    Vector3f dock_target_pos = ahrs.body_to_earth(dock_target_vec) + _current_vehicle_position; // this is the NED target pos relative to EKF origin
                    _filt_dock_target_pos = _dock_target_pos_filter.apply(dock_target_pos);
                    
                    _ready_to_dock = false;
                    Vector3f dock_target_var;
                    if (_dock_target_window_var.apply(dock_target_pos, dock_target_var)) {
                        if (dock_target_var.length() < _wv_thresh) {
                            _ready_to_dock = true;
                        }
                    }
                    // ::fprintf(stderr, "xpos: %f, ypos: %f, zpos: %f\n", dock_target_pos.x, dock_target_pos.y, dock_target_pos.z);
                    // ::fprintf(stderr, "xvar: %f, yvar: %f, zvar: %f\n", dock_target_var.x, dock_target_var.y, dock_target_var.z);

                    filt_yaw_cmd_deg = _yaw_filter.apply(yaw_to_obs_deg);
                    filt_dist_to_obs_m = _dist_filter.apply(dist_to_obs_m);
                    
                    heading_obs_rad = ahrs.get_yaw() + filt_yaw_cmd_deg*DEG_TO_RAD;
                    cos_yaw_obs = cosf(heading_obs_rad);
                    sin_yaw_obs = sinf(heading_obs_rad);
                }

                Vector2f vec_to_target_2d_m = _filt_dock_target_pos.xy() - _current_vehicle_position.xy();

                // Update target information for attach()
                if (!_lock_commands){
                    // _xy_pos = _filt_dock_target_pos.todouble().xy() * 100.0; // xy pos in NEU cm
                    _xy_vel = vec_to_target_2d_m.normalized()*_dock_speed_mps.get()*100.0f;
                    // _z_pos = -_filt_dock_target_pos.z*100.0f;
                    _bearing_cd = get_bearing_cd(_current_vehicle_position.xy(), _filt_dock_target_pos.xy());
                }
                Vector2f reverse_vel = _xy_vel.normalized()*-_undock_speed_mps.get()*100.0f; // convert m/s to cm/s
                float reverse_vel_z = ((0.0<_z_vel)-(_z_vel<0.0))*-_undock_speed_mps.get()*100.0f;

                

                // Docking state controller
                switch (_docking_state){
                    case DockingState::ATTACH_MANEUVER: 
                        // pos_control->input_pos_vel_accel_xy(_xy_pos, _xy_vel, _xy_accel);
                        // pos_control->input_pos_vel_accel_z(_z_pos, _z_pos, _z_accel);
                        pos_control->input_vel_accel_xy(_xy_vel, _xy_accel);
                        pos_control->input_vel_accel_z(_z_vel, _z_accel);
                        filt_yaw_cmd_deg = wrap_180(_bearing_cd/100.0f - ahrs.get_yaw()*RAD_TO_DEG);
                        // ::fprintf(stderr,"sending pos x: %f, y: %f, z: %f\n", _xy_pos.x, _xy_pos.y, _z_pos);
                        // ::fprintf(stderr,"sending vel x: %f, y: %f, z: %f\n", _xy_vel.x, _xy_vel.y, _z_vel);
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
                            distance_err_cm*cos_yaw_obs,
                            distance_err_cm*sin_yaw_obs
                        );
                        Vector2p target_pos = pos_control->get_pos_target_cm().xy();
                        target_pos += dist_correction;
                        Vector2f target_vel(
                            -vel_rt*sin_yaw_obs,
                            vel_rt*cos_yaw_obs
                        );
                        pos_control->input_pos_vel_accel_xy(target_pos, target_vel, Vector2f(0,0)); // input pos and vel targets
                        _last_vehicle_pos = _current_vehicle_position.xy();
                        break;

                }

                // YAW CONTROLLER //
                /*Update the heading controller at a low rate (this is because the auto yaw controller uses
                a dt parameter that goes to zero if you update too fast). We also do it if we've reached a target early.*/
                bool dz_exceeded = abs(filt_yaw_cmd_deg - _last_yaw_cmd_deg) > _yaw_dz;
                dz_exceeded |= (_docking_state==DockingState::ATTACH_MANEUVER);
                if ((millis() - _last_yaw_update_ms > 200) && dz_exceeded) { // || auto_yaw.reached_fixed_yaw_target()
                    int8_t direction = (filt_yaw_cmd_deg >= 0 ? 1.0 : -1.0);
                    auto_yaw.set_fixed_yaw(abs(filt_yaw_cmd_deg), 0.0f, direction, true);
                    _last_yaw_cmd_deg = filt_yaw_cmd_deg;
                    _last_yaw_deg = ahrs.get_yaw()*RAD_TO_DEG;
                    _last_yaw_update_ms = millis();
                }
                // END YAW CONTROLLER //

                pos_control->update_xy_controller(); // run pos controller
                // AUGMENT CONTROL //
                attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
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
