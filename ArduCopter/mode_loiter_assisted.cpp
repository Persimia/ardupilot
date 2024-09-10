#include "Copter.h"

#if MODE_LOITER_ASSISTED_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

#define PITCH_TO_FW_VEL_GAIN_DEFAULT         0.1
#define ROLL_TO_RT_VEL_GAIN_DEFAULT          0.1
#define MIN_OBS_DIST_CM_DEFAULT              150

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

    // Obstacle Heading Vector
    _sin_yaw_obs = sinf(ahrs.get_yaw());
    _cos_yaw_obs = cosf(ahrs.get_yaw());

    _target_acquired = false;
    _distance_target_cm = 300.0f;

    copter.set_simple_mode(Copter::SimpleMode::NONE); // disable simple mode for this mode. TODO: Validate

#if AC_PRECLAND_ENABLED
    _precision_loiter_active = false;
#endif
    // // Set auto yaw... auto yaw is used in auto modes normally, so we shouldn't do that here
    auto_yaw.set_mode(AutoYaw::Mode::HOLD);

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
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

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

    case AltHoldModeState::Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Get Yaw angle and distance to closest object from AP_Proximity
        float yaw_to_obs_deg; // yaw angle is measured in the local frame
        float dist_to_obs_m;

        if (g2.proximity.get_closest_object(yaw_to_obs_deg, dist_to_obs_m)) {
            float heading_obs_rad = ahrs.get_yaw() + yaw_to_obs_deg*DEG_TO_RAD;
            _sin_yaw_obs = sinf(heading_obs_rad);
            _cos_yaw_obs = cosf(heading_obs_rad);

            // YAW CONTROLLER //
            /*Update the heading controller at a low rate (this is because the auto yaw controller uses
            a dt parameter that goes to zero if you update too fast). We also do it if we've reached a target early.*/ 
            if (millis() - auto_yaw.get_last_update_ms() > 200 || auto_yaw.reached_fixed_yaw_target()) {
                yaw_to_obs_deg = wrap_180(yaw_to_obs_deg);
                int8_t direction = (yaw_to_obs_deg >= 0 ? 1.0 : -1.0);
                auto_yaw.set_fixed_yaw(abs(yaw_to_obs_deg), 0.0f, direction, true);
            }
            // END YAW CONTROLLER //

            // POSITION CONTROLLER //
            float vel_fw = -_pitch_to_fw_vel_gain * target_pitch; //Forward Velocity Command TODO: Make this a parameter gain!
            float vel_rt =  _roll_to_rt_vel_gain * target_roll; //Right Velocity Command 
            if(!_target_acquired){// Reacquired target so reset distance
                _distance_target_cm = dist_to_obs_m * 100.0;
                _target_acquired = true;
            }
            else{
                _distance_target_cm -= vel_fw*pos_control->get_dt();
            }

            // Min dist check
            if (_distance_target_cm < _min_obs_dist_cm) {
                _distance_target_cm = _min_obs_dist_cm;
            }

            float distance_err_cm = dist_to_obs_m * 100.0 - _distance_target_cm;
            Vector2p dist_correction(
                distance_err_cm*_cos_yaw_obs,
                distance_err_cm*_sin_yaw_obs
            );
            Vector2p target_pos = pos_control->get_pos_target_cm().xy();
            target_pos += dist_correction;
            Vector2f target_vel(
                -vel_rt*_sin_yaw_obs,
                vel_rt*_cos_yaw_obs
            );
            Vector2f zero;

            pos_control->input_pos_vel_accel_xy(target_pos, target_vel, zero); // input pos and vel targets
            pos_control->update_xy_controller(); // run pos controller

            ::fprintf(stderr, "dtcm: %.0f, decm: %.1f, tp: %.0f, %.0f, dc: %.0f, %.0f\n",
            _distance_target_cm, distance_err_cm, target_pos[0], target_pos[1],
            dist_correction[0], dist_correction[1]);
            // END POSITION CONTROL //
            
            // AUGMENT CONTROL //
            attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
            // get avoidance adjusted climb rate
            #if AP_RANGEFINDER_ENABLED
            // update the vertical offset based on the surface measurement
            copter.surface_tracking.update_surface_offset();
            #endif
            // Send the commanded climb rate to the position controller
            pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
            break;
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
            break;
        }
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

#endif
