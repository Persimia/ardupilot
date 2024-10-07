#include "Copter.h"

#if MODE_DOCK_ENABLED == ENABLED

#define DOCK_VELXY_MAX_DEFAULT 200.0
#define DOCK_MIN_DIST_DEFAULT 100.0

#define CLOSED_LOOP false

/*
 * Init and run calls for dock flight mode
 */

const AP_Param::GroupInfo ModeDock::var_info[] = {
    // @Param: DOCK_VELXY_MAX
    // @DisplayName: Maximum Flight speed in Dock Mode cm_s
    // @Description: Maximum Flight speed in Dock Mode cm_s
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("VEL_MAX", 1, ModeDock, dock_velxy_max, DOCK_VELXY_MAX_DEFAULT),

    // @Param: DOCK_MIN_DIST
    // @DisplayName: Minimum distance to target
    // @Description: Minimum distance to target
    // @Range: 0 50
    // @User: Advanced
    AP_GROUPINFO("MIN_DIST", 2, ModeDock, dock_min_dist, DOCK_MIN_DIST_DEFAULT),
    AP_GROUPEND
};

ModeDock::ModeDock(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// loiter_init - initialise loiter controller
bool ModeDock::init(bool ignore_checks)
{
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

    // last_update of lidar data
    last_update_ms = millis();

    // Obstacle Heading Vector
    sin_yaw_obst = sinf(ahrs.get_yaw());
    cos_yaw_obst = cosf(ahrs.get_yaw());

    target_acquired = false;
    distance_target = 3.0;

    // Get Params
    dock_velxy_max.load();

    return true;
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeDock::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

/* Get Yaw angle and distance to closest object from AP_Proximity
    float target_yaw_angle;
    float separation;
    g2.proximity.get_closest_object(target_yaw_angle,separation);
    // Get Vehicle Heading
    float heading = ahrs.get_yaw()*RAD_TO_DEG; */
    

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        //update_simple_mode(); //Disable simple mode

        // convert pilot input to lean angles TODO: change to get_pilot_desired_vel()
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
#if !CLOSED_LOOP
    // Data Collection
        float yaw_attitude = ahrs.get_yaw();
        float yaw_to_obst_deg;
        float dist_to_obst_m;

    if(g2.proximity.get_closest_object(yaw_to_obst_deg, dist_to_obst_m)){
        // YAW CONTROLLER //

        float heading_obst = wrap_PI(yaw_attitude + yaw_to_obst_deg*DEG_TO_RAD);
#if AP_PROXIMITY_CURVEFIT_ENABLED == 1
            //////////////////////////////////////////////////////////////////////////////////////////////////
        // Curve Fit                                                                                    //
        //////////////////////////////////////////////////////////////////////////////////////////////////
        // Use curvefit to get an improved heading estimate
        g2.proximity.curvefit->get_target(heading_obst,dist_to_obst_m);     
#endif
    }
#endif

    // Loiter State Machine Determination
    AltHoldModeState flight_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (flight_state) {

    case AltHoldModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        target_acquired = false;
        break;

    case AltHoldModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHoldModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        target_acquired = false;
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
        target_acquired = false;
        break;

    case AltHoldModeState::Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        AC_AttitudeControl::HeadingCommand heading_cmd;
        Vector3f thrust_vector;

#if CLOSED_LOOP
        // Get Yaw angle and distance to closest object from AP_Proximity
        float yaw_attitude = ahrs.get_yaw();
        float yaw_to_obst_deg;
        float dist_to_obst_m;
        Vector2f target_vel_xy = get_pilot_desired_velocity_xy(dock_velxy_max.get());
        if(g2.proximity.get_closest_object(yaw_to_obst_deg, dist_to_obst_m)){
            // YAW CONTROLLER //

            float heading_obst = wrap_PI(yaw_attitude + yaw_to_obst_deg*DEG_TO_RAD);

            //////////////////////////////////////////////////////////////////////////////////////////////////
            // Curve Fit                                                                                    //
            //////////////////////////////////////////////////////////////////////////////////////////////////
            // Use curvefit to get an improved heading estimate
#if AP_PROXIMITY_CURVEFIT_ENABLED == 1
            Vector2f curr_pos;
            if(ahrs.get_relative_position_NE_origin(curr_pos)){
                g2.proximity.curvefit.get_target(heading_obst,dist_to_obst_m,curr_pos); //Get improved estimate
            }
#endif

            sin_yaw_obst = sinf(heading_obst);
            cos_yaw_obst = cosf(heading_obst);

            // Use auto yaw only if yaw error is large //
            if(abs(yaw_to_obst_deg) > 22.5){
                /*Update the heading controller at a low rate (this is because the auto yaw controller uses
                a dt parameter that goes to zero if you update too fast). We also do it if we've reached a target early.*/ 
                uint32_t cur_time_ms = millis();
                if (cur_time_ms - last_update_ms > 200 || auto_yaw.reached_fixed_yaw_target()) {
                    last_update_ms = cur_time_ms;
                    yaw_to_obst_deg = wrap_180(yaw_to_obst_deg);
                    int8_t direction = (yaw_to_obst_deg >= 0 ? 1.0 : -1.0);
                    auto_yaw.set_fixed_yaw(abs(yaw_to_obst_deg), 0.0f, direction, true);
                }
            heading_cmd = auto_yaw.get_heading();
            }
            else{// If heading error is small directly use Attitude Controller 
            heading_cmd.heading_mode = AC_AttitudeControl::HeadingMode::Angle_Only;
            heading_cmd.yaw_angle_cd = heading_obst*RAD_TO_DEG*100.0f;
            heading_cmd.yaw_rate_cds = 0.0;
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////

            // END YAW CONTROLLER //

            // POSITION CONTROLLER //


            if(!target_acquired){// Reacquired target so reset distance
                distance_target = dist_to_obst_m * 100.0;
                target_acquired=true;
                GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Target Acquired");
            }
            else if(distance_target > dock_min_dist.get() || target_vel_xy.x < 0.0){
                distance_target -= target_vel_xy.x*pos_control->get_dt();
            }

            double distance_err = dist_to_obst_m * 100.0 - distance_target;



            Vector2p dist_correction(
                distance_err*cos_yaw_obst,
                distance_err*sin_yaw_obst
            );

            Vector2p target_pos(
                pos_control->get_pos_target_cm()[0],
                pos_control->get_pos_target_cm()[1]
            );
            target_pos = target_pos + dist_correction;

            Vector2f target_vel(
                -target_vel_xy.y*sin_yaw_obst,
                target_vel_xy.y*cos_yaw_obst
            );

            Vector2f zero;

            // 
            pos_control->input_pos_vel_accel_xy(target_pos, target_vel, zero);
            // run pos controller
            pos_control->update_xy_controller();
            thrust_vector = pos_control->get_thrust_vector();
            // END POSITION CONTROL //

        }else{
            heading_cmd.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
            heading_cmd.yaw_rate_cds = target_yaw_rate;
            if(target_acquired){
                GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Target Lost");
            }
            target_acquired = false;

            loiter_nav->update(); // false => don't run obstacle avoidance


            thrust_vector = loiter_nav->get_thrust_vector();
        }
#else
        heading_cmd.heading_mode = AC_AttitudeControl::HeadingMode::Rate_Only;
        heading_cmd.yaw_rate_cds = target_yaw_rate;
        loiter_nav->update(); // false => don't run obstacle avoidance
        thrust_vector = loiter_nav->get_thrust_vector();

#endif
        // call attitude controller
        attitude_control->input_thrust_vector_heading(thrust_vector, heading_cmd);

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

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

uint32_t ModeDock::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeDock::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

#endif
