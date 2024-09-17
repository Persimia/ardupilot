#include "Copter.h"

#if MODE_DOCK_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeDock::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles TODO change to get_pilot_desired_vel()
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

    // Loiter State Machine Determination
    AltHoldModeState dock_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (dock_state) {

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
        loiter_nav->update(false); // false => don't run obstacle avoidance

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        target_acquired = false;
        break;

    case AltHoldModeState::Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        AC_AttitudeControl::HeadingCommand heading_cmd;
        // Get Yaw angle and distance to closest object from AP_Proximity
        float yaw_to_obst_deg;
        float dist_to_obst_m;
        Vector3f thrust_vector;

        if(g2.proximity.get_closest_object(yaw_to_obst_deg, dist_to_obst_m)){
            // YAW CONTROLLER //
            /*Update the heading controller at a low rate (this is because the auto yaw controller uses
            a dt parameter that goes to zero if you update too fast). We also do it if we've reached a target early.*/ 
            uint32_t cur_time_ms = millis();
            if (cur_time_ms - last_update_ms > 200 || auto_yaw.reached_fixed_yaw_target()) {
                last_update_ms = cur_time_ms;
                yaw_to_obst_deg = wrap_180(yaw_to_obst_deg);
                int8_t direction = (yaw_to_obst_deg >= 0 ? 1.0 : -1.0);
                auto_yaw.set_fixed_yaw(abs(yaw_to_obst_deg), 0.0f, direction, true);

                //GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "X:%f Y:%f Z: %f", targ[0], targ[1], targ[2]);

                float heading_obst = ahrs.get_yaw() + yaw_to_obst_deg*DEG_TO_RAD;
                sin_yaw_obst = sinf(heading_obst);
                cos_yaw_obst = cosf(heading_obst);
                
                
            }
             heading_cmd = auto_yaw.get_heading();
            // END YAW CONTROLLER //

            // POSITION CONTROLLER //
            float vel_fw = -0.1f * target_pitch; //Forward Velocity Command
            float vel_rt =  0.1f * target_roll; //Right Velocity Command

            if(!target_acquired){// Reacquired target so reset distance
                distance_target = dist_to_obst_m * 100.0;
                target_acquired=true;
                GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Target Acquired");
            }
            else{
                distance_target -= vel_fw*pos_control->get_dt();
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
                -vel_rt*sin_yaw_obst,
                vel_rt*cos_yaw_obst
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
            target_acquired = false;
            GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Target Lost");

            loiter_nav->update(false); // false => don't run obstacle avoidance


            thrust_vector = loiter_nav->get_thrust_vector();
        }
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
