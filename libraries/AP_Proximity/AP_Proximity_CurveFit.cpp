#include "AP_Proximity_CurveFit.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Proximity/AP_Proximity.h>
#include <algorithm>
#include <GCS_MAVLink/GCS.h>
#include <tuple>


#if AP_PROXIMITY_CURVEFIT_ENABLED
#define LARGE_FLOAT 1.0e3
#define QUANTIZATION 1.0E-3

const AP_Param::GroupInfo AP_Proximity_CurveFit::var_info[] = {
    // @Param{Copter}: _GIMBAL
    // @DisplayName: Proximity Curvefit Using Gimbal for Lidar
    // @Description: Put 1 or 0 for if you are using gimbal or not
    // @Values: 0, 1
    // @User: Advanced
    AP_GROUPINFO("_P_CORR", 1, AP_Proximity_CurveFit, _use_pitch_correction, 0),


    // @Param{Copter}: _ANGMIN
    // @DisplayName: Proximity Curvefit Minimum Angle
    // @Description: Proximity Minimum Angle
    // @Values: -90, -5
    // @Units: deg  
    // @User: Advanced
    AP_GROUPINFO("_ANGMIN", 2, AP_Proximity_CurveFit, _angle_min_deg, -65.0f),
    
    // @Param{Copter}: _ANGMAX
    // @DisplayName: Proximity Curvefit Maximum Angle
    // @Description: Proximity Curvefit Maximum Angle
    // @Values: 5, 90
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("_ANGMAX", 3, AP_Proximity_CurveFit, _angle_max_deg, 65.0f),

    // @Param{Copter}: _RNGMIN
    // @DisplayName: Proximity Curvefit Minimum Range
    // @Description: Proximity Curvefit Minimum Range
    // @Values: 0.5, 50
    // @Units m
    // @User: Advanced
    AP_GROUPINFO("_RNGMIN", 4, AP_Proximity_CurveFit, _rng_min_m, 0.2),
    
    // @Param{Copter}: _RNGMAX
    // @DisplayName: Proximity Curvefit Maximum Range
    // @Description: Proximity Curvefit Maximum Range
    // @Values: 0.5, 50
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_RNGMAX", 5, AP_Proximity_CurveFit, _rng_max_m, 3),

    // @Param{Copter}: _TRNC_D_MULT
    // @DisplayName: Proximity Curvefit Truncation distance cutoff
    // @Description: Proximity Curvefit Truncation distance cutoff
    // @User: Advanced
    AP_GROUPINFO("_TRNC_M", 6, AP_Proximity_CurveFit, _truncation_distance_m, 0.3f),

    // @Param{Copter}: _GIM_FW
    // @DisplayName: Proximity Curvefit Gimbal forward distance m
    // @Description: Proximity Curvefit Gimbal forward distance m
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_GIM_FW", 7, AP_Proximity_CurveFit, _gimbal_forward_m, .112f),

    // @Param{Copter}: _GIM_RT
    // @DisplayName: Proximity Curvefit Gimbal right distance m
    // @Description: Proximity Curvefit Gimbal right distance m
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_GIM_RT", 8, AP_Proximity_CurveFit, _gimbal_right_m, -0.00254f),

    // @Param{Copter}: _GIM_UP
    // @DisplayName: Proximity Curvefit Gimbal up distance m
    // @Description: Proximity Curvefit Gimbal up distance m
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_GIM_UP", 9, AP_Proximity_CurveFit, _gimbal_up_m, .204f),

    AP_GROUPEND
};

AP_Proximity_CurveFit::AP_Proximity_CurveFit()
{
    AP_Param::setup_object_defaults(this, var_info);
}


bool AP_Proximity_CurveFit::get_target(Vector2f &normal_vec, Vector2f &center)
{
    Vector2f curr_pos;
    if(!AP::ahrs().get_relative_position_NE_origin(curr_pos)){return false;} // No Position Estimate
    if(!compute_fit(curr_pos, _tangent_vec, _normal_vec, _center, _fit_quality, _fit_type, _fit_num)){return false;}// Unable to solve heading, distance
    normal_vec = _normal_vec;
    center = _center;
    return true;
}


bool AP_Proximity_CurveFit::compute_fit(Vector2f curr_pos,
    Vector2f& tangent_vec, Vector2f& normal_vec, Vector2f& center, 
    float& fit_quality, AP_Proximity_CurveFit::CenterType& fit_type, int &fit_num)
{
    if(fit_type != AP_Proximity_CurveFit::CenterType::NONE){return true;} // Nothing to do

    truncate_data(curr_pos); //this changes the amount of data we might have
    fit_num = _read_end - _read_start;
    if (fit_num < 1) {return false;} // No Data

    AP_Proximity_CurveFit::Coefficients coefficients;
    compute_coefficients(coefficients);

    if(fit_num >= _min_req_for_line){
        if(solve_line(coefficients, curr_pos, tangent_vec, normal_vec, center, fit_quality))
        {//Try to fit a line
            fit_type = AP_Proximity_CurveFit::CenterType::LINE;
            log_fit(center, normal_vec, fit_quality, fit_num);
            return true;
        }
    } else

    if (solve_point(curr_pos, fit_num, tangent_vec, normal_vec, center, fit_quality))
    {
        fit_type = AP_Proximity_CurveFit::CenterType::POINT;
        log_fit(center, normal_vec, fit_quality, fit_num);
        return true;
    }
    return false;
}

bool AP_Proximity_CurveFit::solve_point(Vector2f curr_pos, int fit_num,
    Vector2f& tangent_vec, Vector2f& normal_vec, Vector2f& center, 
    float& fit_quality)
{
    if (fit_num < 1) {
        return false;
    }
    // default to Point
    center.zero();
    for (int i = _read_start; i < _read_end; i++){
        center+=_points_NE_origin[i];
    }
    center /= (fit_num * 1.0f);
    
    // compute tangent and normal of fit here
    Vector2f vec_pos_center = center - curr_pos;
    normal_vec = vec_pos_center.normalized();
    tangent_vec = {-vec_pos_center.y, vec_pos_center.x};

    fit_quality = 1.0f;

    return true; 
}

bool AP_Proximity_CurveFit::solve_line(AP_Proximity_CurveFit::Coefficients c, Vector2f curr_pos,
    Vector2f& tangent_vec, Vector2f& normal_vec, Vector2f& center, 
    float& fit_quality) 
{
    // Ensure there are enough points to perform the fit
    if (c.n < _min_req_for_line) {
        return false;
    }

    // Calculate the centroid (mean point) of the points
    float mean_x = c.Sum_x / c.n;
    float mean_y = c.Sum_y / c.n;

    // Centered sums
    float sum_x_centered2 = c.Sum_x2 - c.n * mean_x * mean_x;
    float sum_y_centered2 = c.Sum_y2 - c.n * mean_y * mean_y;
    float sum_xy_centered = c.Sum_xy - c.n * mean_x * mean_y;

    // Covariance matrix
    float cov_xx = sum_x_centered2 / c.n;
    float cov_yy = sum_y_centered2 / c.n;
    float cov_xy = sum_xy_centered / c.n;

    // Eigenvalues and eigenvectors of the covariance matrix
    float trace = cov_xx + cov_yy;
    float determinant = cov_xx * cov_yy - cov_xy * cov_xy;
    if (determinant < 0) {
        // fprintf(stderr, "Warning: Covariance matrix is not positive semi-definite.\n");
        return false; // or handle accordingly
    }
    float eigenvalue1 = trace / 2 + std::sqrt(trace * trace / 4 - determinant);
    float eigenvalue2 = trace / 2 - std::sqrt(trace * trace / 4 - determinant);

    // Choose the eigenvector corresponding to the larger eigenvalue as the line direction
    if (eigenvalue1 > eigenvalue2) {
        tangent_vec.x = cov_xy;
        tangent_vec.y = eigenvalue1 - cov_xx;
    } else {
        tangent_vec.x = eigenvalue2 - cov_yy;
        tangent_vec.y = cov_xy;
    }
    tangent_vec = tangent_vec.normalized();

    // Calculate the normal vector perpendicular to the tangent vector
    normal_vec.x = -tangent_vec.y;
    normal_vec.y = tangent_vec.x;

    // Center point is the mean of the x and y values
    center.x = mean_x;
    center.y = mean_y;

    // Ensure the normal vector points toward curr_pos
    Vector2f center_to_curr = curr_pos - center;
    if (normal_vec.dot(center_to_curr) < 0) {
        normal_vec = -normal_vec; // Invert the normal vector
    }

    // Calculate fit quality as the ratio of variances along the principal axis
    float total_variance = cov_xx + cov_yy;
    fit_quality = (total_variance > 0) ? (std::max(eigenvalue1, eigenvalue2) / total_variance) : 0;

    return true;
}

void AP_Proximity_CurveFit::add_point(float angle_deg, float distance_m)
{
    angle_deg = wrap_180(angle_deg);

    // check if angle is within range
    if(angle_deg > _angle_min_deg.get() && angle_deg < _angle_max_deg.get()){
        if(distance_m > _rng_min_m.get() && distance_m < _rng_max_m.get()){
            if (_write_end < _write_start + CURVEFIT_DATA_LEN){
                Vector2f current_position_NE_m;
                if(AP::ahrs().get_relative_position_NE_origin(current_position_NE_m)){
                    // compute point in Global NE frame
                    Vector2f point;
                    float pitch_rad = AP::ahrs().get_pitch(); // replace with pitch of sensor
                    float yaw_rad = AP::ahrs().get_yaw();

                    Vector3f gimbal_offset = {_gimbal_forward_m.get(), _gimbal_right_m.get(), -_gimbal_up_m.get()};
                    gimbal_offset = AP::ahrs().get_rotation_body_to_ned()*gimbal_offset;

                    // fprintf(stderr,"Offset x:%.4f\ty:%.4f\tz:%.4f\n",gimbal_offset.x,gimbal_offset.y,gimbal_offset.z);

                    if (_use_pitch_correction.get()) {
                        point.x = distance_m * cosf(pitch_rad) * cosf(angle_deg*DEG_TO_RAD+yaw_rad) + current_position_NE_m.x + gimbal_offset.x;
                        point.y = distance_m * cosf(pitch_rad) * sinf(angle_deg*DEG_TO_RAD+yaw_rad) + current_position_NE_m.y + gimbal_offset.y;
                    }
                    else 
                    {
                        point.x = distance_m * cosf(angle_deg*DEG_TO_RAD+yaw_rad) + current_position_NE_m.x + gimbal_offset.x;
                        point.y = distance_m * sinf(angle_deg*DEG_TO_RAD+yaw_rad) + current_position_NE_m.y + gimbal_offset.y;
                    }
                    
                    _points_NE_origin[_write_end] = point;
                    if (abs(angle_deg) < _nearest_valid_point_angle_deg) {
                        _write_nearest_valid_point_index = _write_end; 
                        _nearest_valid_point_angle_deg = abs(angle_deg);
                    }

                    _write_end += 1;

                }
            } else {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "CFIT BUFFER FULL!");
            }
        }        
    }
    //reset when angle goes out of range
    //this indicates that a new scan is availible and moves the _write_start in preparation for the next scan
    else if(_last_angle > _angle_min_deg.get() && _last_angle < _angle_max_deg.get()){
        reset();
    }

    _last_angle = angle_deg;
    return;
}

void AP_Proximity_CurveFit::truncate_data(Vector2f curr_pos)
{   
    int fit_num = _read_end - _read_start;
    if (fit_num < 1){ return;} //No data

    // Check for distance metric and truncate data 
    for(int i = _read_nearest_valid_point_index; i < _read_end-1; i++){
        if((_points_NE_origin[i+1]-_points_NE_origin[i]).length() > _truncation_distance_m.get()){
            _read_end = i+1;
            break;
        }
    }
    for(int i = _read_nearest_valid_point_index; i > _read_start; i--){
        if((_points_NE_origin[i-1]-_points_NE_origin[i]).length() > _truncation_distance_m.get()){
            _read_start = i;
            break;
        }
    }
    return;
}


void AP_Proximity_CurveFit::compute_coefficients(AP_Proximity_CurveFit::Coefficients &c)
{   
    for(int it = _read_start; it < _read_end; it++){
        float x = _points_NE_origin[it].x;
        float y = _points_NE_origin[it].y;
        c.Sum_x += x;
        c.Sum_y += y;
        c.Sum_x2 += x*x;
        c.Sum_xy += x*y;
        c.Sum_y2 += y*y;
        c.n += 1;
    }
}

void AP_Proximity_CurveFit::reset()
{
    _read_end = _write_end;
    _read_start = _write_start;
    _write_start = (_write_start == CURVEFIT_DATA_LEN ? 0 : CURVEFIT_DATA_LEN);
    _write_end = _write_start;

    _read_nearest_valid_point_index = _write_nearest_valid_point_index; 
    _write_nearest_valid_point_index = 0; 
    _nearest_valid_point_angle_deg = INFINITY;

    _fit_type = AP_Proximity_CurveFit::CenterType::NONE;
}

void AP_Proximity_CurveFit::log_fit(Vector2f center, Vector2f normal_vec, float fit_quality, int fit_num)
{
    if(_fit_type == NONE){return;} // return if there is no valid fit
    float normal_angle = wrap_180(normal_vec.angle()*RAD_TO_DEG);
    AP::logger().Write("CFIT","TimeUS,Typ,CX,CY,NrmAng,Qual,Num","s-mmd--","F-00000","QBffffi",
                        AP_HAL::micros64(),
                        uint8_t(_fit_type),
                        center.x,
                        center.y,
                        normal_angle,
                        fit_quality,
                        fit_num
                        );
}

void AP_Proximity_CurveFit::log_target(const float heading_rad, const float distance_m)
{
    if(_fit_type == NONE){
    // return id there is no valid fit
        return;
    }
    
    struct log_Proximity_Cfit_Target pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PROXIMITY_CFIT_TGT_MSG),
        time_us        :  AP_HAL::micros64(),
        target_heading :  heading_rad*RAD_TO_DEG,
        target_distance:  distance_m
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

#endif