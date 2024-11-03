#include "AP_Proximity_CurveFit.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Proximity/AP_Proximity.h>
#include <algorithm>


#if AP_PROXIMITY_CURVEFIT_ENABLED
#define LARGE_FLOAT 1.0e3
#define QUANTIZATION 1.0E-3

const AP_Param::GroupInfo AP_Proximity_CurveFit::var_info[] = {
    // @Param{Copter}: _DISC
    // @DisplayName: Proximity Curvefit Discontinuity Threshold
    // @Description: Difference in Distance between subsequent points
    // @Values: 0.1, 5.0
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_DISC", 1, AP_Proximity_CurveFit, _discontinuity_threshold, 0.5f),


    // @Param{Copter}: _ANGMIN
    // @DisplayName: Proximity Curvefit Minimum Angle
    // @Description: Proximity Minimum Angle
    // @Values: -90, -5
    // @Units: deg  
    // @User: Advanced
    AP_GROUPINFO("_ANGMIN", 2, AP_Proximity_CurveFit, _angle_min_deg, -30),
    
    // @Param{Copter}: _ANGMAX
    // @DisplayName: Proximity Curvefit Maximum Angle
    // @Description: Proximity Curvefit Maximum Angle
    // @Values: 5, 90
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("_ANGMAX", 3, AP_Proximity_CurveFit, _angle_max_deg, 30),

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

    // @Param{Copter}: _LDR_HZ
    // @DisplayName: Lidar sweep rate in Hz
    // @Description: Lidar sweep rate in Hz
    // @Units: hz
    // @User: Advanced
    AP_GROUPINFO("_LDR_HZ", 6, AP_Proximity_CurveFit, _lidar_sweep_rate_hz, 3.73),

    // @Param{Copter}: _CNTR_HZ
    // @DisplayName: Center position cutoff filter hz
    // @Description: Center position cutoff filter hz
    // @Units: hz
    // @User: Advanced
    AP_GROUPINFO("_CNTR_HZ", 7, AP_Proximity_CurveFit, _center_filter_cutoff_hz, 1),

    AP_GROUPEND
};

AP_Proximity_CurveFit::AP_Proximity_CurveFit()
{
    AP_Param::setup_object_defaults(this, var_info);
    _center_filter.set_cutoff_frequency(_lidar_sweep_rate_hz, _center_filter_cutoff_hz); // TODO: use custom dock hz param

}


bool AP_Proximity_CurveFit::get_target(float &heading, float &distance, Vector2f &tangent_vec, Vector2f &normal_vec, Vector2f &center)
{
    Vector2f curr_pos;

    if(!AP::ahrs().get_relative_position_NE_origin(curr_pos)){
        _center_filter.reset();
        return false;} // No Position Estimate
    if(!compute_fit(curr_pos, _tangent_vec, _normal_vec, _center, _fit_quality, _fit_type, _fit_num)){
        _center_filter.reset();
        return false;}// Unable to solve heading, distance
    // Vector from vehicle position to center of curvature;
    Vector2f r_pos_center = _center - curr_pos;
    switch (_fit_type)
    {
        case AP_Proximity_CurveFit::CenterType::POINT:
            FALLTHROUGH;
        case AP_Proximity_CurveFit::CenterType::LINE:
            distance = r_pos_center.length();
            heading = wrap_PI(r_pos_center.angle());
            tangent_vec = _tangent_vec;
            normal_vec = _normal_vec;
            center = _center;
            break;
        case AP_Proximity_CurveFit::CenterType::NONE:
            FALLTHROUGH;
        default:
            return false;
    }

    log_target(heading, distance); //write to dataflash log
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
            // Vector2f old_center = center;
            center = _center_filter.apply(center);
            // fprintf(stderr, "Unfilt: %.4f, %.4f\t Filt: %.4f, %.4f\n",old_center.x, old_center.y, center.x, center.y);
            log_fit(center, normal_vec, fit_quality, fit_num);
            return true;
        }
    } else

    if (solve_point(curr_pos, fit_num, tangent_vec, normal_vec, center, fit_quality))
    {
        fit_type = AP_Proximity_CurveFit::CenterType::POINT;
        // Vector2f old_center = center;
        center = _center_filter.apply(center);
        // fprintf(stderr, "Unfilt: %.4f, %.4f\t Filt: %.4f, %.4f\n",old_center.x, old_center.y, center.x, center.y);
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

    // Calculate fit quality as the ratio of variances along the principal axis
    float total_variance = cov_xx + cov_yy;
    fit_quality = (total_variance > 0) ? (std::max(eigenvalue1, eigenvalue2) / total_variance) : 0;

    return true;
}



void AP_Proximity_CurveFit::add_point(float angle_deg, float distance)
{
    angle_deg = wrap_180(angle_deg);
    uint8_t dir = (_last_angle < angle_deg) ? 1 : (_last_angle > angle_deg ? -1 : 0);

    // check if angle is within range
    if(angle_deg > _angle_min_deg.get() && angle_deg < _angle_max_deg.get()){
        if(distance > _rng_min_m.get() && distance < _rng_max_m.get()){
            if (_write_end < _write_start + CURVEFIT_DATA_LEN){
                Vector2f current_position;
                if(AP::ahrs().get_relative_position_NE_origin(current_position)){
                    // compute point in Global NE frame
                    Vector2f point;
                    float pitch_rad = AP::ahrs().get_pitch(); // replace with pitch of sensor
                    float yaw_rad = AP::ahrs().get_yaw();
                    point.x = distance * cosf(pitch_rad) * cosf(angle_deg*DEG_TO_RAD+yaw_rad) + current_position.x;
                    point.y = distance * cosf(pitch_rad) * sinf(angle_deg*DEG_TO_RAD+yaw_rad) + current_position.y;

                    _points_NE_origin[_write_end] = point;
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
        _reset_flag = true;
    }
    if (dir != _last_dir) { // if direction changes, we want to reset if we haven't already
        if (!_reset_flag) {
            if (!_first_time_range_error) { // ignore the first one because we can't know first dir
                gcs().send_text(MAV_SEVERITY_CRITICAL, "CFIT ANG RANGE TOO BIG!"); 
                // maybe we don't need to alert anyone since we're doing the reset here anyway?
                reset();
            }
            else {
                _first_time_range_error = false;
            }
        } else {
            _reset_flag = false;
        }
        
    }

    _last_angle = angle_deg;
    _last_dir = dir;
    return;
}

void AP_Proximity_CurveFit::truncate_data(Vector2f curr_pos)
{
    if(_read_start == _read_end){ return;} //No data

    int closest_index = find_closest_index(curr_pos);

    Vector2f normal_dir = (_points_NE_origin[closest_index]).normalized();
    // Check for discontinuity and truncate data 
    for(int i = closest_index; i < _read_end-1; i++){
        if(normal_dir.dot(_points_NE_origin[i+1] - _points_NE_origin[i]) > _discontinuity_threshold.get()){
            _read_end = i+1;
            break;
        }
    }
    
    for(int i = closest_index; i > _read_start; i--){
        if(normal_dir.dot(_points_NE_origin[i-1] - _points_NE_origin[i]) > _discontinuity_threshold.get()){
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

int AP_Proximity_CurveFit::find_closest_index(const Vector2f curr_pos){
    float closest_distance = (_points_NE_origin[_read_start] - curr_pos).length_squared();
    int closest_index = _read_start;
    for(int i=_read_start+1; i<_read_end; i++){
        float distance = (_points_NE_origin[i] - curr_pos).length_squared();
        if(distance < closest_distance){
            closest_index = i;
            // closest_distance = distance;
        }
    }
    return closest_index;
}

void AP_Proximity_CurveFit::reset()
{
    _read_end = _write_end;
    _read_start = _write_start;
    _write_start = (_write_start == CURVEFIT_DATA_LEN ? 0 : CURVEFIT_DATA_LEN);
    _write_end = _write_start;

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

void AP_Proximity_CurveFit::log_target(const float heading, const float distance)
{
    if(_fit_type == NONE){
    // return id there is no valid fit
        return;
    }
    
    struct log_Proximity_Cfit_Target pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PROXIMITY_CFIT_TGT_MSG),
        time_us        :  AP_HAL::micros64(),
        target_heading :  heading*RAD_TO_DEG,
        target_distance:  distance
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
}

#endif