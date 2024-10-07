#include "AP_Proximity_CurveFit.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Proximity/AP_Proximity.h>
#include <algorithm>
#define LARGE_FLOAT 1.0e3

#if AP_PROXIMITY_CURVEFIT_ENABLED

const AP_Param::GroupInfo AP_Proximity_CurveFit::var_info[] = {

    // @param{Copter}: _MINPTS
    // @Displayname: Proximity Curvefit Minimum Points
    // @Description: Minimum nunber of data points to be consideredd a surface
    // @Values: 1, 255
    // @User: Advanced
    AP_GROUPINFO("_MINPTS", 1, AP_Proximity_CurveFit, _min_pts, 20),

    // @Param{Copter}: _FLAT
    // @DisplayName: Proximity Curvefit Flatness Threshold
    // @Description: maximum  value of det(A) to be considered flat
    // @Values: 0.1, 20000.00
    // @User: Advanced
    AP_GROUPINFO("_FLAT", 2, AP_Proximity_CurveFit, _flatness_threshold, 1),

    // @Param{Copter}: _DISC
    // @DisplayName: Proximity Curvefit Discontinuity Threshold
    // @Description: Difference in Distance between subsequent points
    // @Values: 0.1, 5.0
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_DISC", 3, AP_Proximity_CurveFit, _discontinuity_threshold, 2),

    // @Param{Copter}: _CNR
    // @DisplayName: Proximity Curvefit Corner Threshold
    // @Description: maximum  value of det(A) to be considered flat
    // @Values: 0.1, 20000.00
    // @User: Advanced
    AP_GROUPINFO("_CNR", 4, AP_Proximity_CurveFit, _corner_threshold, 0.7),

    // @Param{Copter}: _CNRWIN
    // @DisplayName: Proximity Curvefit Corner Threshold
    // @Description: maximum  value of det(A) to be considered flat
    // @Values: 0.1, 20000.00
    // @User: Advanced
    AP_GROUPINFO("_CNRWIN", 5, AP_Proximity_CurveFit, _corner_window, 10),


    // @Param{Copter}: _ANGMIN
    // @DisplayName: Proximity Curvefit Minimum Angle
    // @Description: Proximity Minimum Angle
    // @Values: -90, -5
    // @Units: deg  
    // @User: Advanced
    AP_GROUPINFO("_ANGMIN", 6, AP_Proximity_CurveFit, _angle_min_deg, -22.5),
    
    // @Param{Copter}: _ANGMAX
    // @DisplayName: Proximity Curvefit Maximum Angle
    // @Description: Proximity Curvefit Maximum Angle
    // @Values: 5, 90
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("_ANGMAX", 7, AP_Proximity_CurveFit, _angle_max_deg, 22.5),

    // @Param{Copter}: _RNGMIN
    // @DisplayName: Proximity Curvefit Minimum Range
    // @Description: Proximity Curvefit Minimum Range
    // @Values: 0.5, 50
    // @Units m
    // @User: Advanced
    AP_GROUPINFO("_RNGMIN", 8, AP_Proximity_CurveFit, _rng_min_m, 0.5),
    
    // @Param{Copter}: _RNGMAX
    // @DisplayName: Proximity Curvefit Maximum Range
    // @Description: Proximity Curvefit Maximum Range
    // @Values: 0.5, 50
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_RNGMAX", 9, AP_Proximity_CurveFit, _rng_max_m, 5),

    AP_GROUPEND
};

AP_Proximity_CurveFit::AP_Proximity_CurveFit()
{
    AP_Param::setup_object_defaults(this, var_info);
    closest_distance_ = _rng_max_m.get();
}


bool AP_Proximity_CurveFit::get_target(float &heading, float &distance)
{
    Vector2f curr_pos;
    if(!AP::ahrs().get_relative_position_NE_origin(curr_pos)){
        return false; // No Position Estimate
    }

    if(!compute_curvature_center(curr_pos)){
        return false; // Unable to solve heading, distance
    }

    // Vector from vehicle position to center of curvature;
    Vector2f r_pos_center = center - curr_pos + reference_point;
    switch (center_type)
    {
    case AP_Proximity_CurveFit::CenterType::POINT:
    {
        distance = r_pos_center.length();
        heading = wrap_PI(r_pos_center.angle());
        break;
    }
    case AP_Proximity_CurveFit::CenterType::CIRCLE_CONVEX:
    {
        distance = r_pos_center.length() - radius;
        heading = wrap_PI(r_pos_center.angle());
        break;
    }
    case AP_Proximity_CurveFit::CenterType::CIRCLE_CONCAVE:
    {
        distance = -(r_pos_center.length()) + radius;
        heading = wrap_PI((-r_pos_center).angle());
        break;
    }
    case AP_Proximity_CurveFit::CenterType::LINE:
    {
        heading = wrap_PI(r_pos_center.angle());
        Vector2f normal = Vector2f{r_pos_center};
        normal.normalize();
        distance = (data[closest_index] - curr_pos).dot(normal);
        break;
    }
    case AP_Proximity_CurveFit::CenterType::NONE:
        FALLTHROUGH;
    default:
        return false;
    }

    log_target(heading,distance); //write to dataflash log
    return true;
}


bool AP_Proximity_CurveFit::compute_curvature_center(Vector2f reference)
{
    truncate_data();
    if(read_end-read_start <= _min_pts.get()){
        // Insufficient Data
        return false;
    }

    if(center_type != AP_Proximity_CurveFit::CenterType::NONE){
        // Nothing to do
        return true;
    }
    
    reference_point = Vector2f(reference);
    
    if(detect_corner()){
        // Point
        center_type = AP_Proximity_CurveFit::CenterType::POINT;
        center = data[closest_index];
        radius = 0.0;
        log_fit();
        return true;
    }


    AP_Proximity_CurveFit::Coefficients coefficients;
    compute_coefficients(coefficients);

    if(solve_circle(coefficients)){
        if((center).length() < radius){
            //origin is inside the circle => concave
            center_type = AP_Proximity_CurveFit::CenterType::CIRCLE_CONCAVE;
        }
        else{
            //origin is outside the circle => convex
            center_type = AP_Proximity_CurveFit::CenterType::CIRCLE_CONVEX;
        }
        log_fit();
        return true;
    }

    if(solve_line(coefficients)){//Try to fit a line
        center_type = AP_Proximity_CurveFit::CenterType::LINE;
        log_fit();
        return true;
    }

    return false;

}

bool AP_Proximity_CurveFit::solve_circle(AP_Proximity_CurveFit::Coefficients c)
{
// Circle
//
// (x-a)^2 + (y-b)^2 - r^2 = 0 
//
//     |2*Sum(xk^2)   2*Sum(xk*yk)    sum(xk)|
// A = |2*Sum(xy*yk)  2*Sum(yk^2)     sum(yk)|
//     |2*Sum(xk)     2*Sum(yk)       n      |
//
//     |Sum(xk^3) + Sum(xk*yk^2)|
// B = |Sum(yk*xk^2) + Sum(yk^3)|
//     |Sum(xk^2) + Sum(yk^2)   |
//

    // Check if there are a sufficient number of points
    if (c.n < 4){
        return false;
    }

    Matrix3f A = {{2*c.Sum_x2, 2*c.Sum_xy, c.Sum_x},
                  {2*c.Sum_xy, 2*c.Sum_y2, c.Sum_y},
                  {2*c.Sum_x,  2*c.Sum_y,  float(c.n)}};
    
    if(A.det()*1.0e5f/(float(c.n) * c.Sum_x2 * c.Sum_y2) < _flatness_threshold.get()){
        return false;
    }

    Matrix3f Ainv;
    if(!A.inverse(Ainv)){
        return false;
    };

    Vector3f B = {c.Sum_x3 + c.Sum_xy2, c.Sum_yx2 + c.Sum_y3, c.Sum_x2 + c.Sum_y2};

    Vector3f solution = Ainv*B;
    center.x = solution.x;
    center.y = solution.y;
    float radius_squared = solution.z + solution.x*solution.x +solution.y*solution.y;
    if(radius_squared <= 0.0){
        return false;
    }
    radius = sqrtf(radius_squared);
    return true;
}

bool AP_Proximity_CurveFit::solve_line(AP_Proximity_CurveFit::Coefficients c)
{
// Line
//
// ax + by - 1 = 0
//
// A = |Sum(xk^2)   Sum(xk*yk)|
//     |Sum(xy*yk)  Sum(yk^2) |
//
// B = |Sum(xk)|
//     |Sum(yk)|
//     
    // Check if there are a sufficient number of points
    if (c.n < 3){
        return false;
    }

    float det = c.Sum_x2 * c.Sum_y2 - c.Sum_xy * c.Sum_xy; 

    if (abs(det) < 1e-6){
        return false;
    }

    center.x = (c.Sum_y2*c.Sum_x - c.Sum_xy*c.Sum_y)/det;
    center.y = (c.Sum_x2*c.Sum_y - c.Sum_xy*c.Sum_x)/det;

    center = center.normalized()*LARGE_FLOAT;
    radius = LARGE_FLOAT;
    return true;
}


void AP_Proximity_CurveFit::add_point(float angle_deg, float distance)
{
    angle_deg = wrap_180(angle_deg);
    if(angle_deg > _angle_min_deg.get() && angle_deg < _angle_max_deg.get()){
        if(distance > _rng_min_m.get() && distance < _rng_max_m.get() && write_end < write_start + CURVEFIT_DATA_LEN){
            Vector2f current_position;
            if(AP::ahrs().get_relative_position_NE_origin(current_position)){
                float yaw = AP::ahrs().get_yaw();
                Vector2f point;
                point.x = distance*cosf(angle_deg*DEG_TO_RAD+yaw) + current_position.x;
                point.y = distance*sinf(angle_deg*DEG_TO_RAD+yaw) + current_position.y;
                data[write_end] = point;

                if (distance <= closest_distance_){
                    closest_distance_ = distance;
                    closest_index_ = write_end;
                }
                write_end += 1;
            }
        }
    }
    else if(last_angle > _angle_min_deg.get() && last_angle < _angle_max_deg.get()){
        reset();
    }
    last_angle = angle_deg;
    return;
}

void AP_Proximity_CurveFit::truncate_data()
{

    if(read_start == read_end){
        //No data
        return;
    }

    Vector2f normal_dir = (data[closest_index] - reference_point).normalized();
    // Check for discontinuity and truncate data 
    for(int i = closest_index; i < read_end-1; i++){
        if(normal_dir.dot(data[i+1] - data[i]) > _discontinuity_threshold.get()){
            read_end = i;
            break;
        }
    }
    
    for(int i = closest_index; i > read_start; i--){
        if(normal_dir.dot(data[i-1] - data[i]) > _discontinuity_threshold.get()){
            read_start = i;
            break;
        }
    }
    return;
}

bool AP_Proximity_CurveFit::detect_corner(){
    // Check if the min_distance is a corner
    if (read_end == read_start){
        return false;
    }

    Vector2f fwd{0.0, 0.0};
    float fwn = 0.0;
    Vector2f bkd{0.0, 0.0};
    float bkn = 0.0;

    if(closest_index == read_end){
        // Vector Corresponding to sight line
        fwd = (data[read_end]-reference_point).normalized();
        fwn  = 1.0;
    }
    else{
        for(int i = closest_index+1; i < read_end && i <= closest_index +_corner_window.get()/2; i++){
            fwd += (data[i] - data[closest_index]).normalized();
            fwn += 1.0;
        }
    }

    if(closest_index == read_start){
        bkd = (data[read_start]-reference_point);
        bkn = 1.0;
    }
    else{
        for(int i = closest_index-1; i >= read_start && i >= closest_index -_corner_window.get()/2; i--){
            bkd += (data[i] - data[closest_index]).normalized();
            bkn += 1.0;
        }
    }

    if(fwn < 0.1 || bkn < 0.1){
        return false;
    }

    fwd = fwd/fwn;
    bkd = bkd/bkn;

    return (1 + fwd.dot(bkd)) > _corner_threshold;

}


void AP_Proximity_CurveFit::compute_coefficients(AP_Proximity_CurveFit::Coefficients &c)
{   
    for(int it = read_start; it < read_end; it++){
        float x = data[it].x - reference_point.x;
        float y = data[it].y - reference_point.y;
        c.Sum_x += x;
        c.Sum_y += y;
        c.Sum_x2 += x*x;
        c.Sum_xy += x*y;
        c.Sum_y2 += y*y;
        c.Sum_xy2 += x*y*y;
        c.Sum_yx2 += y*x*x;
        c.Sum_x3 += x*x*x;
        c.Sum_y3 += y*y*y;
        c.n += 1;
    }
}

void AP_Proximity_CurveFit::reset()
{
    read_end = write_end;
    read_start = write_start;
    write_start = (write_start == CURVEFIT_DATA_LEN ? 0 : CURVEFIT_DATA_LEN);
    write_end = write_start;

    closest_index = closest_index_;
    closest_index_ = write_start;
    closest_distance_ = _rng_max_m.get();

    center_type = AP_Proximity_CurveFit::CenterType::NONE;
}

void AP_Proximity_CurveFit::log_fit()
{
    if(center_type == NONE){
    // return if there is no valid fit
        return;
    }
    Vector2f GlobalCenter = center+reference_point;
    AP::logger().Write("CFIT","TimeUS,Typ,CX,CY,R","s-mmm","F-000","QBfff",
                        AP_HAL::micros64(),
                        uint8_t(center_type),
                        GlobalCenter.x,
                        GlobalCenter.y,
                        radius
    );
}

void AP_Proximity_CurveFit::log_target(const float heading, const float distance)
{
    if(center_type == NONE){
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