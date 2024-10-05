#include "AP_Proximity_CurveFit.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Proximity/AP_Proximity.h>
#include <algorithm>
#define LARGE_FLOAT 1.0e3

#if AP_PROXIMITY_CURVEFIT_ENABLED

const AP_Param::GroupInfo AP_Proximity_CurveFit::var_info[] = {

    // @Param{Copter}: _FLAT
    // @DisplayName: Proximity Curvefit Flatness Threshold
    // @Description: maximum  value of det(A) to be considered flat
    // @Values: 0.1, 20000.00
    // @User: Advanced
    AP_GROUPINFO("_FLAT", 1, AP_Proximity_CurveFit, _flatness_threshold, 1000),

    // @Param{Copter}: _DISC
    // @DisplayName: Proximity Curvefit Discontinuity Threshold
    // @Description: Difference in Distance between subsequent points
    // @Values: 0.1, 5.0
    // @User: Advanced
    AP_GROUPINFO("_DISC", 2, AP_Proximity_CurveFit, _discontinuity_threshold, 2),

    // @Param{Copter}: _CNR
    // @DisplayName: Proximity Curvefit Corner Threshold
    // @Description: maximum  value of det(A) to be considered flat
    // @Values: 0.1, 20000.00
    // @User: Advanced
    AP_GROUPINFO("_CNR", 3, AP_Proximity_CurveFit, _corner_threshold, 0.2),

    // @Param{Copter}: _MINANG
    // @DisplayName: Proximity Curvefit Minimum Angle
    // @Description: Proximity Minimum Angle
    // @Values: -90, -5,
    // @User: Advanced
    AP_GROUPINFO("_MINANG", 4, AP_Proximity_CurveFit, _angle_min_deg, -22.5),
    
    // @Param{Copter}: _MAXANG
    // @DisplayName: Proximity Curvefit Maximum Angle
    // @Description: Proximity Maximum Angle
    // @Values: 5, 90,
    // @User: Advanced
    AP_GROUPINFO("_MAXANG", 5, AP_Proximity_CurveFit, _angle_max_deg, 22.5),

    AP_GROUPEND
};

AP_Proximity_CurveFit::AP_Proximity_CurveFit()
{
    AP_Param::setup_object_defaults(this, var_info);
}


void AP_Proximity_CurveFit::get_target(float &heading, float & distance, const Vector2f curr_pos)
{
    if(!compute_curvature_center(curr_pos)){
        return; // Unable to solve heading, distance
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
        distance = (closest_point - curr_pos).dot(normal);
        break;
    }
    case AP_Proximity_CurveFit::CenterType::NONE:
        FALLTHROUGH;
    default:
        return;
    }

    log_target(heading,distance);

}


bool AP_Proximity_CurveFit::compute_curvature_center(Vector2f reference)
{

    if(center_type != AP_Proximity_CurveFit::CenterType::NONE){
        // Nothing to do
        return true;
    }

    if(read_end-read_start < 1){ // No data
        center_type = AP_Proximity_CurveFit::CenterType::NONE;
        return false;
    }
    
    reference_point = Vector2f(reference);
    filter_data();

    if(read_end-read_start <= 5){ // Treat as a single point
        center_type = AP_Proximity_CurveFit::CenterType::POINT;
        center = Vector2f(closest_point - reference_point);
        radius = 0.0;
        log_fit();
        return true;
    }

    AP_Proximity_CurveFit::Coefficients coefficients;
    compute_coefficients(coefficients);

    if(solve_circle(coefficients)){
        if((center).length() < radius){//origin is inside the circle
            center_type = AP_Proximity_CurveFit::CenterType::CIRCLE_CONCAVE;
        }
        else{ //origin is outside the circle
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

    // All else fails, take the closest point
    center = Vector2f(closest_point-reference_point);
    center_type = AP_Proximity_CurveFit::CenterType::POINT;
    log_fit();
    return true;

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
    
    if(A.det() < _flatness_threshold.get()){
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

    center.normalize();
    center = center*LARGE_FLOAT;
    radius = LARGE_FLOAT;
    return true;
}


void AP_Proximity_CurveFit::add_point(float angle_deg, float distance)
{
    angle_deg = wrap_180(angle_deg);
    if(angle_deg < _angle_max_deg.get() && angle_deg > _angle_min_deg.get() && write_end < write_start + CURVEFIT_DATA_LEN){
        Vector2f current_position;
        if(AP::ahrs().get_relative_position_NE_origin(current_position)){
            float yaw = AP::ahrs().get_yaw();
            PrxData point;
            point.position.x = distance*cosf(angle_deg*DEG_TO_RAD+yaw) + current_position.x;
            point.position.y = distance*sinf(angle_deg*DEG_TO_RAD+yaw) + current_position.y;
            point.distance = distance;
            data[write_end] = point;
            write_end += 1;
        }
    }
    else if(_last_angle > _angle_min_deg.get() && _last_angle < _angle_max_deg.get()){
        reset();
    }
    _last_angle = angle_deg;
}

void AP_Proximity_CurveFit::filter_data()
{
    // Find the closest point
    auto closest_data = std::min_element(data+read_start, data+read_end,
    [](const PrxData a, const PrxData b)->bool {return a.distance<b.distance;});
    closest_point = Vector2f(closest_data->position);
    closest_distance = closest_data->distance;
    // Check for discontinuity and truncate data 
    PrxData* it;
    for(it = closest_data; it!= data+(read_end-1); it++){
        if(abs((it+1)->distance - it->distance) > _discontinuity_threshold.get()){
            read_end = (it+1) - data;
            break;
        }
    }
    
    for(it = closest_data; it!= data; it--){
        if(abs((it-1)->distance - it->distance) > _discontinuity_threshold.get()){
            read_start = it-data;
            break;
        }
    }
    // Check if the min_distance is a corner
    if(closest_data - (data+read_start) > 2 && (data+read_end) - closest_data > 2){
        // Using the 3-point endpoint approximation for the derivative
        float fwd = 0.5*(-3*closest_data->distance + 4*(closest_data+1)->distance - (closest_data+2)->distance);
        float bkd = 0.5*(3*closest_data->distance - 4*(closest_data-1)->distance + (closest_data-2)->distance);

        // If the closest point is a corner remove all other points and return
        if(fwd - bkd > _corner_threshold.get()){
            read_start = closest_data - data;
            read_end = read_start+1;
        }
        return;
    }

    // Check if the closest point is at the end of a data set
    if(closest_data - (data+read_start) <= 2){
        read_end = read_start+2;
        return;
    }
    else if((data+read_end) - closest_data <= 2){
        read_start = read_end-2;
        return;
    }

return;
}


void AP_Proximity_CurveFit::compute_coefficients(AP_Proximity_CurveFit::Coefficients &c)
{   
    for(int it = read_start; it < read_end; it++){
        float x = data[it].position.x - reference_point.x;
        float y = data[it].position.y - reference_point.y;
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

    center_type = AP_Proximity_CurveFit::CenterType::NONE;
    closest_distance = 0.0;
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