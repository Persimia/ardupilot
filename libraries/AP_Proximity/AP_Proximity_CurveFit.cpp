#include "AP_Proximity_CurveFit.h"
#include <algorithm>
#define LARGE_FLOAT 1.0e3

void AP_Proximity_CurveFit::get_target(float &heading, float & distance, const Vector2f curr_pos)
{
    if(!compute_curvature_center(curr_pos)){
        return; // Unable to solve heading, distance
    }
    // Vector from vehicle position ro center of curvature;
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
        break;
    }

}


bool AP_Proximity_CurveFit::compute_curvature_center(Vector2f reference)
{
    reference_point = Vector2f(reference);

    if(center_type != AP_Proximity_CurveFit::CenterType::NONE){
        // Nothing to do
        return true;
    }

    if(data.size() < 1){ // No data
        center_type = AP_Proximity_CurveFit::CenterType::NONE;
        return false;
    }
    
    filter_data();

    if(data.size() <= 5){ // Treat as a single point
        center_type = AP_Proximity_CurveFit::CenterType::POINT;
        center = Vector2f(closest_point - reference_point);
        radius = 0.0;
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
        return true;
    }

    if(solve_line(coefficients)){//Try to fit a line
        center_type = AP_Proximity_CurveFit::CenterType::LINE;
        return true;
    }

    // All else fails, take the closest point
    center = Vector2f(closest_point);
    center_type = AP_Proximity_CurveFit::CenterType::POINT;
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
    
    if(A.det() < (1/flatness_threshold)){
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


void AP_Proximity_CurveFit::add_point(float angle, float distance, Vector2f current_position, float yaw)
{
    float angle_deg = wrap_180(angle*RAD_TO_DEG);
    if( angle_deg < angle_max_deg && angle_deg > angle_min_deg)
    {
        PrxData point;
        point.position.x = distance*cosf(angle+yaw) + current_position.x;
        point.position.y = distance*sinf(angle+yaw) + current_position.y;
        point.distance = distance;
        data.push_back(point);
    }
}

void AP_Proximity_CurveFit::filter_data()
{
    // Find the closest point
    auto closest_data = std::min_element(data.begin(), data.end(),
    [](const PrxData a, const PrxData b)->bool {return a.distance<b.distance;});
    closest_point = Vector2f(closest_data->position);

    // Check for discontinuity and truncate data 
    std::vector<AP_Proximity_CurveFit::PrxData>::iterator it;
    for(it = closest_data; it!= data.end(); it++){
        if(abs((it+1)->distance - it->distance) > discontinuity_threshold){
            data.erase(it+1,data.end());
            break;
        }
    }
    int index = 0;
    for(it = closest_data; it!= data.begin(); it--){
        if(abs((it-1)->distance - it->distance) > discontinuity_threshold){
            data.erase(data.begin(),it);
            closest_data = data.begin() + index;
            break;
        }
        index++;
    }
    // Check if the min_distance is a corner
    if(index > 2 && data.size()-index > 2){
        // Using the 3-point endpoint approximation for the derivative
        float fwd = 0.5*(-3*closest_data->distance + 4*(closest_data+1)->distance - (closest_data+2)->distance);
        float bkd = 0.5*(3*closest_data->distance - 4*(closest_data-1)->distance + (closest_data-2)->distance);

        // If the closest point is a corner remove all other points and return
        if(fwd - bkd > corner_threshold){
            data.erase(closest_data+1,data.end());
            data.erase(data.begin(),closest_data);
        }
        return;
    }

    // Check if the closest point is at the end of a data set
    if(index <= 2){
        data.erase(closest_data+1, data.end());
        return;
    }
    else if(data.size()-index <= 2){
        data.erase(data.begin(),closest_data);
        return;
    }

return;
}


void AP_Proximity_CurveFit::compute_coefficients(AP_Proximity_CurveFit::Coefficients &c)
{   
    std::vector<PrxData>::iterator it;
    for(it = data.begin(); it != data.end(); ++it){
        float x = it->position.x - reference_point.x;
        float y = it->position.y - reference_point.y;
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
    data.clear();
    center_type = AP_Proximity_CurveFit::CenterType::NONE;
}