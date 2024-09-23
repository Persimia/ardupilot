#include "AP_Proximity_CurveFit.h"

#define LARGE_FLOAT 1.0e3

int AP_Proximity_CurveFit::compute_curvature_center(Vector2f &center_curvature)
{
    if(n < 1.0){
        return 0;
    }

    if(n < 5.0){
        solve_centroid(center_curvature);
        center_curvature.x += x_ref;
        center_curvature.y += y_ref;
        return 1;
    }

    if(solve_circle(center_curvature)){
        center_curvature.x += x_ref;
        center_curvature.y += y_ref;
        return 3;
    }

    if(solve_line(center_curvature)){
        center_curvature.x += x_ref;
        center_curvature.y += y_ref;
        return 2;
    }

    return 0;

}

bool AP_Proximity_CurveFit::solve_circle(Vector2f &center)
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
    if (n < 4.0){
        return false;
    }

    Matrix3d A = {{2*Sum_x2, 2*Sum_xy, Sum_x},
                  {2*Sum_xy, 2*Sum_y2, Sum_y},
                  {2*Sum_x,  2*Sum_y,  n    }};
    
    Matrix3d Ainv;
    // Check if the problem is singular
    if(!A.inverse(Ainv)){
        return false;
    }

    Vector3d B = {Sum_x3 + Sum_xy2, Sum_yx2 + Sum_y3, Sum_x2 + Sum_y2};

    Vector3d solution = Ainv*B;
    center.x = solution.x;
    center.y = solution.y;
    return true;
}

bool AP_Proximity_CurveFit::solve_line(Vector2f &normal)
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
    if (n < 3.0){
        return false;
    }

    float det = Sum_x2 * Sum_y2 - Sum_xy * Sum_xy; 

    if (abs(det) < 1e-6){
        return false;
    }

    normal.x = (Sum_y2*Sum_x - Sum_xy*Sum_y)/det;
    normal.y = (Sum_x2*Sum_y - Sum_xy*Sum_x)/det;

    normal.normalize();
    return true;
}

void AP_Proximity_CurveFit::solve_centroid(Vector2f &centroid)
{
    centroid.x = Sum_x/n;
    centroid.y = Sum_y/n;
}


void AP_Proximity_CurveFit::add_point(float angle, float distance, Vector3f current_position, float yaw)
{

    if (n < 1.0){
        x_ref = current_position.x;
        y_ref = current_position.y;
    }

    double x = distance*cosf(angle+yaw) + current_position.x - x_ref;
    double y = distance*sinf(angle+yaw) + current_position.y - y_ref;

    n += 1.0;
    Sum_x += x;
    Sum_y += y;
    Sum_x2 += x*x;
    Sum_xy += x*y;
    Sum_y2 += y*y;
    Sum_xy2 += x*y*y;
    Sum_yx2 += y*x*x;
    Sum_x3 += x*x*x;
    Sum_y3 += y*y*y;
}

void AP_Proximity_CurveFit::reset()
{
    n = 0.0;
    Sum_x = 0.0;
    Sum_y = 0.0;
    Sum_x2 = 0.0;
    Sum_xy = 0.0;
    Sum_y2 = 0.0;
    Sum_xy2 = 0.0;
    Sum_yx2 += 0.0;
    Sum_x3 += 0.0;
    Sum_y3 += 0.0;

    x_ref = 0.0;
    y_ref = 0.0;
}