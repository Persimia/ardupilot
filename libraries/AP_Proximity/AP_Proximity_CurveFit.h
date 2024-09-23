#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>

class AP_Proximity_CurveFit
{
public:
   // Return the X,Y coordinates of the center of curvature relative to home in NE frame
    int compute_curvature_center(Vector2f &center_curvature);

    //Set coordinates of center relative to (x_ref, y_ref) in NE coordinates
    bool solve_circle(Vector2f &center);
    //Set normal in NE coordinates
    bool solve_line(Vector2f &normal);
    //Set centroid  in NE coordinates relative to (x_ref, y_ref) in NE coordinates
    void solve_centroid(Vector2f &centroid);

    void add_point(float angle, float distance, Vector3f current_position, float yaw);
    void reset();

private:

double Sum_x3;
double Sum_y3;
double Sum_xy2;
double Sum_yx2;

double Sum_x2;
double Sum_y2;
double Sum_xy;
double Sum_x;
double Sum_y;
double n;

float x_ref;
float y_ref;
};