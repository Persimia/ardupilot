#pragma once
#include "AP_Proximity_config.h"
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#define CURVEFIT_DATA_LEN 150

#if AP_PROXIMITY_CURVEFIT_ENABLED

class AP_Proximity_CurveFit
{
    typedef struct{
        Vector2f position;
        float distance;
    }PrxData;
    typedef struct{
        float Sum_x3{0.0};
        float Sum_y3{0.0};
        float Sum_xy2{0.0};
        float Sum_yx2{0.0};

        float Sum_x2{0.0};
        float Sum_y2{0.0};
        float Sum_xy{0.0};
        float Sum_x{0.0};
        float Sum_y{0.0};
        int32_t n{0};
    } Coefficients;
    typedef enum{
        NONE,
        POINT,
        CIRCLE_CONVEX,
        LINE,
        CIRCLE_CONCAVE
    } CenterType;

public:

    AP_Proximity_CurveFit();

    // Set "heading" to center in Rad and "distance" to surface in m given current position from EKF origin.
    // Heading and distance are unchanged if curve fit failed
    void get_target(float &heading, float &distance, const Vector2f curr_pos);

    // Add coordinate of obstacle to data set given yaw "angle" to obstacle in Radians 
    // "distance" to obstacle in meters "current_position" in meters from and vehicle "yaw" attitude in Radians 
    void add_point(float angle_deg, float distance);

    // Clear the data vector and set fit_type to NONE.
    void reset();

    // Log the current state
    void log_fit();

    // Log the heading and distance
    void log_target(const float heading, const float distance);

    static const struct AP_Param::GroupInfo var_info[];
    
private:

    // Return true if fit was successful given a reference point.
    // Fit a Circle or a line through the data and find the center of curvature.
    // Fit is performed only if there is new data (ie. fit_type is NONE).
    // set reference_point (origin) of the fit.
    bool compute_curvature_center(Vector2f reference);

    // Clean up data by removing jump discontinuities and isolating corner points;
    // set closest_point property
    void filter_data();

    // set coefficients structure using filtered data.
    void compute_coefficients(AP_Proximity_CurveFit::Coefficients &c);

    // return true if circle could be fit through data.
    // set center and radius properties
    bool solve_circle(const AP_Proximity_CurveFit::Coefficients c);

    // return true if line could be fit through data.
    // set center and radius properties
    bool solve_line(const AP_Proximity_CurveFit::Coefficients c);

    PrxData data[2*CURVEFIT_DATA_LEN];
    int read_start = 0;
    int read_end = 0;
    int write_start = CURVEFIT_DATA_LEN;
    int write_end = CURVEFIT_DATA_LEN;
    float _last_angle;

    Vector2f reference_point; // coordinates in global frame of fit origin.
    Vector2f closest_point; // closest point in global frame of closest point.
    float closest_distance;
    Vector2f center; // coordinates in fit frame of center. For a circle this is the center. For a line this is a vector alligned with normal.
    float radius; // radius in m. For circle this is radius. For line this is length of the normal vector.
    AP_Proximity_CurveFit::CenterType center_type{AP_Proximity_CurveFit::CenterType::NONE};

    AP_Float _flatness_threshold;  // Threshold for det(A) thet should be counted as flat
    AP_Float _discontinuity_threshold; // units m default 2
    AP_Float _corner_threshold; //{0.2}
    AP_Float _angle_min_deg; //{-22.5}
    AP_Float _angle_max_deg; // {22.5}

};

#endif