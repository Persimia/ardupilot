#pragma once
#include "AP_Proximity_config.h"
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#define CURVEFIT_DATA_LEN 150

#if AP_PROXIMITY_CURVEFIT_ENABLED

class AP_Proximity_CurveFit
{
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
        CIRCLE_CONCAVE,
        CORNER,
    } CenterType;

public:

    AP_Proximity_CurveFit();

    // Set "heading" to center in Rad and "distance" to surface in m given current position from EKF origin.
    // Heading and distance are unchanged if curve fit failed
    bool get_target(float &heading, float &distance);

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

    inline int get_center_type() const {return center_type;};
    inline Vector2f get_center() const {return center;};
    inline float get_radius() const {return radius;};
    inline bool angle_within_bound(float angle_deg) const {return angle_deg > _angle_min_deg.get() && angle_deg < _angle_max_deg.get();};
    
private:

    // Return true if fit was successful given a reference point.
    // Fit a Circle or a line through the data and find the center of curvature.
    // Fit is performed only if there is new data (ie. fit_type is NONE).
    // set reference_point (origin) of the fit.
    bool compute_curvature_center(Vector2f reference);

    // Clean up data by removing jump discontinuities;
    void truncate_data();

    // Return true if the closest point is a corner
    bool detect_corner();

    // set coefficients structure using truncated data.
    void compute_coefficients(AP_Proximity_CurveFit::Coefficients &c);

    // return true if circle could be fit through data.
    // set center and radius properties
    bool solve_circle(const AP_Proximity_CurveFit::Coefficients c);

    // return true if line could be fit through data.
    // set center and radius properties
    bool solve_line(const AP_Proximity_CurveFit::Coefficients c);

    void find_closest_index(const Vector2f reference);

    Vector2f data[2*CURVEFIT_DATA_LEN];
    int read_start = 0;
    int read_end = 0;
    int closest_index = 0;

    int write_start = CURVEFIT_DATA_LEN;
    int write_end = CURVEFIT_DATA_LEN;
    int closest_index_ = CURVEFIT_DATA_LEN;

    float last_angle;
    float last_distance;
    float avg_count = 1.0f;
    float closest_distance_;

    Vector2f reference_point; // coordinates in global frame of fit origin.
    Vector2f center; // coordinates in fit frame of center. For a circle this is the center. For a line this is a vector alligned with normal.
    float radius; // radius in m. For circle this is radius. For line this is length of the normal vector.
    AP_Proximity_CurveFit::CenterType center_type{AP_Proximity_CurveFit::CenterType::NONE};

    //Parameters
    AP_Int16 _min_pts;
    AP_Float _flatness_threshold;  // Threshold for det(A)/n^3 that should be counted as flat
    AP_Float _discontinuity_threshold;
    AP_Float _corner_threshold;
    AP_Int16 _corner_window;
    AP_Float _angle_min_deg;
    AP_Float _angle_max_deg;
    AP_Float _rng_max_m;
    AP_Float _rng_min_m;



};

#endif