#pragma once
#include "AP_Proximity_config.h"
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>

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
        LINE,
    } CenterType;

public:

    AP_Proximity_CurveFit();

    // Set "heading" to center in Rad and "distance" to surface in m given current position from EKF origin.
    // Heading and distance are unchanged if curve fit failed
    bool get_target(float &heading, float &distance, Vector2f &tangent_vec, Vector2f &normal_vec, Vector2f &center);

    // Add coordinate of obstacle to data set given yaw "angle" to obstacle in Radians 
    // "distance" to obstacle in meters "current_position" in meters from and vehicle "yaw" attitude in Radians 
    void add_point(float angle_deg, float distance);

    // Clear the data vector and set _fit_type to NONE.
    void reset();

    // Log the current state
    void log_fit(Vector2f center, Vector2f normal_vec, float fit_quality, int fit_num);

    // Log the heading and distance
    void log_target(const float heading, const float distance);

    static const struct AP_Param::GroupInfo var_info[];

    inline int get_center_type() const {return _fit_type;};
    inline Vector2f get_center() const {return _center;};
    inline bool angle_within_bound(float angle_deg) const {return angle_deg > _angle_min_deg.get() && angle_deg < _angle_max_deg.get();};
    
private:

    // Return true if fit was successful given a reference point.
    // Fit a Circle or a line through the data and find the center of curvature.
    // Fit is performed only if there is new data (ie. _fit_type is NONE).
    // set reference_point (origin) of the fit.
    bool compute_fit(Vector2f curr_pos,
        Vector2f& tangent_vec, Vector2f& normal_vec, Vector2f& center, 
        float& fit_quality, AP_Proximity_CurveFit::CenterType& fit_type, int &fit_num);

    // Clean up data by removing jump discontinuities;
    void truncate_data(Vector2f curr_pos);

    // set coefficients structure using truncated data.
    void compute_coefficients(AP_Proximity_CurveFit::Coefficients &c);

    // return true if circle could be fit through data.
    // set center and radius properties
    bool solve_circle(const AP_Proximity_CurveFit::Coefficients c);

    // return true if line could be fit through data.
    // set center and radius properties
    bool solve_line(AP_Proximity_CurveFit::Coefficients c, Vector2f curr_pos,
        Vector2f& tangent_vec, Vector2f& normal_vec, Vector2f& center, 
        float& fit_quality) ;

    bool solve_point(Vector2f curr_pos, int fit_num,
        Vector2f& tangent_vec, Vector2f& normal_vec, Vector2f& center, 
        float& fit_quality);

    int find_closest_index(const Vector2f curr_pos);

    Vector2f _points_NE_origin[2*CURVEFIT_DATA_LEN];
    int _read_start = 0;
    int _read_end = 0;

    int _write_start = CURVEFIT_DATA_LEN;
    int _write_end = CURVEFIT_DATA_LEN;

    float _last_angle;
    uint8_t _last_dir;
    bool _reset_flag;
    bool _first_time_range_error{true};

    uint8_t _min_req_for_line{3};

    Vector2f _center; // coordinates of the center of the target 
    
    AP_Proximity_CurveFit::CenterType _fit_type{AP_Proximity_CurveFit::CenterType::NONE};

    Vector2f _normal_vec;
    Vector2f _tangent_vec;

    //Parameters
    AP_Float _discontinuity_threshold;
    AP_Float _angle_min_deg;
    AP_Float _angle_max_deg;
    AP_Float _rng_max_m;
    AP_Float _rng_min_m;
    AP_Float _lidar_sweep_rate_hz;
    AP_Float _center_filter_cutoff_hz;

    float _fit_quality;
    int _fit_num;

    LowPassFilterVector2f _center_filter;
};

#endif