/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   The Oradar MS200P serial interface based on user manual PD-P2117008
*/
#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_MS200P_ENABLED

#include "AP_Proximity_Backend_Serial.h"

class AP_Proximity_MS200P : public AP_Proximity_Backend_Serial
{

public:
    // constructor
    AP_Proximity_MS200P(AP_Proximity &_frontend,
                        AP_Proximity::Proximity_State &_state,
                        AP_Proximity_Params& _params,
                        uint8_t serial_instance) :
            AP_Proximity_Backend_Serial(_frontend, _state, _params, serial_instance) {}

    uint16_t rxspace() const override {
        return 512;
    };

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 12.0f; }
    float distance_min() const override { return 0.03f; }

private:

    // parse data from serial port
    void parse_point_cloud_data();

    // process one byte received on serial port
    // returns true if a complete packet has been received
    bool parse_byte(uint8_t b);

    // CRC-8 calculation for data validation
    uint8_t get_crc8(const uint8_t *data, uint16_t len);

    enum class ParseState {
        WAITING_HEADER,
        WAITING_POINT_COUNT,
        WAITING_SPEED,
        WAITING_START_ANGLE,
        WAITING_POINT_DATA,
        WAITING_END_ANGLE,
        WAITING_TIMESTAMP,
        WAITING_CRC
    } _parse_state = ParseState::WAITING_HEADER;

    // packet parsing state
    uint8_t _packet_buffer[256];
    uint16_t _packet_buffer_len = 0;
    uint8_t _num_points = 0;
    uint16_t _current_point = 0;
    uint16_t _start_angle = 0;  // in 0.01 degrees
    uint16_t _end_angle = 0;    // in 0.01 degrees
    uint16_t _speed = 0;        // rotation speed in degrees/s
    uint32_t _timestamp = 0;    // timestamp in ms

    // timing variables
    uint32_t _last_distance_received_ms = 0;

    // for combining readings within mini sectors
    static constexpr float PROXIMITY_MS200P_COMBINE_READINGS_DEG = 5.0f;
    uint8_t _current_minisector = UINT8_MAX;
    float _minisector_distance_m = 0;
    float _minisector_angle_deg = 0;
    bool _minisector_valid = false;

    // for tracking face boundaries
    AP_Proximity_Boundary_3D::Face _current_face;
    float _face_distance_m = 0;
    float _face_yaw_deg = 0;
    bool _face_valid = false;

    // CRC lookup table for CRC-8
    static const uint8_t CRC_TABLE[256];
};

#endif // AP_PROXIMITY_MS200P_ENABLED
