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
   Serial communication: 230400 baud, 8 data bits, 1 stop bit, no parity
*/

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_MS200P_ENABLED

#include "AP_Proximity_MS200P.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

static const uint32_t PROXIMITY_MS200P_TIMEOUT_MS = 2000;

// CRC-8 lookup table
const uint8_t AP_Proximity_MS200P::CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

// update the state of the sensor
void AP_Proximity_MS200P::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    // process incoming data
    uint32_t nbytes = MIN(_uart->available(), 1024U);
    while (nbytes-- > 0) {
        uint8_t c;
        if (!_uart->read(c)) {
            continue;
        }
        if (parse_byte(c)) {
            parse_point_cloud_data();
        }
    }

    // check for timeout and set health status
    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_MS200P_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

// calculate CRC-8 for data validation
uint8_t AP_Proximity_MS200P::get_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = CRC_TABLE[(crc ^ data[i]) & 0xff];
    }
    return crc;
}

// parse one byte from serial port
// returns true when a complete packet is received
bool AP_Proximity_MS200P::parse_byte(uint8_t b)
{
    switch (_parse_state) {
        case ParseState::WAITING_HEADER:
            if (b == 0x54) {
                _packet_buffer[0] = b;
                _packet_buffer_len = 1;
                _parse_state = ParseState::WAITING_POINT_COUNT;
            }
            break;

        case ParseState::WAITING_POINT_COUNT:
            _packet_buffer[_packet_buffer_len++] = b;
            _num_points = b & 0x1F;  // Lower 5 bits indicate number of points (currently fixed at 12)
            if (_num_points == 12) {
                _parse_state = ParseState::WAITING_SPEED;
            } else {
#if HAL_GCS_ENABLED
                static uint32_t last_bad_count_ms = 0;
                if (AP_HAL::millis() - last_bad_count_ms > 2000) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MS200P: Bad point count %u (0x%02x)",
                                  (unsigned)_num_points, (unsigned)b);
                    last_bad_count_ms = AP_HAL::millis();
                }
#endif
                // Invalid point count, reset
                _parse_state = ParseState::WAITING_HEADER;
            }
            break;

        case ParseState::WAITING_SPEED:
            _packet_buffer[_packet_buffer_len++] = b;
            if (_packet_buffer_len == 4) {
                _speed = UINT16_VALUE(_packet_buffer[3], _packet_buffer[2]);
                _parse_state = ParseState::WAITING_START_ANGLE;
            }
            break;

        case ParseState::WAITING_START_ANGLE:
            _packet_buffer[_packet_buffer_len++] = b;
            if (_packet_buffer_len == 6) {
                _start_angle = UINT16_VALUE(_packet_buffer[5], _packet_buffer[4]);
                _current_point = 0;
                _parse_state = ParseState::WAITING_POINT_DATA;
            }
            break;

        case ParseState::WAITING_POINT_DATA:
            _packet_buffer[_packet_buffer_len++] = b;
            // Each point is 3 bytes: 2 bytes distance + 1 byte intensity
            if (_packet_buffer_len == 6 + (_num_points * 3)) {
                _parse_state = ParseState::WAITING_END_ANGLE;
            }
            break;

        case ParseState::WAITING_END_ANGLE:
            _packet_buffer[_packet_buffer_len++] = b;
            if (_packet_buffer_len == 6 + (_num_points * 3) + 2) {
                _end_angle = UINT16_VALUE(_packet_buffer[_packet_buffer_len - 1], _packet_buffer[_packet_buffer_len - 2]);
                _parse_state = ParseState::WAITING_TIMESTAMP;
            }
            break;

        case ParseState::WAITING_TIMESTAMP:
            _packet_buffer[_packet_buffer_len++] = b;
            if (_packet_buffer_len == 6 + (_num_points * 3) + 4) {
                _timestamp = UINT16_VALUE(_packet_buffer[_packet_buffer_len - 1], _packet_buffer[_packet_buffer_len - 2]);
                _parse_state = ParseState::WAITING_CRC;
            }
            break;

        case ParseState::WAITING_CRC:
            _packet_buffer[_packet_buffer_len++] = b;
            // Verify CRC
            uint8_t calculated_crc = get_crc8(_packet_buffer, _packet_buffer_len - 1);
            if (calculated_crc == b) {
                // Valid packet received
                _parse_state = ParseState::WAITING_HEADER;
                return true;
            } else {
#if HAL_GCS_ENABLED
                static uint32_t last_crc_error_ms = 0;
                if (AP_HAL::millis() - last_crc_error_ms > 2000) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MS200P: CRC fail calc=0x%02x got=0x%02x len=%u",
                                  (unsigned)calculated_crc, (unsigned)b, (unsigned)_packet_buffer_len);
                    last_crc_error_ms = AP_HAL::millis();
                }
#endif
                // CRC mismatch, reset
                _parse_state = ParseState::WAITING_HEADER;
            }
            break;
    }

    return false;
}

// process point cloud data packet
void AP_Proximity_MS200P::parse_point_cloud_data()
{
    _last_distance_received_ms = AP_HAL::millis();

    // Calculate angle step between points
    int32_t angle_diff = _end_angle - _start_angle;
    if (angle_diff < 0) {
        angle_diff += 36000;  // Handle wrap-around
    }

    float angle_step = 0.0f;
    if (_num_points > 1) {
        angle_step = (float)angle_diff / (_num_points - 1);
    }

    // Process each point in the packet
    for (uint8_t i = 0; i < _num_points; i++) {
        // Extract distance (in mm) and intensity
        uint16_t offset = 6 + (i * 3);
        uint16_t distance_mm = UINT16_VALUE(_packet_buffer[offset + 1], _packet_buffer[offset]);
        uint8_t intensity = _packet_buffer[offset + 2];

        // Calculate angle for this point
        float angle_raw = (_start_angle + (i * angle_step)) * 0.01f;  // Convert to degrees
        float angle_deg = correct_angle_for_orientation(angle_raw);

        // Convert distance to meters
        float distance_m = distance_mm * 0.001f;

        // Ignore invalid readings (intensity 0 or 1 are error codes, 0-15 are reserved)
        if (intensity < 16 || distance_m < distance_min() || distance_m > distance_max()) {
            continue;
        }

        // Check if we should ignore this reading based on user settings
        if (ignore_reading(angle_deg, distance_m)) {
            continue;
        }

#if AP_PROXIMITY_CURVEFIT_ENABLED
        // Add data point for curve fitting
        frontend.curvefit->add_point(angle_deg, distance_m);
#endif

        // Determine which face this reading belongs to
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(angle_deg);

        // Update face if it changed
        if (face != _current_face) {
            // Store previous face data
            if (_face_valid) {
                frontend.boundary.set_face_attributes(_current_face, wrap_360(_face_yaw_deg), _face_distance_m, state.instance);
            } else {
                frontend.boundary.reset_face(_current_face, state.instance);
            }

            // Reset for new face
            _current_face = face;
            _face_yaw_deg = 0;
            _face_distance_m = distance_max();
            _face_valid = false;
        }

        // Update shortest distance for current face
        if (!_face_valid || distance_m < _face_distance_m) {
            _face_yaw_deg = wrap_180(angle_deg);
            _face_distance_m = distance_m;
            _face_valid = true;
        }

        // Determine minisector for obstacle database
        uint8_t minisector = (uint8_t)(wrap_360(angle_deg + (PROXIMITY_MS200P_COMBINE_READINGS_DEG * 0.5f)) / PROXIMITY_MS200P_COMBINE_READINGS_DEG);

        // Update minisector if it changed
        if (minisector != _current_minisector) {
            // Store previous minisector data
            if (_minisector_valid) {
                database_push(_minisector_angle_deg, _minisector_distance_m);
            }

            // Reset for new minisector
            _current_minisector = minisector;
            _minisector_angle_deg = 0;
            _minisector_distance_m = distance_max();
            _minisector_valid = false;
        }

        // Update shortest distance for current minisector
        if (!_minisector_valid || distance_m < _minisector_distance_m) {
            _minisector_angle_deg = angle_deg;
            _minisector_distance_m = distance_m;
            _minisector_valid = true;
        }
    }
}

#endif // AP_PROXIMITY_MS200P_ENABLED
