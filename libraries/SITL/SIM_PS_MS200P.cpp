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
*/
/*
  Simulator for the Oradar MS200P proximity sensor
*/

#include "SIM_PS_MS200P.h"

#if HAL_SIM_PS_MS200P_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <AP_DDS/AP_DDS_Client.h>

using namespace SITL;

// CRC-8 lookup table (same as in AP_Proximity_MS200P.cpp)
const uint8_t PS_MS200P::CRC_TABLE[256] = {
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

// Calculate CRC-8 for data
uint8_t PS_MS200P::get_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = CRC_TABLE[(crc ^ data[i]) & 0xff];
    }
    return crc;
}

// Calculate CRC for packet
uint8_t PS_MS200P::MS200PPacket::calculate_crc() const
{
    return get_crc8((const uint8_t*)this, sizeof(MS200PPacket) - 1);
}

uint32_t PS_MS200P::packet_for_location(const Location &location,
                                         uint8_t *data,
                                         uint8_t buflen)
{
    return 0;  // Not used - we use update() instead
}

void PS_MS200P::update(const Location &location)
{
    // Check if scan data is already processed
    if (AP_DDS_Client::rx_laser_scan_used) {
        return;
    }

    // Get laser scan data from ROS2/DDS
    const sensor_msgs_msg_LaserScan* rx_laser_scan_topic = AP_DDS_Client::getLaserScanData();
    if (rx_laser_scan_topic == nullptr) {
        static uint32_t last_warn_ms = 0;
        if (AP_HAL::millis() - last_warn_ms > 5000) {
            ::fprintf(stderr, "MS200P SITL: No laser scan data available from ROS2/DDS\n");
            last_warn_ms = AP_HAL::millis();
        }
        return;
    }

    const uint32_t sample_count = rx_laser_scan_topic->ranges_size;
    const float angle_min = rx_laser_scan_topic->angle_min;
    // const float angle_max = rx_laser_scan_topic->angle_max;
    const float angle_increment = rx_laser_scan_topic->angle_increment;
    const float max_range = rx_laser_scan_topic->range_max;
    const float min_range = rx_laser_scan_topic->range_min;

    // Debug: Print scan parameters once
    static bool printed_params = false;
    if (!printed_params) {
        ::fprintf(stderr, "MS200P SITL: LaserScan has %u samples, angle_min=%.3f, angle_max=%.3f, increment=%.4f\n",
                 sample_count, angle_min, rx_laser_scan_topic->angle_max, angle_increment);
        // Check if ranges array has trailing zeros
        uint32_t last_valid_idx = 0;
        for (uint32_t i = 0; i < sample_count; i++) {
            float range = rx_laser_scan_topic->ranges[i];
            // Count as valid if non-zero OR infinite/nan (inf means no obstacle detected)
            if (range > 0.001f || !isfinite(range)) {
                last_valid_idx = i;
            }
        }
        ::fprintf(stderr, "MS200P SITL: Last valid range at index %u (total %u)\n", last_valid_idx, sample_count);
        // printed_params = true;
    }

    // Send batch_size packets per update (like SF45B does)
    for (uint32_t i = 0; i < batch_size; i++) {
        // Check if we've completed this scan
        if (last_scan_iter >= sample_count) {
            AP_DDS_Client::rx_laser_scan_used = true;
            last_scan_iter = 0;
            break;
        }

        // Determine start index for this packet
        const uint32_t start_idx = last_scan_iter;

        // Create MS200P packet with 12 points - initialize all fields
        MS200PPacket packet;
        memset(&packet, 0, sizeof(packet));
        packet.header = 0x54;
        packet.point_count = 12;
        packet.speed = 3590;  // Real MS200P speed: ~3590 deg/s (10 rev/s)

        // Calculate start angle in 0.01 degrees
        // ROS2 LaserScan: angle_min to angle_max, CCW from front (typically -π to +π)
        // MS200P: 0° = front, CW, in 0.01° units (0-35999)
        // Negate to convert CCW → CW
        float start_angle_rad = angle_min + (angle_increment * start_idx);
        float start_angle_deg = -(RAD_TO_DEG * start_angle_rad);

        // Normalize to 0-360° range (MS200P uses 0° = front)
        while (start_angle_deg < 0.0f) {
            start_angle_deg += 360.0f;
        }
        while (start_angle_deg >= 360.0f) {
            start_angle_deg -= 360.0f;
        }

        packet.start_angle = static_cast<uint16_t>(start_angle_deg * 100.0f);

        // Fill in 12 points
        for (uint8_t pt = 0; pt < 12; pt++) {
            uint32_t idx = start_idx + pt;
            if (idx >= sample_count) {
                // Out of bounds, use invalid data
                packet.points[pt].distance = 0;
                packet.points[pt].intensity = 0;  // Error code
                continue;
            }

            // Get distance from laser scan
            float distance = rx_laser_scan_topic->ranges[idx];

            // Check if distance is valid
            if (distance > max_range || distance < min_range || !isfinite(distance)) {
                packet.points[pt].distance = 0;
                packet.points[pt].intensity = 0;  // Error code
            } else {
                // Convert distance to mm
                packet.points[pt].distance = static_cast<uint16_t>(distance * 1000.0f);
                // Use high intensity for valid measurements (range 16-255, per MS200P manual)
                packet.points[pt].intensity = 200;
            }
        }

        // Calculate end angle in 0.01 degrees
        uint32_t end_idx = start_idx + 11;
        if (end_idx >= sample_count) {
            end_idx = sample_count - 1;
        }
        float end_angle_rad = angle_min + (angle_increment * end_idx);
        float end_angle_deg = -(RAD_TO_DEG * end_angle_rad);  // Negate: CCW → CW

        // Normalize to 0-360° range
        while (end_angle_deg < 0.0f) {
            end_angle_deg += 360.0f;
        }
        while (end_angle_deg >= 360.0f) {
            end_angle_deg -= 360.0f;
        }

        packet.end_angle = static_cast<uint16_t>(end_angle_deg * 100.0f);

        // Set timestamp (rolling counter in ms, 0~29999 per MS200P manual)
        packet.timestamp = packet_timestamp;
        packet_timestamp += 10;  // Increment by ~10ms per packet
        if (packet_timestamp >= 30000) {
            packet_timestamp = 0;
        }

        // Calculate and set CRC
        packet.crc = packet.calculate_crc();

        // Debug: Print packets with close obstacles (distance < 6m)
        static uint32_t packet_count = 0;
        bool has_close_obstacle = false;
        float min_dist = 999.0f;
        for (uint8_t p = 0; p < 12; p++) {
            if (packet.points[p].distance > 0 && packet.points[p].distance < 6000) {
                has_close_obstacle = true;
                float dist_m = packet.points[p].distance / 1000.0f;
                if (dist_m < min_dist) {
                    min_dist = dist_m;
                }
            }
        }
        if (has_close_obstacle && packet_count < 10) {
            // Calculate ROS2 angle for comparison
            float ros2_angle_deg = RAD_TO_DEG * (angle_min + angle_increment * start_idx);
            ::fprintf(stderr, "MS200P SITL: Close obstacle #%u: AP_angle=%.1f° ROS2_angle=%.1f° iter=%u min_dist=%.2fm\n",
                     packet_count++, packet.start_angle/100.0f, ros2_angle_deg,
                     start_idx, min_dist);
        }

        // Send packet to autopilot (like SF45B's send() method)
        const ssize_t ret = write_to_autopilot((const char*)&packet, sizeof(packet));
        if (ret < 0 || (uint32_t)ret != sizeof(packet)) {
            // Write failed - just skip this packet and continue
            // (SF45B doesn't abort on write failures either)
        }

        // Move to next packet (12 points)
        last_scan_iter += 12;
    }
}

#endif  // HAL_SIM_PS_MS200P_ENABLED
