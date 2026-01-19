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

Example usage:
./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:ms200p --speedup=1 -l 51.8752066,14.6487840,54.15,0

param set SERIAL5_PROTOCOL 11  # proximity
param set PRX1_TYPE 18  # MS200P
reboot

arm throttle
rc 3 1600

# for avoidance:
param set DISARM_DELAY 0
param set AVOID_ENABLE 2 # use proximity sensor
param set AVOID_MARGIN 2.00  # 2m
param set AVOID_BEHAVE 0 # slide
reboot
mode loiter
arm throttle
rc 3 1600
rc 3 1500
rc 2 1450
*/

#pragma once

#include "SIM_SerialProximitySensor.h"

#ifndef HAL_SIM_PS_MS200P_ENABLED
#define HAL_SIM_PS_MS200P_ENABLED HAL_SIM_SERIALPROXIMITYSENSOR_ENABLED
#endif

#if HAL_SIM_PS_MS200P_ENABLED

namespace SITL {

class PS_MS200P : public SerialProximitySensor {
public:

    using SerialProximitySensor::SerialProximitySensor;

    uint32_t packet_for_location(const Location &location,
                                 uint8_t *data,
                                 uint8_t buflen) override;

    void update(const Location &location) override;

private:

    // MS200P packet structure
    // Total size must be exactly 47 bytes:
    // 1 (header) + 1 (point_count) + 2 (speed) + 2 (start_angle) +
    // 36 (12 points * 3 bytes) + 2 (end_angle) + 2 (timestamp) + 1 (crc) = 47
    struct PACKED MS200PPacket {
        uint8_t header;                       // Frame header (0x54)
        uint8_t point_count;                  // Number of points (12)
        uint16_t speed;                       // Rotation speed in deg/s
        uint16_t start_angle;                 // Start angle in 0.01 degrees

        struct PACKED PointData {
            uint16_t distance;                // Distance in mm
            uint8_t intensity;                // Intensity (0-255)
        } points[12];

        uint16_t end_angle;                   // End angle in 0.01 degrees
        uint16_t timestamp;                   // Timestamp in ms
        uint8_t crc;                          // CRC-8 checksum

        // Calculate CRC-8 for the packet
        uint8_t calculate_crc() const;

        // Get packet size in bytes
        static constexpr uint32_t packet_size() {
            return sizeof(MS200PPacket);
        }
    };

    // CRC-8 lookup table
    static const uint8_t CRC_TABLE[256];

    // Calculate CRC-8 for data
    static uint8_t get_crc8(const uint8_t *data, uint16_t len);

    // Scan state
    float last_angle_deg { 0.0f };           // Last scan angle (unused, kept for compatibility)
    uint32_t last_scan_iter { 0 };           // Iteration counter for batch processing
    const uint32_t batch_size { 10 };        // Number of packets to send per update (avoid overwhelming serial buffer)
    uint16_t packet_timestamp { 0 };         // Rolling timestamp counter
};

};

#endif  // HAL_SIM_PS_MS200P_ENABLED
