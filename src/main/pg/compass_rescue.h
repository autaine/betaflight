/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef struct compassRescue_s {

    uint16_t ascendRate;            // cm/s, for altitude corrections on ascent
    uint16_t rescueAltitudeBufferM; // meters
    uint16_t minReturnAltitudeM;    // meters
    uint16_t direction;             // that's direction home (approx)
    uint16_t angle;                 // that's pitch angle == velocity when flying home
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint16_t throttleHover;
    uint8_t  throttleP, throttleI, throttleD;
    uint8_t  yawP;
    uint8_t  rollMix;
    uint8_t  altitudeMode;

} compassRescueConfig_t;

PG_DECLARE(compassRescueConfig_t, compassRescueConfig);
