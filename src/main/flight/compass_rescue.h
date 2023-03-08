/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

#include "common/axis.h"

#include "pg/compass_rescue.h"

#define TASK_GPS_RESCUE_RATE_HZ 100  // in sync with altitude task rate

typedef enum {
    COMPASS_RESCUE_ALT_MODE_MAX = 0,  // Use max recorded alt
    COMPASS_RESCUE_ALT_MODE_MIN,      // Fly no lower than min alt, if current is higher use it
    COMPASS_RESCUE_ALT_MODE_CURRENT,  // Just use this alt, no ascend
    COMPASS_RESCUE_ALT_MODE_COUNT
} compassRescueAltitudeMode_e;

extern float compassRescueAngle[ANGLE_INDEX_COUNT]; // NOTE: ANGLES ARE IN CENTIDEGREES

void compassRescueInit(void);
void compassRescueUpdate(void);
void compassRescueNewGpsData(void);

float compassRescueGetYawRate(void);
float compassRescueGetThrottle(void);
bool compassRescueIsConfigured(void);
bool compassRescueIsAvailable(void);
