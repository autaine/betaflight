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

#include "platform.h"

#ifdef USE_MAG

#include "flight/compass_rescue.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "compass_rescue.h"

PG_REGISTER_WITH_RESET_TEMPLATE(compassRescueConfig_t, compassRescueConfig, PG_COMPASS_RESCUE, 1);

PG_RESET_TEMPLATE(compassRescueConfig_t, compassRescueConfig,

    .direction = 0,

    .altitudeMode = COMPASS_RESCUE_ALT_MODE_MIN,
    .rescueAltitudeBufferM = 1,
    .minReturnAltitudeM = 100,
    .ascendRate = 500, 

    .angle = 40,

    .throttleMin = 1100,
    .throttleMax = 1600,
    .throttleHover = 1275,

    .throttleP = 15,
    .throttleI = 15,
    .throttleD = 15,

    .yawP = 20,
    .rollMix = 150
);

#endif // USE_MAG
