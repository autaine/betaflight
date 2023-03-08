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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_MAG

#include "cli/settings.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_compass_rescue.h"

#include "config/feature.h"

#include "config/config.h"

#include "flight/compass_rescue.h"

static uint16_t compassRescueConfig_direction;
static uint8_t compassRescueConfig_altitudeMode;
static uint16_t compassRescueConfig_rescueAltitudeBufferM; // meters
static uint16_t compassRescueConfig_minReturnAltitudeM; // meters
static uint16_t compassRescueConfig_ascendRate;

static uint16_t compassRescueConfig_angle; //degrees

static uint16_t compassRescueConfig_throttleMin;
static uint16_t compassRescueConfig_throttleMax;
static uint16_t compassRescueConfig_throttleHover;

static uint8_t compassRescueConfig_throttleP, compassRescueConfig_throttleI, compassRescueConfig_throttleD;
static uint8_t compassRescueConfig_yawP;
static uint8_t compassRescueConfig_rollMix;
//static uint8_t compassRescueConfig_velP, compassRescueConfig_velI, compassRescueConfig_velD;


static const void *cms_menuCompassRescuePidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    compassRescueConfig_throttleP = compassRescueConfig()->throttleP;
    compassRescueConfig_throttleI = compassRescueConfig()->throttleI;
    compassRescueConfig_throttleD = compassRescueConfig()->throttleD;

    compassRescueConfig_yawP = compassRescueConfig()->yawP;

    compassRescueConfig_rollMix = compassRescueConfig()->rollMix;

    return NULL;
}

static const void *cms_menuCompassRescuePidOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    compassRescueConfigMutable()->throttleP = compassRescueConfig_throttleP;
    compassRescueConfigMutable()->throttleI = compassRescueConfig_throttleI;
    compassRescueConfigMutable()->throttleD = compassRescueConfig_throttleD;

    compassRescueConfigMutable()->yawP = compassRescueConfig_yawP;

    compassRescueConfigMutable()->rollMix = compassRescueConfig_rollMix;

    return NULL;
}

const OSD_Entry cms_menuCompassRescuePidEntries[] =
{
    {"--- CMP RESCUE PID---", OME_Label, NULL, NULL},

    { "THROTTLE P",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &compassRescueConfig_throttleP, 0, 255, 1 } },
    { "THROTTLE I",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &compassRescueConfig_throttleI, 0, 255, 1 } },
    { "THROTTLE D",        OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &compassRescueConfig_throttleD, 0, 255, 1 } },

    { "YAW P",             OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &compassRescueConfig_yawP, 0, 255, 1 } },

    { "ROLL MIX",          OME_UINT8 | REBOOT_REQUIRED, NULL, &(OSD_UINT8_t){ &compassRescueConfig_rollMix, 0, 255, 1 } },

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cms_menuCompassRescuePid = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUGPSRPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cms_menuCompassRescuePidOnEnter,
    .onExit = cms_menuCompassRescuePidOnExit,
    .onDisplayUpdate = NULL,
    .entries = cms_menuCompassRescuePidEntries,
};

static const void *cmsx_menuCompassRescueOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    compassRescueConfig_direction = compassRescueConfig()->direction;
    compassRescueConfig_altitudeMode = compassRescueConfig()->altitudeMode;
    compassRescueConfig_rescueAltitudeBufferM = compassRescueConfig()->rescueAltitudeBufferM;
    compassRescueConfig_minReturnAltitudeM = compassRescueConfig()->minReturnAltitudeM;
    compassRescueConfig_ascendRate = compassRescueConfig()->ascendRate;

    compassRescueConfig_angle = compassRescueConfig()->angle;

    compassRescueConfig_throttleMin = compassRescueConfig()->throttleMin;
    compassRescueConfig_throttleMax = compassRescueConfig()->throttleMax;
    compassRescueConfig_throttleHover = compassRescueConfig()->throttleHover;

    return NULL;
}

static const void *cmsx_menuCompassRescueOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    compassRescueConfigMutable()->direction = compassRescueConfig_direction;
    compassRescueConfigMutable()->altitudeMode = compassRescueConfig_altitudeMode;
    compassRescueConfigMutable()->rescueAltitudeBufferM = compassRescueConfig_rescueAltitudeBufferM;
    compassRescueConfigMutable()->minReturnAltitudeM = compassRescueConfig_minReturnAltitudeM;
    compassRescueConfigMutable()->ascendRate = compassRescueConfig_ascendRate;

    compassRescueConfigMutable()->angle = compassRescueConfig_angle;

    compassRescueConfigMutable()->throttleMin = compassRescueConfig_throttleMin;
    compassRescueConfigMutable()->throttleMax = compassRescueConfig_throttleMax;
    compassRescueConfigMutable()->throttleHover = compassRescueConfig_throttleHover;

    return NULL;
}

const OSD_Entry cmsx_menuCompassRescueEntries[] =
{
    {"- COMPASS  RESCUE -", OME_Label, NULL, NULL},

    { "DIRECTION 0=N   M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &compassRescueConfig_direction, 0, 360, 1 } },
    { "ALTITUDE MODE"    , OME_TAB    | REBOOT_REQUIRED, NULL, &(OSD_TAB_t) { &compassRescueConfig_altitudeMode, 2, lookupTableCompassRescueAltitudeMode} },
    { "INITAL CLIMB    M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &compassRescueConfig_rescueAltitudeBufferM, 0, 100, 1 } },
    { "ASCEND RATE  CM/S", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &compassRescueConfig_ascendRate, 50, 2500, 1 } },

    { "MIN RETURN ALT  M", OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &compassRescueConfig_minReturnAltitudeM, 0, 1000, 1 } },
    { "PITCH ANGLE RET",   OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &compassRescueConfig_angle, 0, 60, 1 } },

    { "THROTTLE MIN",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &compassRescueConfig_throttleMin, 1000, 2000, 1 } },
    { "THROTTLE MAX",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &compassRescueConfig_throttleMax, 1000, 2000, 1 } },
    { "THROTTLE HOV",      OME_UINT16 | REBOOT_REQUIRED, NULL, &(OSD_UINT16_t){ &compassRescueConfig_throttleHover, 1000, 2000, 1 } },

    { "COMPAS RESCUE PID", OME_Submenu, cmsMenuChange, &cms_menuCompassRescuePid},

    {"BACK", OME_Back, NULL, NULL},
    {NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuCompassRescue = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUCOMRES",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuCompassRescueOnEnter,
    .onExit = cmsx_menuCompassRescueOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuCompassRescueEntries,
};

#endif
