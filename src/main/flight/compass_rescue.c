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

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "platform.h"

#ifdef USE_MAG

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/gps.h"

#include "config/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "compass_rescue.h"

typedef enum {
    RESCUE_IDLE,
    RESCUE_INITIALIZE,
    RESCUE_ATTAIN_ALT,
    RESCUE_ROTATE,
    RESCUE_FLY_HOME,
    //RESCUE_DESCENT,
    //RESCUE_LANDING,
    RESCUE_ABORT,
    RESCUE_COMPLETE,
    RESCUE_DO_NOTHING
} c_rescuePhase_e;

//typedef enum {
//     RESCUE_HEALTHY,
//     RESCUE_FLYAWAY,
//     RESCUE_GPSLOST,
//     RESCUE_LOWSATS,
//     RESCUE_CRASH_FLIP_DETECTED,
//     RESCUE_STALLED,
//     RESCUE_TOO_CLOSE,
//     RESCUE_NO_HOME_POINT
//} rescueFailureState_e;

typedef struct {
    float maxAltitudeCm;
    float returnAltitudeCm;
    float targetAltitudeCm;
    //float targetLandingAltitudeCm;
    //float targetVelocityCmS;
    float pitchAngleLimitDeg;
    float rollAngleLimitDeg;
    //float descentDistanceM;
    int8_t secondsFailing;
    float altitudeStep;
    //float descentRateModifier;
    float yawAttenuator;
    float disarmThreshold;
} c_rescueIntent_s;

typedef struct {
    float currentAltitudeCm;
    // float distanceToHomeCm;
    // float distanceToHomeM;
    // uint16_t groundSpeedCmS;
    int16_t directionToHome;
    float accMagnitude;
    bool healthy;
    float errorAngle;
    //float gpsDataIntervalSeconds;
    float altitudeDataIntervalSeconds;
    //float velocityToHomeCmS;
    float alitutudeStepCm;
    float maxPitchStep;
    float absErrorAngle;
} c_rescueSensorData_s;

typedef struct {
    c_rescuePhase_e phase;
    //rescueFailureState_e failure;
    c_rescueSensorData_s sensor;
    c_rescueIntent_s intent;
    bool isAvailable;
} c_rescueState_s;

#define COMPASS_RESCUE_MAX_YAW_RATE          180    // deg/sec max yaw rate
#define COMPASS_RESCUE_MIN_DESCENT_DIST_M    5      // minimum descent distance
#define COMPASS_RESCUE_MAX_ITERM_VELOCITY    1000   // max iterm value for velocity
#define COMPASS_RESCUE_MAX_ITERM_THROTTLE    200    // max iterm value for throttle
#define COMPASS_RESCUE_MAX_PITCH_RATE        3000   // max change in pitch per second in degrees * 100
#define COMPASS_RESCUE_DISARM_THRESHOLD      2.0f   // disarm threshold in G's

static float c_rescueThrottle;
static float c_rescueYaw;
float       compassRescueAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
//bool        magForceDisable = false;
//static bool newGPSData = false;
static pt2Filter_t c_throttleDLpf;
static pt2Filter_t c_velocityDLpf;
static pt3Filter_t c_pitchLpf;

c_rescueState_s c_rescueState;

void compassRescueInit(void)
{
    // We are using the same Hz as GPS
    const float sampleTimeS = HZ_TO_INTERVAL(TASK_GPS_RESCUE_RATE_HZ);
    float cutoffHz, gain;

    cutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    gain = pt2FilterGain(cutoffHz, sampleTimeS);
    pt2FilterInit(&c_throttleDLpf, gain);

    cutoffHz = 0.8f;
    gain = pt2FilterGain(cutoffHz, 1.0f);
    pt2FilterInit(&c_velocityDLpf, gain);

    cutoffHz = 4.0f;
    gain = pt3FilterGain(cutoffHz, sampleTimeS);
    pt3FilterInit(&c_pitchLpf, gain);
}

/*
 If we have new GPS data, update home heading if possible and applicable.
*/
// void compassRescueNewGpsData(void)
// {
//     newGPSData = true;
// }

static void c_rescueStart(void)
{
    c_rescueState.phase = RESCUE_INITIALIZE;
}

static void c_rescueStop(void)
{
    c_rescueState.phase = RESCUE_IDLE;
}

// Things that need to run when Rescue is enabled, and while armed, but while there is no Rescue in place
static void c_setReturnAltitude(void)
{
    // Hold maxAltitude at zero while disarmed
    if (!ARMING_FLAG(ARMED)) {
        c_rescueState.intent.maxAltitudeCm = 0.0f;
        return;
    }

    // While armed, but not during the rescue, update the max altitude value
    c_rescueState.intent.maxAltitudeCm = fmaxf(c_rescueState.sensor.currentAltitudeCm, c_rescueState.intent.maxAltitudeCm);

    // TODO replace with compass ready?
    //if (newGPSData) {

    // set the target altitude to current values, so there will be no D kick on first run
    c_rescueState.intent.targetAltitudeCm = c_rescueState.sensor.currentAltitudeCm;

    // Keep the descent distance and intended altitude up to date with latest GPS values
    //c_rescueState.intent.descentDistanceM = constrainf(c_rescueState.sensor.distanceToHomeM, COMPASS_RESCUE_MIN_DESCENT_DIST_M, compassRescueConfig()->descentDistanceM);
    //const float initialAltitudeCm = compassRescueConfig()->initialAltitudeM * 100.0f;
    const float rescueAltitudeBufferCm = compassRescueConfig()->rescueAltitudeBufferM * 100.f;
    switch (compassRescueConfig()->altitudeMode) {
        case COMPASS_RESCUE_ALT_MODE_MIN:
            c_rescueState.intent.returnAltitudeCm = fmaxf(c_rescueState.intent.maxAltitudeCm, (compassRescueConfig()->minReturnAltitudeM * 100.f))  + rescueAltitudeBufferCm;
            break;
        case COMPASS_RESCUE_ALT_MODE_CURRENT:
            c_rescueState.intent.returnAltitudeCm = c_rescueState.sensor.currentAltitudeCm + rescueAltitudeBufferCm;
            break;
        case COMPASS_RESCUE_ALT_MODE_MAX:
        default:
            c_rescueState.intent.returnAltitudeCm = c_rescueState.intent.maxAltitudeCm + rescueAltitudeBufferCm;
            break;
    }
    //}
}

static void c_rescueAttainPosition(void)
{
    // runs at 100hz, but only updates RPYT settings when new GPS Data arrives and when not in idle phase.
    //static float previousVelocityError = 0.0f;
    static float velocityI = 0.0f;
    static float previousPitchAdjustment = 0.0f;
    static float throttleI = 0.0f;
    static float previousAltitudeError = 0.0f;
    static int16_t throttleAdjustment = 0;

    switch (c_rescueState.phase) {
    case RESCUE_IDLE:
        // values to be returned when no rescue is active
        compassRescueAngle[AI_PITCH] = 0.0f;
        compassRescueAngle[AI_ROLL] = 0.0f;
        c_rescueThrottle = rcCommand[THROTTLE];
        return;
    case RESCUE_INITIALIZE:
        // Initialize internal variables each time GPS Rescue is started
        //previousVelocityError = 0.0f;
        //velocityI = 0.0f;
        previousPitchAdjustment = 0.0f;
        throttleI = 0.0f;
        previousAltitudeError = 0.0f;
        throttleAdjustment = 0;
        c_rescueState.intent.disarmThreshold = COMPASS_RESCUE_DISARM_THRESHOLD;
        return;
    case RESCUE_DO_NOTHING:
        // 20s of slow descent for switch induced sanity failures to allow time to recover
        compassRescueAngle[AI_PITCH] = 0.0f;
        compassRescueAngle[AI_ROLL] = 0.0f;
        c_rescueThrottle = compassRescueConfig()->throttleHover - 100;
        return;
     default:
        break;
    }

    /**
        Altitude (throttle) controller
    */
    // currentAltitudeCm is updated at TASK_COMPASS_RESCUE_RATE_HZ
    const float altitudeError = (c_rescueState.intent.targetAltitudeCm - c_rescueState.sensor.currentAltitudeCm) * 0.01f;
    // height above target in metres (negative means too low)
    // at the start, the target starts at current altitude plus one step.  Increases stepwise to intended value.

    // P component
    const float throttleP = compassRescueConfig()->throttleP * altitudeError;

    // I component
    throttleI += 0.1f * compassRescueConfig()->throttleI * altitudeError * c_rescueState.sensor.altitudeDataIntervalSeconds;
    throttleI = constrainf(throttleI, -1.0f * COMPASS_RESCUE_MAX_ITERM_THROTTLE, 1.0f * COMPASS_RESCUE_MAX_ITERM_THROTTLE);
    // up to 20% increase in throttle from I alone

    // D component is error based, so includes positive boost when climbing and negative boost on descent
    float verticalSpeed = ((altitudeError - previousAltitudeError) / c_rescueState.sensor.altitudeDataIntervalSeconds);
    previousAltitudeError = altitudeError;
    //verticalSpeed += c_rescueState.intent.descentRateModifier * verticalSpeed; // no descent but maybe we should boost ascend
    // add up to 2x D when descent rate is faster

    float throttleD = pt2FilterApply(&c_throttleDLpf, verticalSpeed);

    //c_rescueState.intent.disarmThreshold = COMPASS_RESCUE_DISARM_THRESHOLD - throttleD / 15.0f; // make disarm more likely if throttle D is high
    // we don't really want to disarm

    throttleD = compassRescueConfig()->throttleD * throttleD;

    // acceleration component not currently implemented - was needed previously due to GPS lag, maybe not needed now.

    float tiltAdjustment = 1.0f - getCosTiltAngle(); // 0 = flat, gets to 0.2 correcting on a windy day
    tiltAdjustment *= (compassRescueConfig()->throttleHover - 1000);
    // if hover is 1300, and adjustment .2, this gives us 0.2*300 or 60 of extra throttle, not much, but useful
    // too much and landings with lots of pitch adjustment, eg windy days, can be a problem

    // boost for some extra trottle action
    tiltAdjustment *= 2.f;

    throttleAdjustment = throttleP + throttleI + throttleD + tiltAdjustment;

    c_rescueThrottle = compassRescueConfig()->throttleHover + throttleAdjustment;
    c_rescueThrottle = constrainf(c_rescueThrottle, compassRescueConfig()->throttleMin, compassRescueConfig()->throttleMax);
    // TODO add proper debug output (or just forget it)
    //DEBUG_SET(DEBUG_COMPASS_RESCUE_THROTTLE_PID, 0, lrintf(throttleP));
    //DEBUG_SET(DEBUG_COMPASS_RESCUE_THROTTLE_PID, 1, lrintf(throttleD));

    /**
        Heading / yaw controller
    */
    // simple yaw P controller with roll mixed in.
    // attitude.values.yaw is set by imuCalculateEstimatedAttitude() and is updated from GPS while groundspeed exceeds 2 m/s
    // below 2m/s groundspeed, the IMU uses gyro to estimate yaw attitude change from previous values
    // above 2m/s, GPS course over ground us ysed to 'correct' the IMU heading
    // if the course over ground, due to wind or pre-exiting movement, is different from the attitude of the quad, the GPS correction will be less accurate
    // the craft should not return much less than 5m/s during the rescue or the GPS corrections may be inaccurate.
    // the faster the return speed, the more accurate the IMU will be, but the consequences of IMU error at the start are greater
    // A compass (magnetometer) is vital for accurate GPS rescue at slow speeds, but must be calibrated and validated.

    c_rescueYaw = c_rescueState.sensor.errorAngle * compassRescueConfig()->yawP * c_rescueState.intent.yawAttenuator * 0.1f;
    c_rescueYaw = constrainf(c_rescueYaw, -COMPASS_RESCUE_MAX_YAW_RATE, COMPASS_RESCUE_MAX_YAW_RATE);
    // c_rescueYaw is the yaw rate in deg/s to correct the heading error

    const float rollMixAttenuator = constrainf(1.0f - fabsf(c_rescueYaw) * 0.01f, 0.0f, 1.0f);
    // less roll at higher yaw rates, no roll at 100 deg/s of yaw
    const float rollAdjustment = -c_rescueYaw * compassRescueConfig()->rollMix * rollMixAttenuator;
    // if rollMix = 100, the roll:yaw ratio is 1:1 at small angles, reducing linearly to zero when the yaw rate is 100 deg/s
    // when compassRescueConfig()->rollMix is zero, there is no roll adjustment
    // rollAdjustment is degrees * 100
    // note that the roll element has the opposite sign to the yaw element *before* GET_DIRECTION

    const float rollLimit = 100.0f * c_rescueState.intent.rollAngleLimitDeg;
    compassRescueAngle[AI_ROLL] = constrainf(rollAdjustment, -rollLimit, rollLimit);
    // compassRescueAngle is added to the normal roll Angle Mode corrections in pid.c

    c_rescueYaw *= GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);

    /**
        Pitch / velocity controller
    */
    static float pitchAdjustment = 0.0f;
    //if (newGPSData) {

       // const float sampleIntervalNormaliseFactor = c_rescueState.sensor.gpsDataIntervalSeconds * 10.0f;

        //const float velocityError = (c_rescueState.intent.targetVelocityCmS - c_rescueState.sensor.velocityToHomeCmS);
        // velocityError is in cm per second, positive means too slow.
        // NB positive pitch setpoint means nose down.

        // P component
        // const float velocityP = velocityError * compassRescueConfig()->velP;

        // // I component

        // Use magic velocity I term to emulate asceleration
        velocityI += 0.01f * 30.f * 1.f * 100.f; // that's gonna be 30.f
        // // increase amount added when GPS sample rate is slower
        // velocityI = constrainf(velocityI, -1.0f * COMPASS_RESCUE_MAX_ITERM_VELOCITY, 1.0f * COMPASS_RESCUE_MAX_ITERM_VELOCITY);
        // // I component alone cannot exceed a pitch angle of 10%

        // // D component
        // float velocityD = ((velocityError - previousVelocityError) / sampleIntervalNormaliseFactor);
        // previousVelocityError = velocityError;
        // const float gain = pt2FilterGain(0.8f, HZ_TO_INTERVAL(gpsGetSampleRateHz()));
        // pt2FilterUpdateCutoff(&c_velocityDLpf, gain);
        // velocityD = pt2FilterApply(&c_velocityDLpf, velocityD);
        // velocityD *= compassRescueConfig()->velD;

        //const float velocityIAttenuator = c_rescueState.intent.targetVelocityCmS / compassRescueConfig()->rescueGroundspeed;
        // reduces iTerm as target velocity decreases, to minimise overshoot during deceleration to landing phase

        // IGNORE all velocity stuff, no data in compass mode
        // Assume we need to move fast with a preset pitch angle

        const float velocityIAttenuator =  2.f; ////c_rescueState.intent.targetVelocityCmS / compassRescueConfig()->rescueGroundspeed;

        pitchAdjustment = 1.f;//velocityP + velocityD;
        if (c_rescueState.phase == RESCUE_FLY_HOME) {
            pitchAdjustment *= 0.7f; // attenuate pitch PIDs during main fly home phase, tighten up in descent.
        }
        pitchAdjustment += velocityI * velocityIAttenuator;

        const float movingAvgPitchAdjustment = 0.5f * (previousPitchAdjustment + pitchAdjustment);
         // moving average seems to work best here, a lot of sequential up and down in velocity data
        previousPitchAdjustment = pitchAdjustment;
        pitchAdjustment = movingAvgPitchAdjustment;
        // pitchAdjustment is the absolute Pitch angle adjustment value in degrees * 100
        // it gets added to the normal level mode Pitch adjustments in pid.c
        //DEBUG_SET(DEBUG_COMPASS_RESCUE_VELOCITY, 0, lrintf(velocityP));
        //DEBUG_SET(DEBUG_COMPASS_RESCUE_VELOCITY, 1, lrintf(velocityD));
    //}

    const float pitchAdjustmentFiltered = pt3FilterApply(&c_pitchLpf, pitchAdjustment);
    // upsampling and smoothing of pitch angle steps

    const float pitchAngleLimit = c_rescueState.intent.pitchAngleLimitDeg * 100.0f;
    compassRescueAngle[AI_PITCH] = constrainf(pitchAdjustmentFiltered, -pitchAngleLimit, pitchAngleLimit);
    // this angle gets added to the normal pitch Angle Mode control values in pid.c - will be seen in pitch setpoint

    //DEBUG_SET(DEBUG_COMPASS_RESCUE_VELOCITY, 3, lrintf(c_rescueState.intent.targetVelocityCmS));
    //DEBUG_SET(DEBUG_COMPASS_RESCUE_TRACKING, 1, lrintf(c_rescueState.intent.targetVelocityCmS));
}

static void c_performSanityChecks(void)
{
    // ! We can only check for crash here really

    // static timeUs_t previousTimeUs = 0; // Last time Stalled/LowSat was checked
    // static float prevAltitudeCm = 0.0f; // to calculate ascent or descent change
    // static float prevTargetAltitudeCm = 0.0f; // to calculate ascent or descent target change
    // //static float previousDistanceToHomeCm = 0.0f; // to check that we are returning
    // //static int8_t secondsLowSats = 0; // Minimum sat detection
    // static int8_t secondsDoingNothing; // Limit on doing nothing
    // const timeUs_t currentTimeUs = micros();

    // if (c_rescueState.phase == RESCUE_IDLE) {
    //     //c_rescueState.failure = RESCUE_HEALTHY;
    //     return;
    // } else if (c_rescueState.phase == RESCUE_INITIALIZE) {
    //     // Initialize these variables each time a GPS Rescue is started
    //     previousTimeUs = currentTimeUs;
    //     prevAltitudeCm = c_rescueState.sensor.currentAltitudeCm;
    //     prevTargetAltitudeCm = c_rescueState.intent.targetAltitudeCm;
    //     //previousDistanceToHomeCm = c_rescueState.sensor.distanceToHomeCm;
    //     //secondsLowSats = 0;
    //     secondsDoingNothing = 0;
    // }

    // Handle events that set a failure mode to other than healthy.
    // Disarm via Abort when sanity on, or for hard Rx loss in FS_ONLY mode
    // Otherwise allow 20s of semi-controlled descent with impact disarm detection
    // const bool hardFailsafe = !rxIsReceivingSignal();

    // if (c_rescueState.failure != RESCUE_HEALTHY) {
    //     // Default to 20s semi-controlled descent with impact detection, then abort
    //     c_rescueState.phase = RESCUE_DO_NOTHING;

    //     switch(compassRescueConfig()->sanityChecks) {
    //     case RESCUE_SANITY_ON:
    //         c_rescueState.phase = RESCUE_ABORT;
    //         break;
    //     case RESCUE_SANITY_FS_ONLY:
    //         if (hardFailsafe) {
    //             c_rescueState.phase = RESCUE_ABORT;
    //         }
    //         break;
    //     default:
    //         // even with sanity checks off,
    //         // override when Allow Arming without Fix is enabled without GPS_FIX_HOME and no Control link available.
    //         if (compassRescueConfig()->allowArmingWithoutFix && !STATE(GPS_FIX_HOME) && hardFailsafe) {
    //             c_rescueState.phase = RESCUE_ABORT;
    //         }
    //     }
    // }

    // Crash detection is enabled in all rescues.  If triggered, immediately disarm.
    if (crashRecoveryModeActive()) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_CRASH_PROTECTION);
        c_rescueStop();
    }

    // Check if GPS comms are healthy
    // ToDo - check if we have an altitude reading; if we have Baro, we can use Landing mode for controlled descent without GPS
    // if (!c_rescueState.sensor.healthy) {
    //     c_rescueState.failure = RESCUE_GPSLOST;
    // }

    //  Things that should run at a low refresh rate (such as flyaway detection, etc) will be checked at 1Hz
    // const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    // if (dTime < 1000000) { //1hz
    //     return;
    // }
    // previousTimeUs = currentTimeUs;

    // checks that we are getting closer to home.
    // if the quad is stuck, or if GPS data packets stop, there will be no change in distance to home
    // we can't use c_rescueState.sensor.currentVelocity because it will be held at the last good value if GPS data updates stop
//     if (c_rescueState.phase == RESCUE_FLY_HOME) {
//         const float velocityToHomeCmS = previousDistanceToHomeCm- c_rescueState.sensor.distanceToHomeCm; // cm/s
//         previousDistanceToHomeCm = c_rescueState.sensor.distanceToHomeCm;
//         c_rescueState.intent.secondsFailing += (velocityToHomeCmS < 0.5f * c_rescueState.intent.targetVelocityCmS) ? 1 : -1;
//         c_rescueState.intent.secondsFailing = constrain(c_rescueState.intent.secondsFailing, 0, 15);
//         if (c_rescueState.intent.secondsFailing == 15) {
// #ifdef USE_MAG
//             //If there is a mag and has not been disabled, we have to assume is healthy and has been used in imu.c
//             if (sensors(SENSOR_MAG) && compassRescueConfig()->useMag && !magForceDisable) {
//                 //Try again with mag disabled
//                 magForceDisable = true;
//                 c_rescueState.intent.secondsFailing = 0;
//             } else
// #endif
//             {
//                 c_rescueState.failure = RESCUE_FLYAWAY;
//             }
//         }
//     }

    //secondsLowSats += (!STATE(GPS_FIX) || (gpsSol.numSat < GPS_MIN_SAT_COUNT)) ? 1 : -1;
    //secondsLowSats = constrain(secondsLowSats, 0, 10);

    // if (secondsLowSats == 10) {
    //     c_rescueState.failure = RESCUE_LOWSATS;
    // }


    // These conditions ignore sanity mode settings, and apply in all rescues, to handle getting stuck in a climb or descend

    // const float actualAltitudeChange = c_rescueState.sensor.currentAltitudeCm - prevAltitudeCm;
    // const float targetAltitudeChange = c_rescueState.intent.targetAltitudeCm - prevTargetAltitudeCm;
    // const float ratio = actualAltitudeChange / targetAltitudeChange;
    // prevAltitudeCm = c_rescueState.sensor.currentAltitudeCm;
    // prevTargetAltitudeCm = c_rescueState.intent.targetAltitudeCm;

    //switch (c_rescueState.phase) {
    // case RESCUE_LANDING:
    //     c_rescueState.intent.secondsFailing += ratio > 0.5f ? -1 : 1;
    //     c_rescueState.intent.secondsFailing = constrain(c_rescueState.intent.secondsFailing, 0, 10);
    //     if (c_rescueState.intent.secondsFailing == 10) {
    //         c_rescueState.phase = RESCUE_ABORT;
    //         // Landing mode shouldn't take more than 10s
    //     }
    //     break;
    // case RESCUE_ATTAIN_ALT:
    // case RESCUE_DESCENT:
    //     c_rescueState.intent.secondsFailing += ratio > 0.5f ? -1 : 1;
    //     c_rescueState.intent.secondsFailing = constrain(c_rescueState.intent.secondsFailing, 0, 10);
    //     if (c_rescueState.intent.secondsFailing == 10) {
    //         c_rescueState.phase = RESCUE_LANDING;
    //         c_rescueState.intent.secondsFailing = 0;
    //         // if can't climb, or slow descending, enable impact detection and time out in 10s
    //     }
    //     break;
    // case RESCUE_DO_NOTHING:
    //     secondsDoingNothing = MIN(secondsDoingNothing + 1, 20);
    //     if (secondsDoingNothing == 20) {
    //         c_rescueState.phase = RESCUE_ABORT;
    //         // time-limited semi-controlled fall with impact detection
    //     }
    //     break;
    // default:
    //     // do nothing
    //     break;
    // }

    //DEBUG_SET(DEBUG_RTH, 2, (c_rescueState.failure * 10 + c_rescueState.phase));
    //DEBUG_SET(DEBUG_RTH, 3, (c_rescueState.intent.secondsFailing * 100 + secondsLowSats));
}

static void c_sensorUpdate(void)
{
    //static float prevDistanceToHomeCm = 0.0f;
    const timeUs_t currentTimeUs = micros();

    static timeUs_t previousAltitudeDataTimeUs = 0;
    const timeDelta_t altitudeDataIntervalUs = cmpTimeUs(currentTimeUs, previousAltitudeDataTimeUs);
    c_rescueState.sensor.altitudeDataIntervalSeconds = altitudeDataIntervalUs * 0.000001f;
    previousAltitudeDataTimeUs = currentTimeUs;

    c_rescueState.sensor.currentAltitudeCm = getAltitude();

    // DEBUG_SET(DEBUG_COMPASS_RESCUE_TRACKING, 2, lrintf(c_rescueState.sensor.currentAltitudeCm));
    // DEBUG_SET(DEBUG_COMPASS_RESCUE_THROTTLE_PID, 2, lrintf(c_rescueState.sensor.currentAltitudeCm));
    // DEBUG_SET(DEBUG_COMPASS_RESCUE_HEADING, 0, c_rescueState.sensor.groundSpeedCmS); // groundspeed cm/s
    // DEBUG_SET(DEBUG_COMPASS_RESCUE_HEADING, 1, gpsSol.groundCourse); // degrees * 10
    // DEBUG_SET(DEBUG_COMPASS_RESCUE_HEADING, 2, attitude.values.yaw); // degrees * 10
    // DEBUG_SET(DEBUG_COMPASS_RESCUE_HEADING, 3, c_rescueState.sensor.directionToHome); // degrees * 10

    //c_rescueState.sensor.healthy = gpsIsHealthy();

    // if (c_rescueState.phase == RESCUE_LANDING) {
    //     // do this at sensor update rate, not the much slower GPS rate, for quick disarm
    //     c_rescueState.sensor.accMagnitude = (float) sqrtf(sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y])) * acc.dev.acc_1G_rec;
    // }

    //c_rescueState.sensor.directionToHome = GPS_directionToHome;

    // Use pre-set direction
    c_rescueState.sensor.directionToHome = compassRescueConfig()->direction;
    c_rescueState.sensor.errorAngle = (attitude.values.yaw - c_rescueState.sensor.directionToHome) * 0.1f;
    // both attitude and direction are in degrees * 10, errorAngle is degrees
    if (c_rescueState.sensor.errorAngle <= -180) {
        c_rescueState.sensor.errorAngle += 360;
    } else if (c_rescueState.sensor.errorAngle > 180) {
        c_rescueState.sensor.errorAngle -= 360;
    }
    c_rescueState.sensor.absErrorAngle = fabsf(c_rescueState.sensor.errorAngle);

    // if (!newGPSData) {
    //     return;
    //     // GPS ground speed, velocity and distance to home will be held at last good values if no new packets
    // }

    // c_rescueState.sensor.distanceToHomeCm = GPS_distanceToHomeCm;
    // c_rescueState.sensor.distanceToHomeM = c_rescueState.sensor.distanceToHomeCm / 100.0f;
    // c_rescueState.sensor.groundSpeedCmS = gpsSol.groundSpeed; // cm/s

    // static timeUs_t previousGPSDataTimeUs = 0;
    // const timeDelta_t gpsDataIntervalUs = cmpTimeUs(currentTimeUs, previousGPSDataTimeUs);
    // c_rescueState.sensor.gpsDataIntervalSeconds = constrainf(gpsDataIntervalUs * 0.000001f, 0.01f, 1.0f);
    // // Range from 10ms (100hz) to 1000ms (1Hz). Intended to cover common GPS data rates and exclude unusual values.
    // previousGPSDataTimeUs = currentTimeUs;

    // c_rescueState.sensor.velocityToHomeCmS = (prevDistanceToHomeCm - c_rescueState.sensor.distanceToHomeCm) / c_rescueState.sensor.gpsDataIntervalSeconds;
    // // positive = towards home.  First value is useless since prevDistanceToHomeCm was zero.
    // prevDistanceToHomeCm = c_rescueState.sensor.distanceToHomeCm;

    // c_rescueState.sensor.maxPitchStep = c_rescueState.sensor.gpsDataIntervalSeconds * COMPASS_RESCUE_MAX_PITCH_RATE;

    // DEBUG_SET(DEBUG_COMPASS_RESCUE_VELOCITY, 2, lrintf(c_rescueState.sensor.velocityToHomeCmS));
    // DEBUG_SET(DEBUG_COMPASS_RESCUE_TRACKING, 0, lrintf(c_rescueState.sensor.velocityToHomeCmS));

}

// This function flashes "RESCUE N/A" in the OSD if:
// 1. sensor healthy - GPS data is being received.
// 2. GPS has a 3D fix.
// 3. GPS number of satellites is greater than or equal to the minimum configured satellite count.
// Note 1: cannot arm without the required number of sats
// hence this flashing indicates that after having enough sats, we now have below the minimum and the rescue likely would fail
// Note 2: this function does not take into account the distance from home
// The sanity checks are independent, this just provides the OSD warning
static bool checkcompassRescueIsAvailable(void)
{
    // YES

    // static timeUs_t previousTimeUs = 0; // Last time LowSat was checked
    // const timeUs_t currentTimeUs = micros();
    // static int8_t secondsLowSats = 0; // Minimum sat detection
    // static bool lowsats = false;
    // static bool noGPSfix = false;
    // bool result = true;

    // if (!gpsIsHealthy() || !STATE(GPS_FIX_HOME)) {
    //     return false;
    // }

    // //  Things that should run at a low refresh rate >> ~1hz
    // const timeDelta_t dTime = cmpTimeUs(currentTimeUs, previousTimeUs);
    // if (dTime < 1000000) { //1hz
    //     if (noGPSfix || lowsats) {
    //         result = false;
    //     }
    //     return result;
    // }

    // previousTimeUs = currentTimeUs;

    // if (!STATE(GPS_FIX)) {
    //     result = false;
    //     noGPSfix = true;
    // } else {
    //     noGPSfix = false;
    // }

    // secondsLowSats = constrain(secondsLowSats + ((gpsSol.numSat < GPS_MIN_SAT_COUNT) ? 1 : -1), 0, 2);
    // if (secondsLowSats == 2) {
    //     lowsats = true;
    //     result = false;
    // } else {
    //     lowsats = false;
    // }

    return true;
}

void c_disarmOnImpact(void)
{
    if (c_rescueState.sensor.accMagnitude > c_rescueState.intent.disarmThreshold) {
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_CRASH_PROTECTION);
        c_rescueStop();
    }
}

// void descend(void)
// {
//     if (newGPSData) {
//         const float distanceToLandingAreaM = c_rescueState.sensor.distanceToHomeM - (c_rescueState.intent.targetLandingAltitudeCm / 200.0f);
//         // considers home to be a circle half landing height around home to avoid overshooting home point
//         const float proximityToLandingArea = constrainf(distanceToLandingAreaM / c_rescueState.intent.descentDistanceM, 0.0f, 1.0f);
//         c_rescueState.intent.targetVelocityCmS = compassRescueConfig()->rescueGroundspeed * proximityToLandingArea;
//         // reduce target velocity as we get closer to home. Zero within 2m of home, reducing risk of overshooting.
//         // if quad drifts further than 2m away from home, should by then have rotated towards home, so pitch is allowed
//         c_rescueState.intent.rollAngleLimitDeg = compassRescueConfig()->angle * proximityToLandingArea;
//         // reduce roll capability when closer to home, none within final 2m
//     }

//     // adjust altitude step for interval between altitude readings
//     c_rescueState.intent.altitudeStep = -1.0f * c_rescueState.sensor.altitudeDataIntervalSeconds * compassRescueConfig()->descendRate;

//     // descend more slowly if return altitude is less than 20m
//     const float descentAttenuator = c_rescueState.intent.returnAltitudeCm / 2000.0f;
//     if (descentAttenuator < 1.0f) {
//         c_rescueState.intent.altitudeStep *= descentAttenuator;
//     }
//     // descend more quickly from higher altitude
//     c_rescueState.intent.descentRateModifier = constrainf(c_rescueState.intent.targetAltitudeCm / 5000.0f, 0.0f, 1.0f);
//     c_rescueState.intent.targetAltitudeCm += c_rescueState.intent.altitudeStep * (1.0f + (2.0f * c_rescueState.intent.descentRateModifier));
//     // increase descent rate to max of 3x default above 50m, 2x above 25m, 1.2 at 5m, default by ground level.
// }

void c_altitudeAchieved(void)
{
    c_rescueState.intent.targetAltitudeCm = c_rescueState.intent.returnAltitudeCm;
    c_rescueState.intent.altitudeStep = 0;
    c_rescueState.phase = RESCUE_ROTATE;
}

void compassRescueUpdate(void)
// this runs a lot faster than the GPS Data update rate, and runs whether or not rescue is active
{
    if (!FLIGHT_MODE(COMPASS_RESCUE_MODE)) {
        c_rescueStop(); // sets phase to RESCUE_IDLE; does nothing else.  RESCUE_IDLE tasks still run.
    } else if (FLIGHT_MODE(COMPASS_RESCUE_MODE) && c_rescueState.phase == RESCUE_IDLE) {
        c_rescueStart(); // sets phase to rescue_initialise if we enter GPS Rescue mode while idle
        c_rescueAttainPosition(); // Initialise basic parameters when a Rescue starts (can't initialise sensor data reliably)
        c_performSanityChecks(); // Initialises sanity check values when a Rescue starts
    }

    // Will now be in RESCUE_INITIALIZE mode, if just entered Rescue while IDLE, otherwise stays IDLE

    c_sensorUpdate(); // always get latest GPS and Altitude data, update ascend and descend rates

    bool startedLow = true;
    c_rescueState.isAvailable = checkcompassRescueIsAvailable();

    switch (c_rescueState.phase) {
    case RESCUE_IDLE:
        // in Idle phase = NOT in GPS Rescue
        // update the return altitude and descent distance values, to have valid settings immediately they are needed
        c_setReturnAltitude();
        break;
        // sanity checks are bypassed in IDLE mode; instead, failure state is always initialised to HEALTHY
        // target altitude is always set to current altitude.

    case RESCUE_INITIALIZE:
        // Things that should abort the start of a Rescue
        // if (!STATE(GPS_FIX_HOME)) {
        //     // we didn't get a home point on arming
        //     c_rescueState.failure = RESCUE_NO_HOME_POINT;
            // will result in a disarm via the sanity check system, with delay if switch induced
            // alternative is to prevent the rescue by returning to IDLE, but this could cause flyaways
        // } else if (c_rescueState.sensor.distanceToHomeM < compassRescueConfig()->minRescueDth) {
        //     // Attempt to initiate inside minimum activation distance -> landing mode
        //     c_rescueState.intent.altitudeStep = -c_rescueState.sensor.altitudeDataIntervalSeconds * compassRescueConfig()->descendRate;
        //     c_rescueState.intent.targetVelocityCmS = 0; // zero forward velocity
        //     c_rescueState.intent.pitchAngleLimitDeg = 0; // flat on pitch
        //     c_rescueState.intent.rollAngleLimitDeg = 0.0f; // flat on roll also
        //     c_rescueState.intent.targetAltitudeCm = c_rescueState.sensor.currentAltitudeCm + c_rescueState.intent.altitudeStep;
        //     c_rescueState.phase = RESCUE_LANDING;
        //     // start landing from current altitude
       // } else {
        {
            c_rescueState.phase = RESCUE_ATTAIN_ALT;
            c_rescueState.intent.secondsFailing = 0; // reset the sanity check timer for the climb
            //c_rescueState.intent.targetLandingAltitudeCm = 100.0f * compassRescueConfig()->targetLandingAltitudeM;
            startedLow = (c_rescueState.sensor.currentAltitudeCm <= c_rescueState.intent.returnAltitudeCm);
            c_rescueState.intent.yawAttenuator = 0.0f;
            //c_rescueState.intent.targetVelocityCmS = 0.0f; // zero forward velocity while climbing
            c_rescueState.intent.pitchAngleLimitDeg = 0.0f; // no pitch
            c_rescueState.intent.rollAngleLimitDeg = 0.0f; // no roll until flying home
            c_rescueState.intent.altitudeStep = 0.0f;
            //c_rescueState.intent.descentRateModifier = 0.0f;
        }
        break;

    case RESCUE_ATTAIN_ALT:
        // gradually increment the target altitude until the craft reaches target altitude
        // note that this can mean the target altitude may increase above returnAltitude if the craft lags target
        // sanity check will abort if altitude gain is blocked for a cumulative period
        // If using current alt mode then ignore alt completely
        if (startedLow && compassRescueConfig()->altitudeMode != COMPASS_RESCUE_ALT_MODE_CURRENT) {
            if (c_rescueState.intent.targetAltitudeCm < c_rescueState.intent.returnAltitudeCm) {
                c_rescueState.intent.altitudeStep = c_rescueState.sensor.altitudeDataIntervalSeconds * compassRescueConfig()->ascendRate;
            } else if (c_rescueState.sensor.currentAltitudeCm > c_rescueState.intent.returnAltitudeCm) {
                c_altitudeAchieved();
            } 
        } else { // 
            //if (c_rescueState.intent.targetAltitudeCm > c_rescueState.intent.returnAltitudeCm) {
            //    c_rescueState.intent.altitudeStep = -c_rescueState.sensor.altitudeDataIntervalSeconds * compassRescueConfig()->descendRate;
            // } else if (c_rescueState.sensor.currentAltitudeCm < c_rescueState.intent.returnAltitudeCm) {
                c_altitudeAchieved();
            //}
        }
        c_rescueState.intent.targetAltitudeCm += c_rescueState.intent.altitudeStep;
        break;

    case RESCUE_ROTATE:
        if (c_rescueState.intent.yawAttenuator < 1.0f) { // gradually acquire yaw authority
            c_rescueState.intent.yawAttenuator += 0.01f;
        }
        if (c_rescueState.sensor.absErrorAngle < 30.0f) {
            c_rescueState.intent.pitchAngleLimitDeg = compassRescueConfig()->angle; // allow pitch
            c_rescueState.phase = RESCUE_FLY_HOME; // enter fly home phase
            c_rescueState.intent.secondsFailing = 0; // reset sanity timer for flight home
        }
        break;

    case RESCUE_FLY_HOME:
        if (c_rescueState.intent.yawAttenuator < 1.0f) { // be sure to accumulate full yaw authority
            c_rescueState.intent.yawAttenuator += 0.01f;
        }
        // steadily increase target velocity target until full return velocity is acquired
        // if (c_rescueState.intent.targetVelocityCmS < compassRescueConfig()->rescueGroundspeed) {
        //     c_rescueState.intent.targetVelocityCmS += 0.01f * compassRescueConfig()->rescueGroundspeed;
        // }
        // acquire full roll authority slowly when pointing to home
        if (c_rescueState.sensor.absErrorAngle < 10.0f && c_rescueState.intent.rollAngleLimitDeg < compassRescueConfig()->angle) {
            // roll is primarily intended to deal with wind drift causing small yaw errors during return
            c_rescueState.intent.rollAngleLimitDeg += 0.1f;
        } 

        // if (newGPSData) {
        //     if (c_rescueState.sensor.distanceToHomeM <= c_rescueState.intent.descentDistanceM) {
        //         c_rescueState.phase = RESCUE_DESCENT;
        //         c_rescueState.intent.secondsFailing = 0; // reset sanity timer for descent
        //     }
        // }
        break;

    // case RESCUE_DESCENT:
    //     // attenuate velocity and altitude targets while updating the heading to home
    //     if (c_rescueState.sensor.currentAltitudeCm < c_rescueState.intent.targetLandingAltitudeCm) {
    //         // enter landing mode once below landing altitude
    //         c_rescueState.phase = RESCUE_LANDING;
    //         c_rescueState.intent.secondsFailing = 0; // reset sanity timer for landing
    //     }
    //     descend();
    //     break;

    // case RESCUE_LANDING:
    //     // Reduce altitude target steadily until impact, then disarm.
    //     // control yaw angle and throttle and pitch, attenuate velocity, roll and pitch iTerm
    //     descend();
    //     c_disarmOnImpact();
    //     break;

    case RESCUE_COMPLETE:
        c_rescueStop();
        break;

    case RESCUE_ABORT:
        setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
        disarm(DISARM_REASON_FAILSAFE);
        c_rescueStop();
        break;

    case RESCUE_DO_NOTHING:
        c_disarmOnImpact();
        break;

    default:
        break;
    }

    //DEBUG_SET(DEBUG_COMPASS_RESCUE_TRACKING, 3, lrintf(c_rescueState.intent.targetAltitudeCm));
    //DEBUG_SET(DEBUG_COMPASS_RESCUE_THROTTLE_PID, 3, lrintf(c_rescueState.intent.targetAltitudeCm));
    DEBUG_SET(DEBUG_RTH, 0, lrintf(c_rescueState.intent.maxAltitudeCm));

    c_performSanityChecks();
    c_rescueAttainPosition();

    //newGPSData = false;
}

float compassRescueGetYawRate(void)
{
    return c_rescueYaw;
}

float compassRescueGetThrottle(void)
{
    // Calculated a desired commanded throttle scaled from 0.0 to 1.0 for use in the mixer.
    // We need to compensate for min_check since the throttle value set by gps rescue
    // is based on the raw rcCommand value commanded by the pilot.
    float commandedThrottle = scaleRangef(c_rescueThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);

    return commandedThrottle;
}

bool compassRescueIsConfigured(void)
{
    // TODO add box mode
    return failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_COMPASS_RESCUE;// || isModeActivationConditionPresent(BOXcompassRescue);
}

bool compassRescueIsAvailable(void)
{
    return c_rescueState.isAvailable;
}

// bool compassRescueIsDisabled(void)
// // used for OSD warning
// {
//     return (!STATE(GPS_FIX_HOME));
// }

// #ifdef USE_MAG
// bool compassRescueDisableMag(void)
// {
//     return ((!compassRescueConfig()->useMag || magForceDisable) && (c_rescueState.phase >= RESCUE_INITIALIZE) && (c_rescueState.phase <= RESCUE_LANDING));
// }
// #endif
#endif
