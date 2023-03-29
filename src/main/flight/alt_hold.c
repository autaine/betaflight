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

#include "platform.h"
#include "alt_hold.h"

#ifdef USE_ALTHOLD_MODE

#include "drivers/time.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "config/config.h"
#include "rx/rx.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "osd/osd.h"
#include "common/printf.h"
#include "common/maths.h"
#include "math.h"
#include "build/debug.h"

// TODO REMOVE
//static pt2Filter_t ah_throttleDLpf;

PG_REGISTER_WITH_RESET_TEMPLATE(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 4);


PG_RESET_TEMPLATE(altholdConfig_t, altholdConfig,
    .velPidP = 30,
    .velPidD = 0,

    .altPidP = 75,
    .altPidI = 20,

    .minThrottle = 6,
    .maxThrottle = 65,

    .maxVerticalVelocity = 3,// 30,
);


void simplePidInit(simplePid_s* simplePid, float min, float max, float kp, float kd, float ki)
{
    simplePid->max = max;
    simplePid->min = min;
    simplePid->kp = kp;
    simplePid->kd = kd;
    simplePid->ki = ki;
    simplePid->lastErr = 0;
    simplePid->integral = 0;
/*
    const float sampleTimeS = HZ_TO_INTERVAL(ALTHOLD_TASK_PERIOD);
    float cutoffHz, gain;

    cutoffHz = positionConfig()->altitude_d_lpf / 100.0f;
    gain = pt2FilterGain(cutoffHz, sampleTimeS);
    pt2FilterInit(&ah_throttleDLpf, gain); */
}

float simplePidCalculate(simplePid_s* simplePid, float dt, float targetValue, float measuredValue)
{
    float error = targetValue - measuredValue;

    float pOut = simplePid->kp * error;

    float iOut = simplePid->ki * simplePid->integral;

    simplePid->integral += error * dt;

    float derivative = (error - simplePid->lastErr) / dt;
    float dOut = simplePid->kd * derivative;

    float output = pOut + iOut + dOut;
    output = constrainf(output, simplePid->min, simplePid->max);

    simplePid->lastErr = error;
    return output;
}

static float getCurrentAltitude(altHoldState_s* altHoldState)
{
#ifdef USE_BARO
    if (sensors(SENSOR_BARO) && baroIsCalibrated()) {
        return 0.01f * baro.altitude;
    }
#endif
    float rawAltitude = 0.01f * getEstimatedAltitudeCm();
    if (ABS(altHoldState->smoothedAltitude) < 0.01f) {
        altHoldState->smoothedAltitude = rawAltitude;
    }
    float smoothFactor = 0.98f;
    altHoldState->smoothedAltitude = (1.0f - smoothFactor) * rawAltitude + smoothFactor * altHoldState->smoothedAltitude;
    return altHoldState->smoothedAltitude;
}

void altHoldReset(altHoldState_s* altHoldState)
{
    simplePidInit(&altHoldState->altPid, -50.0f, 50.0f,
                  0.01f * altholdConfig()->altPidP,
                  0.0f,
                  0.01f * altholdConfig()->altPidI);

    simplePidInit(&altHoldState->velPid, 0.0f, 1.0f,
                  0.01f * altholdConfig()->velPidP,
                  0.01f * altholdConfig()->velPidD,
                  0.0f);

    altHoldState->throttle = 0.0f;
    altHoldState->enterTime = millis();
    altHoldState->exitTime = 0;
    altHoldState->velocityEstimationVario = 0.01f * getEstimatedVario();
    altHoldState->startVelocityEstimationAccel = altHoldState->velocityEstimationAccel - altHoldState->velocityEstimationVario;
    altHoldState->targetAltitude = getCurrentAltitude(altHoldState);
    altHoldState->smoothedAltitude = 0.01f;

}

void altHoldInit(altHoldState_s* altHoldState)
{
    altHoldState->altHoldEnabled = false;
    altHoldState->throttleFactor = 0.0f;
    altHoldState->velocityEstimationAccel = 0.0f;
    altHoldReset(altHoldState);
}

void altHoldProcessTransitions(altHoldState_s* altHoldState) {
    bool newAltHoldEnabled = FLIGHT_MODE(ALTHOLD_MODE);

    if (FLIGHT_MODE(GPS_RESCUE_MODE) | failsafeIsActive()) {
        newAltHoldEnabled = false;
    }

    if (newAltHoldEnabled && !altHoldState->altHoldEnabled)
    {
        altHoldReset(altHoldState);
    }
    if (!newAltHoldEnabled && altHoldState->altHoldEnabled) {
        altHoldState->exitTime = millis();
    }
    altHoldState->altHoldEnabled = newAltHoldEnabled;

    uint32_t currTime = millis();

    if (newAltHoldEnabled) {
        uint32_t timeSinceEnter = currTime - altHoldState->enterTime;
        if (timeSinceEnter < ALTHOLD_ENTER_PERIOD) {
            float delta = (float)timeSinceEnter / ALTHOLD_ENTER_PERIOD;
            altHoldState->throttleFactor = MAX(delta, altHoldState->throttleFactor);
            return;
        }
        altHoldState->throttleFactor = 1.0f;
        return;
    }

    if (altHoldState->exitTime == 0) {
        altHoldState->throttleFactor = 0.0f;
        return;
    }

    uint32_t timeSinceExit = currTime - altHoldState->exitTime;
    if (timeSinceExit < ALTHOLD_MAX_EXIT_PERIOD) {
        float delta = (float)timeSinceExit / ALTHOLD_MAX_EXIT_PERIOD;
        altHoldState->throttleFactor = MIN(altHoldState->throttleFactor, 1.0f - delta);
        return;
    }

    altHoldState->throttleFactor = 0.0f;
}

void altHoldUpdateTarget(altHoldState_s* altHoldState)
{
    float rcThrottle = rcCommand[THROTTLE];

    rcThrottle -= rxConfig()->midrc;
    rcThrottle = constrainf(rcThrottle, -500.0f, 500.0f);

    rcThrottle = scaleRangef(rcThrottle, -500.0f, 500.0f, 0.0f, 1.0f);

    //if (rcThrottle < 0.25f) {
    if (rcThrottle < 0.3f) {
        rcThrottle = scaleRangef(rcThrottle, 0.0f, 0.25f, -1.0f, 0.0f);
    //} else if (rcThrottle > 0.75f) {
    } else if (rcThrottle > 0.7f) {
        rcThrottle = scaleRangef(rcThrottle, 0.75f, 1.0f, 0.0f, 1.0f);
    } else {
        rcThrottle = 0.0f;
    }

    float altitudeChangeMaxSpeed = 0.1f * altholdConfig()->maxVerticalVelocity;
    altHoldState->targetVelocity = 0.01f * altitudeChangeMaxSpeed * rcThrottle;
    float newTargetAltitude = altHoldState->targetAltitude + altHoldState->targetVelocity;

    newTargetAltitude = MAX(newTargetAltitude, altHoldState->measuredAltitude - 20.0f);
    newTargetAltitude = MIN(newTargetAltitude, altHoldState->measuredAltitude + 20.0f);

    // Do not allow to set target altitule lower -1 meter
    newTargetAltitude = MAX(newTargetAltitude, -1.f);

    altHoldState->targetAltitude = newTargetAltitude;
}

void altHoldUpdate(altHoldState_s* altHoldState)
{
    altHoldProcessTransitions(altHoldState);
    altHoldUpdateTarget(altHoldState);

    float timeInterval = 1.0f / ALTHOLD_TASK_PERIOD;

    float measuredAltitude = getCurrentAltitude(altHoldState);

    t_fp_vector accelerationVector = {{
        acc.accADC[X],
        acc.accADC[Y],
        acc.accADC[Z]
    }};

    imuTransformVectorBodyToEarth(&accelerationVector);

    float measuredAccel = 9.8f * (accelerationVector.V.Z - acc.dev.acc_1G) / acc.dev.acc_1G;

//    DEBUG_SET(DEBUG_ALTHOLD, 0, (int16_t)(measuredAccel * 100.0f));

    altHoldState->velocityEstimationVario = 0.01f * getEstimatedVario();

    altHoldState->velocityEstimationAccel += measuredAccel * timeInterval;
    altHoldState->velocityEstimationAccel *= 0.999f;

    float currentVelocityEstimationAccel = altHoldState->velocityEstimationAccel - altHoldState->startVelocityEstimationAccel;
//    DEBUG_SET(DEBUG_ALTHOLD, 1, (int16_t)(100.0f * currentVelocityEstimationAccel));

    altHoldState->measuredAltitude = measuredAltitude;
    altHoldState->measuredAccel = measuredAccel;

    float measuredAltitudeExtrapolated = altHoldState->measuredAltitude + 1.0f * altHoldState->velocityEstimationVario;

    float velocityTarget = simplePidCalculate(&altHoldState->altPid, timeInterval, altHoldState->targetAltitude, measuredAltitudeExtrapolated);

//    DEBUG_SET(DEBUG_ALTHOLD, 2, (int16_t)(100.0f * velocityTarget));

    float velPidForce = simplePidCalculate(&altHoldState->velPid, timeInterval, velocityTarget, currentVelocityEstimationAccel);

    float newThrottle = velPidForce;

//    DEBUG_SET(DEBUG_ALTHOLD, 3, (int16_t)(100.0f * velPidForce));

    newThrottle = constrainf(newThrottle, 0.0f, 1.0f);
    newThrottle = scaleRangef(newThrottle, 0.0f, 1.0f, 0.01f * altholdConfig()->minThrottle, 0.01f * altholdConfig()->maxThrottle);

    if (altHoldState->altHoldEnabled) {
        altHoldState->throttle = newThrottle;

        DEBUG_SET(DEBUG_ALTHOLD, 0, (int16_t)(altHoldState->targetAltitude * 100.0f));
        DEBUG_SET(DEBUG_ALTHOLD, 1, (int16_t)(altHoldState->measuredAltitude * 100.0f));
    } else {
        DEBUG_SET(DEBUG_ALTHOLD, 0, 0);
        DEBUG_SET(DEBUG_ALTHOLD, 1, 0);
    }

    // LOGIC used in rescue
    /*
    static float velocityI = 0.0f;
    static float previousPitchAdjustment = 0.0f;
    static float throttleI = 0.0f;
    static float previousAltitudeError = 0.0f;
    static int16_t throttleAdjustment = 0;
    
    // currentAltitudeCm is updated at TASK_COMPASS_RESCUE_RATE_HZ
    const float altitudeError = (altHoldState->targetAltitude - measuredAltitudeExtrapolated) * 0.01f;
    // height above target in metres (negative means too low)
    // at the start, the target starts at current altitude plus one step.  Increases stepwise to intended value.

    // P component
    const float throttleP = 15.f * altitudeError;

    // I component
    throttleI += 0.1f * 15.f * altitudeError * timeInterval;
    const float maxIterm = 200.f;
    throttleI = constrainf(throttleI, -1.0f * maxIterm, 1.0f * maxIterm);
    // up to 20% increase in throttle from I alone

    // D component is error based, so includes positive boost when climbing and negative boost on descent
    float verticalSpeed = ((altitudeError - previousAltitudeError) / timeInterval);
    previousAltitudeError = altitudeError;
    //verticalSpeed += c_rescueState.intent.descentRateModifier * verticalSpeed; // no descent but maybe we should boost ascend
    // add up to 2x D when descent rate is faster

    float throttleD = pt2FilterApply(&ah_throttleDLpf, verticalSpeed);

    throttleD = 15.f * throttleD;

    float tiltAdjustment = 1.0f - getCosTiltAngle(); // 0 = flat, gets to 0.2 correcting on a windy day
    tiltAdjustment *= 300.f;
    // if hover is 1300, and adjustment .2, this gives us 0.2*300 or 60 of extra throttle, not much, but useful
    // too much and landings with lots of pitch adjustment, eg windy days, can be a problem

    // boost for some extra trottle action
    tiltAdjustment *= 2.f;

    throttleAdjustment = throttleP + throttleI + throttleD + tiltAdjustment;

    newThrottle = 1300.f + throttleAdjustment;
    newThrottle = constrainf(newThrottle, altholdConfig()->minThrottle, altholdConfig()->minThrottle);
    */
}


altHoldState_s altHoldState;

void initAltHoldState(void) {
    altHoldInit(&altHoldState);
}

void updateAltHoldState(timeUs_t currentTimeUs) {
    altHoldUpdate(&altHoldState);
    (void)currentTimeUs;
}

float getAltHoldThrottle(void) {
    return altHoldState.throttle;
}

float getAltHoldThrottleFactor(float currentThrottle) {
    if (!altHoldState.altHoldEnabled
        && altHoldState.exitTime != 0
        && (ABS(currentThrottle - altHoldState.throttle) < 0.15f))
    {
        altHoldState.exitTime = 0;
    }
    return altHoldState.throttleFactor;
}

#endif
