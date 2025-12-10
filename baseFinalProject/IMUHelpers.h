#ifndef IMUHELPERS_H
#define IMUHELPERS_H

#include <Arduino.h>

const float PITCH_MIN_DEG = -60.0f;
const float PITCH_MAX_DEG =  60.0f;
const float TILT_RANGE_SEMITONES = 24.0f;
const  float CF_ALPHA  = 0.98f;
const  float YAW_BIAS_ALPHA = 0.002f;
const  float CALM_GZ_DPS    = 10.0f;

void computeAccelAngles(float ux, float uy, float uz,
                        float &pitch_acc, float &roll_acc);

void applyComplementaryFilter(float gx_dps, float gy_dps,
                              float pitch_acc, float roll_acc,
                              float dt,
                              float &pitch_est, float &roll_est);

void updateYaw(float gz_dps, float dt,
               float &yaw_bias_dps, float &yaw_deg);

int computeTargetFreq(float pitch_est);

void computeVibratoBucket(float roll_est,
                          int &vibLevel,
                          float &vibRateHz);

void computeGestureAxes(float ax, float ay, float az,
                        float &gx, float &gy, float &gz);

#endif
