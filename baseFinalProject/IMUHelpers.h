#ifndef IMUHELPERS_H
#define IMUHELPERS_H

#include <Arduino.h>

// ---- Mount transform ----
void computeMountTransform(float ax, float ay, float az,
                           float &ux, float &uy, float &uz);

// ---- Accelerometer → pitch/roll ----
void computeAccelAngles(float ux, float uy, float uz,
                        float &pitch_acc, float &roll_acc);

// ---- Complementary filter ----
void applyComplementaryFilter(float gx_dps, float gy_dps,
                              float pitch_acc, float roll_acc,
                              float dt,
                              float &pitch_est, float &roll_est);

// ---- Yaw integration with bias adaptation ----
void updateYaw(float gz_dps, float dt,
               float &yaw_bias_dps, float &yaw_deg);

// ---- Pitch → frequency mapping ----
int computeTargetFreq(float pitch_est);

// ---- Vibrato bucket computation ----
void computeVibratoBucket(float roll_est,
                          int &vibLevel,
                          float &vibRateHz);

// ---- Gesture axes (accel-based) ----
void computeGestureAxes(float ax, float ay, float az,
                        float &gx, float &gy, float &gz);

#endif
