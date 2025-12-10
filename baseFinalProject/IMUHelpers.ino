#include "IMUHelpers.h"
#include "SoundEngine.h"

void computeMountTransform(float ax, float ay, float az,
                           float &ux, float &uy, float &uz)
{
    ux = ax;
    uy = ay;
    uz = az;
}

void computeAccelAngles(float ux, float uy, float uz,
                        float &pitch_acc, float &roll_acc)
{
    pitch_acc = atan2f(-ux, sqrtf(uy*uy + uz*uz)) * 180.0f / PI;
    roll_acc  = atan2f( uy, uz ) * 180.0f / PI;
}

void applyComplementaryFilter(float gx_dps, float gy_dps,
                              float pitch_acc, float roll_acc,
                              float dt,
                              float &pitch_est, float &roll_est)
{
    pitch_est = CF_ALPHA * (pitch_est + gy_dps * dt) +
                (1.0f - CF_ALPHA) * pitch_acc;

    roll_est  = CF_ALPHA * (roll_est  + gx_dps * dt) +
                (1.0f - CF_ALPHA) * roll_acc;
}

void updateYaw(float gz_dps, float dt,
               float &yaw_bias_dps, float &yaw_deg)
{
    if (fabsf(gz_dps) < CALM_GZ_DPS)
        yaw_bias_dps = (1.0f - YAW_BIAS_ALPHA) * yaw_bias_dps +
                        YAW_BIAS_ALPHA * gz_dps;

    yaw_deg += (gz_dps - yaw_bias_dps) * dt;
}

static int   lastSemi = 9999;
static float semiEdgeLow  = -1e9f;
static float semiEdgeHigh =  1e9f;

//since angles are continuous and we want to play discrete notes there are times where the progrram will flicker between two notes. 
//to avoid this we add buffers to the current note so that when we change tho the adjacent note the edge is further away
static inline int quantizeWithHys(float semi_cont) {
  if (lastSemi == 9999) { 
    lastSemi = (int)roundf(semi_cont);
    semiEdgeLow  = lastSemi - 0.6f;   
    semiEdgeHigh = lastSemi + 0.6f;
  }
  if (semi_cont < semiEdgeLow) {
    lastSemi--;
    semiEdgeLow  = lastSemi - 0.6f;
    semiEdgeHigh = lastSemi + 0.6f;
  } else if (semi_cont > semiEdgeHigh) {
    lastSemi++;
    semiEdgeLow  = lastSemi - 0.6f;
    semiEdgeHigh = lastSemi + 0.6f;
  }
  return lastSemi;
}

int computeTargetFreq(float pitch_est)
{
    if (baseFreq <= 0)
        return 0;

    float norm = (pitch_est - PITCH_MIN_DEG) /
                 (PITCH_MAX_DEG - PITCH_MIN_DEG);
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    float semi_cont = norm * TILT_RANGE_SEMITONES;
    int   semi_disc = quantizeWithHys(semi_cont);
    float f = (float)baseFreq * powf(2.0f, semi_disc / 12.0f);

    if (f < 50.0f)   f = 50.0f;
    if (f > 4000.0f) f = 4000.0f;

    return (int)f;
}

void computeVibratoBucket(float roll_est,
                          int &vibLevel,
                          float &vibRateHz)
{
    float r = roll_est;
    if (r < 0.0f)  r = 0.0f;
    if (r > 90.0f) r = 90.0f;

    if      (r < 15.0f) vibLevel = 0;
    else if (r < 35.0f) vibLevel = 1;
    else if (r < 60.0f) vibLevel = 2;
    else                vibLevel = 3;

    vibRateHz = vibRates[vibLevel];
}

void computeGestureAxes(float ax, float ay, float az,
                        float &gx, float &gy, float &gz)
{
    gx = ax * 100.0f;
    gy = ay * 100.0f;
    gz = az * 100.0f;
}
