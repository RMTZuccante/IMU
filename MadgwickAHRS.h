#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <math.h>

#define beta 0.1f

class Madgwick {
  private:
    static float invSqrt(float x);
    float q0;
    float q1;
    float q2;
    float q3;
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    bool anglesComputed;
    void computeAngles();
  public:
    void start(float frequency);
    void update(float gx, float gy, float gz, float ax, float ay, float az);
    float getYaw();
    float getPitch();
    float getRoll();
};

#endif
