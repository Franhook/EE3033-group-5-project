#ifndef QUATERNION_SOLUTION_H
#define QUATERNION_SOLUTION_H

#include <cmath>

class QuaternionSolution {
public:
    QuaternionSolution();
    
    //Main solution function
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

    // Get the quaternion result
    float getQ0() const { return q0; }
    float getQ1() const { return q1; }
    float getQ2() const { return q2; }
    float getQ3() const { return q3; }

private:
    float invSqrt(float number);

    volatile float twoKp;    // 2 * proportional gain (Kp)
    volatile float twoKi;    // 2 * integral gain (Ki)
    volatile float q0, q1, q2, q3; // quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx, integralFBy, integralFBz; // integral error terms scaled by Ki
};

#endif
