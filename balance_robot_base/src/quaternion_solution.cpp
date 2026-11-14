#include "balance_robot_base/quaternion_solution.h"

// Sampling frequency (corresponding to the macro in the original code))
#define SAMPLING_FREQ 20.0f 

QuaternionSolution::QuaternionSolution() 
    : twoKp(1.0f), twoKi(0.0f), 
      q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f),
      integralFBx(0.0f), integralFBy(0.0f), integralFBz(0.0f)
{
}

float QuaternionSolution::invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;
    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

void QuaternionSolution::update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
        // 1. Normalize the accelerometer
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;      

        // 2. Estimate the gravity direction (derived from the quaternion))
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 3. Calculate the error (cross product of measured gravity and estimated gravity))
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 4. Integral feedback (Integral Feedback)
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;  
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;
            gx += integralFBx; gy += integralFBy; gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
        }

        // 5. Proportional feedback (Proportional Feedback)
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // 6. Integrate the quaternion differential equation
    gx *= (0.5f * dt);   
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    
    qa = q0;
    qb = q1;
    qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 

    // 7. Normalize the quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
