/**
 * @file madgwick.c
 * @brief Madgwick AHRS filter implementation for 9-DOF sensor fusion
 *
 * This is the standard Madgwick algorithm with full magnetometer support.
 * It fuses accelerometer (gravity reference), gyroscope (angular rate),
 * and magnetometer (magnetic north reference) to produce a drift-free
 * quaternion orientation.
 */
#include "madgwick.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / M_PI)

static float invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

void Madgwick_init(Madgwick_t* filter, float sampleFreq, float beta) {
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->beta = beta;
    filter->sampleFreq = sampleFreq;
}

void Madgwick_update(Madgwick_t* f,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;

    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        float mag_norm_sq = mx * mx + my * my + mz * mz;
        float mag_magnitude = sqrtf(mag_norm_sq);
        
        // -------------------------------------------------------------------
        // MAGNETIC REJECTION LOGIC
        // Typical Earth's field is 25-65 uT. We reject if outside 20-80 uT.
        // If rejected, we skip the magnetometer-based correction step.
        // -------------------------------------------------------------------
        int use_mag = (mag_magnitude > 20.0f && mag_magnitude < 80.0f);

        if (use_mag) {
            recipNorm = invSqrt(mag_norm_sq);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1
               + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2
               - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3
                 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient descent algorithm corrective step (9-DOF)
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
               + _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
               - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
               + _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
               - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
               + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
               + _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
               - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az)
               + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
               + _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
               + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx)
               + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
               + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        } else {
            // Gradient descent algorithm corrective step (6-DOF: Accel Only)
            // Need to define variables used in 6-DOF calculation
            float q1q1 = q1 * q1;
            float q2q2 = q2 * q2;
            float _2q0 = 2.0f * q0;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q0q2 = 2.0f * q0 * q2;
            float _2q2q3 = 2.0f * q2 * q3;
            float q0q1 = q0 * q1;

            s0 = -_2q2 * (2.0f * q1 * q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay);
            s1 = _2q3 * (2.0f * q1 * q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
            s2 = -_2q0 * (2.0f * q1 * q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
            s3 = _2q1 * (2.0f * q1 * q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay);
        }

        // Normalise step magnitude
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= f->beta * s0;
        qDot2 -= f->beta * s1;
        qDot3 -= f->beta * s2;
        qDot4 -= f->beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / f->sampleFreq);
    q1 += qDot2 * (1.0f / f->sampleFreq);
    q2 += qDot3 * (1.0f / f->sampleFreq);
    q3 += qDot4 * (1.0f / f->sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    f->q0 = q0 * recipNorm;
    f->q1 = q1 * recipNorm;
    f->q2 = q2 * recipNorm;
    f->q3 = q3 * recipNorm;
}

void Madgwick_getEuler(Madgwick_t* f, float* roll, float* pitch, float* yaw) {
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    // Pitch (y-axis rotation) - clamp to avoid NaN from asinf
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp >= 1.0f)
        *pitch = 90.0f;
    else if (sinp <= -1.0f)
        *pitch = -90.0f;
    else
        *pitch = asinf(sinp) * RAD_TO_DEG;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    // Convert yaw to 0-360 range
    if (*yaw < 0.0f) *yaw += 360.0f;
}
