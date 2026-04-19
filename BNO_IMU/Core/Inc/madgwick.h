/**
 * @file madgwick.h
 * @brief Madgwick AHRS (Attitude and Heading Reference System) filter
 *
 * 9-DOF sensor fusion: Accelerometer + Gyroscope + Magnetometer
 * Produces drift-free quaternion orientation, converted to Euler angles.
 *
 * Reference: Sebastian Madgwick, "An efficient orientation filter for
 *            inertial and inertial/magnetic sensor arrays", 2010.
 */
#ifndef MADGWICK_H_
#define MADGWICK_H_

#include <stdint.h>

typedef struct {
    float q0, q1, q2, q3;   // Quaternion state
    float beta;              // Algorithm gain (2 * proportional gain)
    float sampleFreq;        // Sample frequency in Hz
} Madgwick_t;

/**
 * @brief Initialize the Madgwick filter
 * @param filter     Pointer to filter struct
 * @param sampleFreq Sample frequency in Hz (e.g. 100.0f)
 * @param beta       Filter gain. Higher = more acc/mag trust, less drift,
 *                   but more noise. Typical: 0.04 to 0.15
 */
void Madgwick_init(Madgwick_t* filter, float sampleFreq, float beta);

/**
 * @brief Update the filter with new 9-DOF sensor data
 * @param filter  Pointer to filter struct
 * @param gx,gy,gz Gyroscope data in RAD/S
 * @param ax,ay,az Accelerometer data (any unit, will be normalized)
 * @param mx,my,mz Magnetometer data (any unit, will be normalized)
 */
void Madgwick_update(Madgwick_t* filter,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz);

/**
 * @brief Extract Euler angles from current quaternion state
 * @param filter Pointer to filter struct
 * @param roll   Output roll in degrees
 * @param pitch  Output pitch in degrees
 * @param yaw    Output yaw in degrees (0-360, magnetic north referenced)
 */
void Madgwick_getEuler(Madgwick_t* filter, float* roll, float* pitch, float* yaw);

#endif /* MADGWICK_H_ */
