#include "FusionWrapper.h"

FusionWrapper::FusionWrapper(float sample_rate_hz, float gyroscope_recovery_dps)
    : sample_rate_hz_(sample_rate_hz), gyroscope_recovery_dps_(gyroscope_recovery_dps)
{
    // Initialize bias correction
    FusionBiasInitialise(&bias_, sample_rate_hz_);

    // Initialize AHRS
    FusionAhrsInitialise(&ahrs_);

    // Set default settings
    setSettings();
}

void FusionWrapper::reset()
{
    FusionAhrsReset(&ahrs_);
    FusionBiasInitialise(&bias_, sample_rate_hz_);
}

void FusionWrapper::setSettings(float gain,
                                float accel_rejection_deg,
                                float mag_rejection_deg,
                                float recovery_period_s)
{
    const float clampedRecoverySeconds = recovery_period_s < 0.0f ? 0.0f : recovery_period_s;
    FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,  // North-West-Up coordinate system
        .gain = gain,
        .gyroscopeRange = gyroscope_recovery_dps_,
        .accelerationRejection = accel_rejection_deg,
        .magneticRejection = mag_rejection_deg,
        .recoveryTriggerPeriod = static_cast<unsigned int>(clampedRecoverySeconds * sample_rate_hz_)
    };

    FusionAhrsSetSettings(&ahrs_, &settings);
}

void FusionWrapper::update(float gyro_x, float gyro_y, float gyro_z,
                          float accel_x, float accel_y, float accel_z,
                          float mag_x, float mag_y, float mag_z,
                          float delta_time_s)
{
    // Create vectors
    FusionVector gyroscope = {.axis = {.x = gyro_x, .y = gyro_y, .z = gyro_z}};
    FusionVector accelerometer = {.axis = {.x = accel_x, .y = accel_y, .z = accel_z}};
    FusionVector magnetometer = {.axis = {.x = mag_x, .y = mag_y, .z = mag_z}};

    // Apply bias correction to gyroscope
    gyroscope = FusionBiasUpdate(&bias_, gyroscope);

    // Update AHRS
    FusionAhrsUpdate(&ahrs_, gyroscope, accelerometer, magnetometer, delta_time_s);
}

void FusionWrapper::updateNoMag(float gyro_x, float gyro_y, float gyro_z,
                               float accel_x, float accel_y, float accel_z,
                               float delta_time_s)
{
    // Create vectors
    FusionVector gyroscope = {.axis = {.x = gyro_x, .y = gyro_y, .z = gyro_z}};
    FusionVector accelerometer = {.axis = {.x = accel_x, .y = accel_y, .z = accel_z}};

    // Apply bias correction to gyroscope
    gyroscope = FusionBiasUpdate(&bias_, gyroscope);

    // Update AHRS without magnetometer
    FusionAhrsUpdateNoMagnetometer(&ahrs_, gyroscope, accelerometer, delta_time_s);
}

void FusionWrapper::getEulerAngles(float& roll, float& pitch, float& yaw) const
{
    FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs_);
    FusionEuler euler = FusionQuaternionToEuler(quaternion);

    roll = euler.angle.roll;
    pitch = euler.angle.pitch;
    yaw = euler.angle.yaw;
}

void FusionWrapper::getQuaternion(float& w, float& x, float& y, float& z) const
{
    FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs_);

    w = quaternion.element.w;
    x = quaternion.element.x;
    y = quaternion.element.y;
    z = quaternion.element.z;
}

void FusionWrapper::getEarthAcceleration(float& x, float& y, float& z) const
{
    // Returns linear acceleration in earth (global/NWU) frame with gravity removed
    FusionVector earthAccel = FusionAhrsGetEarthAcceleration(&ahrs_);

    x = earthAccel.axis.x;
    y = earthAccel.axis.y;
    z = earthAccel.axis.z;
}

void FusionWrapper::getLinearAcceleration(float& x, float& y, float& z) const
{
    // Returns linear acceleration in the sensor body frame with gravity removed
    FusionVector linearAccel = FusionAhrsGetLinearAcceleration(&ahrs_);

    x = linearAccel.axis.x;
    y = linearAccel.axis.y;
    z = linearAccel.axis.z;
}
