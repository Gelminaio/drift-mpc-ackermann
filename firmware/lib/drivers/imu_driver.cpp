#include "imu_driver.h"

#include "config.h"

namespace drivers
{

    ImuDriver::ImuDriver(uint8_t i2c_address, uint32_t i2c_frequency_hz)
        : i2c_address_(i2c_address),
          i2c_frequency_hz_(i2c_frequency_hz) {}

    bool ImuDriver::begin()
    {
        // I2C initialization
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, i2c_frequency_hz_);

        if (!imu_.begin(i2c_address_, Wire))
        {
            LOGLN("[imu] ERROR: BNO085 not responding at 0x%02X\n",
                  i2c_address_);
            return false;
        }

        Wire.setClock(i2c_frequency_hz_);

        // Gyro and linear acceleration at 200 Hz. No orientation report: the BNO085 packs
        // reports that come due together, the library parses only the first of a packet,
        // and with three reports the accelerometer was lost 30% of the time (issue #74).
        imu_.enableLinearAccelerometer(IMU_REPORT_INTERVAL_MS); // accel w/o gravity
        imu_.enableGyro(IMU_REPORT_INTERVAL_MS);                // calibrated gyro

        initialized_ = true;
        LOGLN("[imu] BNO085 initialized at 0x%02X, reports @ %u ms (%u Hz)\n",
              i2c_address_,
              IMU_REPORT_INTERVAL_MS,
              1000U / IMU_REPORT_INTERVAL_MS);
        return true;
    }

    bool ImuDriver::read()
    {
        if (!initialized_)
            return false;

        // one call parses one packet: reading one per poll left the accelerometer
        // stale for up to 1 s (issue #74)
        bool fresh = false;
        while (imu_.dataAvailable())
            fresh = true;
        if (!fresh)
            return false;

        latest_.lin_acc_x = imu_.getLinAccelX();
        latest_.lin_acc_y = imu_.getLinAccelY();
        latest_.lin_acc_z = imu_.getLinAccelZ();

        latest_.gyro_x = imu_.getGyroX();
        latest_.gyro_y = imu_.getGyroY();
        latest_.gyro_z = imu_.getGyroZ();

        latest_.timestamp_ms = millis();
        latest_.valid = true;

        return true;
    }

    ImuData ImuDriver::getData() const
    {
        return latest_;
    }

}