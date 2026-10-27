#pragma once
#include "lsm6dsox_reg.h"
#include "hardware/i2c.h"

constexpr static uint32_t IMU_I2C_BAUD = 400000;    // 400 kHz
constexpr static uint32_t IMU_UPDATE_FREQ = 104;    // 104 Hz

class Imu {
    stmdev_ctx_t ctx;

    float accel_x = 0;
    float accel_y = 0;
    float accel_z = 0;

    float gyro_x = 0;
    float gyro_y = 0;
    float gyro_z = 0;

    float running_z_rotation = 0;
    bool initialized = false;

    volatile bool data_ready = true;

    void clear_i2c_bus();
public:
    struct Config {
        uint32_t i2c_baud;
        uint32_t update_freq;
        i2c_inst_t* i2c_inst;
        uint8_t i2c_address;
        uint8_t i2c_sda_pin;
        uint8_t i2c_scl_pin;
    };

    Config config;

    Imu(const Imu&) = delete;
    Imu& operator=(const Imu&) = delete;

    void initialize();
    void calibrate();

    void imu_update();

    float gyro_z_bias = 0;
    float get_rotation(bool radians = false);
    float get_velocity(bool radians = false);

    Imu(const Config& config);
};