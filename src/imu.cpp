#include "imu.hpp"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "lsm6dsox_reg.h"
#include <cstdio>
#include <cstring>

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
    // get our i2c instance from the passed in handle
    Imu::Config* config = static_cast<Imu::Config*>(handle);

    uint8_t buf[256];

    if (len > 255) {
        // we do not have enough space in our preallocated buffer
        return -1;
    }

    // set first byte of buffer to register address
    buf[0] = reg;

    // copy data into remaining buffer
    memcpy(&buf[1], data, len);

    // write over i2c
    int ret = i2c_write_blocking(config->i2c_inst, config->i2c_address, buf, len + 1, false);

    return (ret == PICO_ERROR_GENERIC) ? -1 : 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    Imu::Config* config = static_cast<Imu::Config*>(handle);

    // write i2c register address
    int ret = i2c_write_blocking(config->i2c_inst, config->i2c_address, &reg, 1, true); // keep bus
    if (ret == PICO_ERROR_GENERIC) return -1;

    // read back data
    ret = i2c_read_blocking(config->i2c_inst, config->i2c_address, data, len, false);
    return (ret == PICO_ERROR_GENERIC) ? -1 : 0;
}

void Imu::clear_i2c_bus() {
    gpio_init(config.i2c_sda_pin);
    gpio_init(config.i2c_scl_pin);

    gpio_set_dir(config.i2c_sda_pin, GPIO_IN);
    gpio_set_dir(config.i2c_scl_pin, GPIO_OUT);

    gpio_pull_up(config.i2c_sda_pin);
    gpio_pull_up(config.i2c_scl_pin);

    // Clock up to 9 times until SDA goes high (device releases the bus)
    for (int i = 0; i < 9; i++) {
        if (gpio_get(config.i2c_sda_pin)) {
            // Bus is free
            break; 
        }

        gpio_put(config.i2c_scl_pin, 0); 
        sleep_us(5);
        gpio_put(config.i2c_scl_pin, 1); 
        sleep_us(5);
    }

    // Send a STOP condition to cleanly end any dangling transaction
    gpio_set_dir(config.i2c_sda_pin, GPIO_OUT);
    gpio_put(config.i2c_sda_pin, 0); 
    sleep_us(5);
    gpio_put(config.i2c_scl_pin, 1); 
    sleep_us(5);
    gpio_put(config.i2c_sda_pin, 1); 
    sleep_us(5);
}

Imu::Imu(const Config& config) : config(config) {

}

void Imu::initialize() {
    i2c_deinit(config.i2c_inst);
    printf("Called deinit\n");

    sleep_ms(10);
    clear_i2c_bus();

    printf("Cleared i2c bus\n");
    
    i2c_init(config.i2c_inst, config.i2c_baud);
    
    printf("Initiazlied i2c\n");

    // set GPIO pins to use i2c
    gpio_set_function(config.i2c_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config.i2c_scl_pin, GPIO_FUNC_I2C);
    
    ctx.write_reg = platform_write;
    ctx.read_reg  = platform_read;
    //ctx.mdelay    = platform_delay;
    ctx.handle    = &config;

    printf("Checking WHOAMI\n");

    // verify WHO_AM_I
    uint8_t whoami = 0;
    lsm6dsox_device_id_get(&ctx, &whoami);
    printf("Got device ID\n");

    if (whoami != LSM6DSOX_ID) {
        printf("Got wrong WHOAMI: %u instead of %u\r\n", whoami, LSM6DSOX_ID);
        // TODO: handle WHOAMI error
        while (true) tight_loop_contents();
    }

    printf("resetting defaults\r\n");

    // reset to defaults
    uint8_t rst = 1;
    lsm6dsox_reset_set(&ctx, PROPERTY_ENABLE);
    do { lsm6dsox_reset_get(&ctx, &rst); } while (rst);

    printf("Disabling I3C defaults\r\n");
    // disable I3C (good practice)
    lsm6dsox_i3c_disable_set(&ctx, LSM6DSOX_I3C_DISABLE);

    printf("Enabling block data update\r\n");
    // enable Block Data Update (prevents reading mid-update)
    lsm6dsox_block_data_update_set(&ctx, PROPERTY_ENABLE);

    printf("Configuring accelerometer\r\n");
    // configure accelerometer: 104 Hz, ±4g
    lsm6dsox_xl_data_rate_set(&ctx, LSM6DSOX_XL_ODR_104Hz);
    lsm6dsox_xl_full_scale_set(&ctx, LSM6DSOX_4g);

    printf("Configuring gyroscope\r\n");
    // configure gyroscope: 104 Hz, ±2000 dps
    lsm6dsox_gy_data_rate_set(&ctx, LSM6DSOX_GY_ODR_104Hz);
    lsm6dsox_gy_full_scale_set(&ctx, LSM6DSOX_2000dps);

    // configure FIFO queue
    lsm6dsox_fifo_gy_batch_set(&ctx, LSM6DSOX_GY_BATCHED_AT_104Hz);
    lsm6dsox_fifo_mode_set(&ctx, LSM6DSOX_STREAM_MODE);

    initialized = true;
}

void Imu::calibrate() {
    printf("Initializing IMU...\n");

    if (!initialized) {
        initialize();
    }

    printf("Imu initialized\n");
    
    constexpr uint32_t N_READINGS = 500;
    sleep_ms(100);

    // get N_READINGS of yaw rate from the imu
    float gyro_z_readings[N_READINGS];

    int i = 0;

    while (i < N_READINGS) {
        int16_t raw_gyro[3];
        uint8_t drdy_gy;
        
        lsm6dsox_gy_flag_data_ready_get(&ctx, &drdy_gy);
        
        if (drdy_gy) {
            lsm6dsox_angular_rate_raw_get(&ctx, raw_gyro);
            gyro_z_readings[i] = lsm6dsox_from_fs2000_to_mdps(raw_gyro[2]);
            i += 1;
        }

        sleep_ms(20);
    }

    // average readings to get bias
    float gyro_z_sum = 0;

    for(int i = 0; i < N_READINGS; i++) {
        gyro_z_sum += gyro_z_readings[i];
    }

    gyro_z_bias = gyro_z_sum / N_READINGS;
}

void Imu::imu_update() {
    uint16_t len;

    lsm6dsox_fifo_data_level_get(&ctx, &len);

    for(uint16_t i = 0; i < len; i++) {
        uint8_t raw_data[6];
        int16_t raw_gyro[3];
        float fifo_z;
        lsm6dsox_fifo_tag_t tag;

        int get_tag_ret = lsm6dsox_fifo_sensor_tag_get(&ctx, &tag);

        if (get_tag_ret != 0) {
            printf("Get tag error: %d\n", get_tag_ret);
            continue;
        }
        
        int raw_get_ret = lsm6dsox_fifo_out_raw_get(&ctx, raw_data);

        if (raw_get_ret != 0) {
            printf("Raw get error: %d\n", raw_get_ret);
            continue;
        }

        switch (tag) {
            case LSM6DSOX_GYRO_NC_TAG:
                raw_gyro[0] = static_cast<int16_t>(raw_data[1]) << 8 | raw_data[0];
                raw_gyro[1] = static_cast<int16_t>(raw_data[3]) << 8 | raw_data[2];
                raw_gyro[2] = static_cast<int16_t>(raw_data[5]) << 8 | raw_data[4];

                fifo_z = lsm6dsox_from_fs2000_to_mdps(raw_gyro[2]);

                running_z_rotation += ((fifo_z - gyro_z_bias)/1000.0f) / IMU_UPDATE_FREQ;
                break;
            default:
                break;
        }
    }

    uint8_t drdy_xl, drdy_gy;

    // only update accelerometer if data is ready
    lsm6dsox_xl_flag_data_ready_get(&ctx, &drdy_xl);
    if (drdy_xl) {
        int16_t raw_accel[3];
        lsm6dsox_acceleration_raw_get(&ctx, raw_accel);
        accel_x = lsm6dsox_from_fs4_to_mg(raw_accel[0]);
        accel_y = lsm6dsox_from_fs4_to_mg(raw_accel[1]);
        accel_z = lsm6dsox_from_fs4_to_mg(raw_accel[2]);
    }

    // only update gyroscope if data is ready
    lsm6dsox_gy_flag_data_ready_get(&ctx, &drdy_gy);
    if (drdy_gy) {
        int16_t raw_gyro[3];
        lsm6dsox_angular_rate_raw_get(&ctx, raw_gyro);
        gyro_x = lsm6dsox_from_fs2000_to_mdps(raw_gyro[0]);
        gyro_y = lsm6dsox_from_fs2000_to_mdps(raw_gyro[1]);
        gyro_z = lsm6dsox_from_fs2000_to_mdps(raw_gyro[2]);
    }
}

float Imu::get_velocity(bool radians) {
    if (radians) {
        return (gyro_z - gyro_z_bias) * (M_PI / 180.0);
    } else {
        return gyro_z - gyro_z_bias;
    }
    
}

float Imu::get_rotation(bool radians) {
    if (radians) {
        return running_z_rotation * (M_PI / 180.0);
    } else {
        return running_z_rotation;
    }
}