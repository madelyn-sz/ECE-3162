#include <hardware/gpio.h>
#include <pico/platform/common.h>
#include <pico/stdio_usb.h>
#include <pico/time.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "imu.hpp"
#include "motor.hpp"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
constexpr i2c_inst_t* I2C_IMU_INST = i2c0;
constexpr uint8_t I2C_IMU_ADDRESS = 0x6A;
constexpr uint8_t I2C_IMU_SDA = 16;
constexpr uint8_t I2C_IMU_SCL = 17;

constexpr uint8_t GPIO_DRIVER_MODE_PIN = 15;


constexpr uint8_t GPIO_DRIVER_ENABLE_A_PIN = 20;
constexpr uint8_t GPIO_DRIVER_PHASE_A_PIN = 21;

constexpr uint8_t GPIO_DRIVER_ENABLE_B_PIN = 18;
constexpr uint8_t GPIO_DRIVER_PHASE_B_PIN = 19;

constexpr uint8_t GPIO_HALL_A_CH_A_PIN = 12; // CH_B after on GPIO 13
constexpr uint8_t GPIO_HALL_B_CH_A_PIN = 10; // CH_B after on GPIO 11

auto motor_left = Motor(GPIO_DRIVER_PHASE_A_PIN, GPIO_DRIVER_ENABLE_A_PIN, GPIO_HALL_A_CH_A_PIN, 0.06);
auto motor_right = Motor(GPIO_DRIVER_PHASE_B_PIN, GPIO_DRIVER_ENABLE_B_PIN, GPIO_HALL_B_CH_A_PIN, 0.06);

auto imu = Imu({
    .i2c_baud = 400*1000, // 400 kHz
    .update_freq = 104, // 104 Hz
    .i2c_inst = I2C_IMU_INST,
    .i2c_address = I2C_IMU_ADDRESS,
    .i2c_sda_pin = I2C_IMU_SDA,
    .i2c_scl_pin = I2C_IMU_SCL
});

int main()
{
    stdio_init_all();

    // set indicator LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, true);

    // set DRV8835 MODE high (PHASE/ENABLE)
    gpio_init(GPIO_DRIVER_MODE_PIN);
    gpio_set_dir(GPIO_DRIVER_MODE_PIN, GPIO_OUT);
    gpio_put(GPIO_DRIVER_MODE_PIN, true);

    while(!stdio_usb_connected()) {
        sleep_ms(100);
    }
    
    printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
    printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));

    motor_left.initialize();
    motor_right.initialize();
    
    // phase calibration from test runs
    motor_left.set_calibration(70, 125, 198);
    motor_right.set_calibration(51, 96, 205);
    
    imu.initialize();

    motor_left.set_pwm(1);
    motor_right.set_pwm(1);

    for(int i = 0; i < 100; i++) {
        printf("Left position (m): %f, Right position (m): %f, Heading (deg): %f\n", motor_left.get_position(), motor_right.get_position(), imu.get_rotation());
        motor_left.encoder_update();
        motor_right.encoder_update();
        imu.imu_update();
        sleep_ms(100);
    }

    motor_left.set_pwm(0);
    motor_right.set_pwm(0);

    while (true) {
        sleep_ms(100);
    }
}
