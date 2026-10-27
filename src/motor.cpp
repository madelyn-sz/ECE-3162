#include "motor.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "quadrature_encoder_substep.h"
#include <cstdint>
#include <cmath>
#include <algorithm>

// encoder PIO program for quadrature substep counting
static bool PIO_INITIALIZED = false;
PIO Motor::pio;
uint32_t Motor::sm;
uint Motor::program_offset;

void Motor::initialize_pio() {
    Motor::pio = pio0;
    Motor::sm = 0;
    Motor::program_offset = pio_add_program(Motor::pio, &quadrature_encoder_substep_program);
}

void Motor::initialize() {
    if (!PIO_INITIALIZED) {
        Motor::initialize_pio();
        PIO_INITIALIZED = true;
    }

    assert(Motor::sm < 4 && "No more PIO state machines available!");

    substep_init_state(Motor::pio, Motor::sm, encoder_pin, &state);
    state.idle_stop_samples = 20;
    //substep_set_calibration_data(&state, 65, 124, 193); // from calibration run

    Motor::sm += 1;

    gpio_init(phase_pin);
    gpio_set_dir(phase_pin, GPIO_OUT);
    gpio_put(phase_pin, true);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, 65534);
    pwm_init(pwm_gpio_to_slice_num(enable_pin), &cfg, true);

    gpio_set_function(enable_pin, GPIO_FUNC_PWM);

    tare();
}

void Motor::set_pwm(float value) {
    value = std::clamp<float>(value, -1.0, 1.0);

    bool forward = value >= 0;
    uint16_t level = static_cast<uint16_t>(std::abs(value) * 65534);

    gpio_put(phase_pin, !forward);
    pwm_set_gpio_level(enable_pin, level);
}

Motor::Motor(
    uint8_t phase_pin, 
    uint8_t enable_pin, 
    uint8_t encoder_pin,
    float wheel_diameter
) : phase_pin(phase_pin), enable_pin(enable_pin), encoder_pin(encoder_pin), wheel_diameter(wheel_diameter) {

}

void Motor::encoder_update() {
    substep_update(&state);
}

void Motor::tare() {
    start = state.position;
}

float Motor::get_position() const {
    return static_cast<float>(state.position - start) / SUBSTEPS_PER_STEP / ENCODER_RESOLUTION * wheel_diameter * M_PI;
}

float Motor::get_velocity() const {
    return static_cast<float>(state.speed) / SUBSTEPS_PER_STEP / ENCODER_RESOLUTION * wheel_diameter * M_PI;
}

int Motor::get_raw_counts() const {
    return state.position - start;
}

void Motor::set_calibration(int step0, int step1, int step2) {
    substep_set_calibration_data(&state, step0, step1, step2);
}