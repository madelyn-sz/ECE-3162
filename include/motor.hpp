#pragma once
#include <cstdint>
#include "hardware/timer.h"
#include "quadrature_encoder_substep.h"
#include "quadrature_encoder_substep.pio.h"

constexpr uint16_t PWM_WRAP = 65534;
constexpr uint16_t ENCODER_RESOLUTION = 585;
constexpr uint16_t SUBSTEPS_PER_STEP = 64;

class Motor {
private:
    
    static uint32_t sm;
    static uint program_offset;
    static void initialize_pio();

    substep_state_t state;

    uint8_t phase_pin;
    uint8_t enable_pin;
    uint8_t encoder_pin;

    int64_t start = 0;

    float wheel_diameter;
public:
    static PIO pio;
    Motor(uint8_t phase_pin, uint8_t enable_pin, uint8_t encoder_pin, float wheel_diameter);
    void initialize();
    void set_pwm(float value);
    void encoder_update();
    void tare();
    void set_calibration(int step0, int step1, int step2);
    float get_position() const;
    float get_velocity() const;
    int get_raw_counts() const;
};