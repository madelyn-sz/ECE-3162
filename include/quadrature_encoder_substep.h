#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "hardware/pio.h"

typedef struct substep_state_t {
    // configuration data:
    uint calibration_data[4]; // relative phase sizes
    uint clocks_per_us;       // save the clk_sys frequency in clocks per us
    uint idle_stop_samples;   // after these samples without transitions, assume the encoder is stopped
    PIO pio;
    uint sm;

    // internal fields to keep track of the previous state:
    uint prev_trans_pos, prev_trans_us;
    uint prev_step_us;
    uint prev_low, prev_high;
    uint idle_stop_sample_count;
    int speed_2_20;
    int stopped;

    // output of the encoder update function:
    int speed;     // estimated speed in substeps per second
    int position; // estimated position in substeps

    uint raw_step; // raw step count
} substep_state_t;

void set_pwm(uint8_t pwm_pin, uint8_t dir_pin, float value);
void init_pwm(uint8_t pwm_pin, uint8_t dir_pin);
void read_pio_data(substep_state_t *state, uint *step, uint *step_us, uint *transition_us, int *forward);
uint get_step_start_transition_pos(substep_state_t *state, uint step);
int substep_calc_speed(int delta_substep, int delta_us);
void substep_init_state(PIO pio, int sm, int pin_a, substep_state_t *state);
void substep_update(substep_state_t *state);
void substep_calibrate_phases(PIO pio, uint sm);
void substep_set_calibration_data(substep_state_t *state, int step0, int step1, int step2);

#ifdef __cplusplus
}
#endif