#include "constants.h"
#include "config.h"


double calculate_wait_energy_by_slot(int duration) {
    double actual_duration_seconds = duration * TIME_SLOT_SIZE;
    return actual_duration_seconds * PASSIVE_BATTERY_DRAIN_RATE;
}

/* calculate the amount of safe waits given a battery and const and distance */
int safe_waits_motor_calc(double current_battery_joules, double joule_cost, int instruction_count) {
    // Calculate path time from instruction count
    double path_time_s = (PRIM_LENGTH * instruction_count) / NOMINAL_VELOCITY;

    // E_batt = E_env/effic_const + (passive drain rate * T)
    // PASSIVE_BATTERY_DRAIN_RATE (static discharge rate in watts)
    // T = path_time_s (total time in seconds)
    // joules / (no unit) + W / s (joules) = joules
    double e_batt = (joule_cost / MOTOR_EFFICIENCY) + (PASSIVE_BATTERY_DRAIN_RATE * path_time_s);

    // subtract total battery cost and safety margin
    double remaining_battery = current_battery_joules - e_batt - BATTERY_SAFETY_MARGIN;

    if (remaining_battery <= 0) {
        return 0;
    }

    // waits left? factor in idle drain rate
    double one_wait = calculate_wait_energy_by_slot(1);
    double max_waits = remaining_battery / one_wait;

    return (int)max_waits;
}