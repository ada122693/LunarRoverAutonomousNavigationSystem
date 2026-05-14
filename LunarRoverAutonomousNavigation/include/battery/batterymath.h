#ifndef BATTERYMATH_H
#define BATTERYMATH_H

#ifdef __cplusplus
extern "C" {
#endif
	double calculate_wait_energy_by_slot (int duration);
	int safe_waits_motor_calc (double current_battery_joules, double joule_cost, int instruction_count);
#ifdef __cplusplus
}
#endif
#endif
