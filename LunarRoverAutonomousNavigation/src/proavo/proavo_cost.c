#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <constants/constants.h>
#include <constants/config.h>
#include "structs.h"

const double CALCULATE_POWER_FAIL_RETURN = -1.0;

/*computes power*/
static double calculate_power (double vel, double slope_rads, double steer_angle, double steer_vel) {
	double centripetal_accel = vel * vel; //curvature is multiplied here, so on no turn, it is 0
	/**
	* longitude
	* longitudinal power = (F_rolling + F_grade) * v * longitudinal slip factor
	* F_long = F_rolling + F_grade + F_inertial
	* F_rolling = C_rr * m * g * cos(slope)
	* F_grade = m * g * sin(slope)
	* F_intertial = m * 0 = 0
	* long_slip_factor = K_traction * |F_long| + BASELINE_LOSS
	*/

	double f_rolling = ROLLING_RESISTANCE_COEFF * LUNAR_WEIGHT * cos (slope_rads);
	double f_grade = LUNAR_WEIGHT * sin (slope_rads);
	double f_inertial = 0.0;

	double f_long = f_rolling + f_grade + f_inertial;
	double f_long_no_rolling = f_grade + f_inertial; //for tipover calculation
	double long_slip_factor = K_TRACTION * fabs (f_long) + BASELINE_LOSS;

	double power_long = f_long * vel * long_slip_factor;

	/**
	* cornering
	* cornering power = F_lat * v * lat_slip_factor
	* curvature = tan(steer_angle) / wheelbase
	* F_lat = m * a * curvature
	* latitudinal slip factor = k1 * |curvature| + k2
	* lat slip factor: turn harder -> more energy loss k1 = hard turn loss, k2 = baseline loss
	*/
	//cont. below the tipover calc for fail fast
	double curvature = tan (steer_angle) / ROVER_WHEELBASE;
	double f_lat = VEHICLE_MASS_KG * centripetal_accel * curvature;
	
	//** checking tipover, overturning moment / restoring moment */
	/**
	 * latitudinal tipover (more likely)
	 * tip ratio < H
	 * tip ratio = | lat overturning / lat restoring |
	 * lat overturning = (F_lat + F_gravity) * cog height
	 * lat restoring = 0.5 * track * F_restore
	 * F_gravity = m * g * sin(slope)
	 * F_restore = m * g * cos(slope)
	 * H = 1 - z * variance
	 */
	/**ALWAYS verify critical moment to ensure discrete checks are conservative enough to factor in accumulation of inertia */
	/**accumulated inertia fits into variance in H = 1 - z * variance */
	double f_gravity = f_grade; //identical, different name for clarity
	double f_restore = LUNAR_WEIGHT * cos(slope_rads);

	double lat_overturning = (f_lat + f_gravity) * COG_HEIGHT;
	double lat_restoring = 0.5 * ROVER_TRACK * f_restore;

	double lat_tip_ratio = fabs (lat_overturning / lat_restoring);
	if (lat_tip_ratio >= TIP_MARGIN_HEURISTIC) {
		return CALCULATE_POWER_FAIL_RETURN;
	}

	/**
	 * longitudinal tipover (less likely)
	 * tip ratio < H
	 * tip ratio = | long overturning / long restoring |
	 * long overturning = F_long(without rolling) (already includes gravity) * cog height
	 * long restoring = 0.5 * wheelbase * F_restore
	 * F_gravity = m * g * sin(slope)
	 * F_restore = m * g * cos(slope)
	 * H = 1 - z * variance
	 */
	 double long_overturning = f_long_no_rolling * COG_HEIGHT;
	 double long_restoring = 0.5 * ROVER_WHEELBASE * f_restore;
	 
	 double long_tip_ratio = fabs (long_overturning / long_restoring);
	 if (long_tip_ratio >= TIP_MARGIN_HEURISTIC) {
		 return CALCULATE_POWER_FAIL_RETURN;
	 }
	 


	//cornering continued
	double lat_slip_factor = HARDER_TURN_LOSS * fabs (curvature) + BASELINE_LOSS;
	double power_corner = f_lat * vel * lat_slip_factor;

	/**
	* Ignoring inertial torque
	* steering
	* steering power = |T_static * delta_dot|
	* T_static = K_steering * delta
	*/
	double torque_static = K_STEERING * steer_angle;
	double power_steering = fabs (torque_static * steer_vel);

	//to get joules
	double delta_t = PRIM_LENGTH / vel;

	//total power + thermal loss margin
	return (power_long + power_corner + power_steering) * delta_t * THERMAL_LOSS_MARGIN;
}

/* integrate into hybrid a**/

bool update_energy_cost (struct VehicleState* current, struct VehicleState* successor, double slope_rads, double steer_angle, double steer_vel) {
	//long over prim length
	double vel = NOMINAL_VELOCITY;
	double power_total = calculate_power (vel, slope_rads, steer_angle, steer_vel);

	//return false if tipover risk is high
	if (power_total <= CALCULATE_POWER_FAIL_RETURN) {
		return false;
	}

	if (slope_rads < SAFE_HAVEN_SLOPE_THRESHOLD && steer_angle == 0.0) {
		successor->safe_haven = true;
	}

	//power * time
	double travel_time = (PRIM_LENGTH / vel);
	double step_joules = power_total;

	successor->accumulated_path_cost = current->accumulated_path_cost + step_joules;
	successor->travel_time = travel_time;
	successor->total_time_cost = current->total_time_cost + travel_time;
	

	//map to uint32_t
	successor->accumulated_path_cost = successor->accumulated_path_cost;
	
	return true;
}
