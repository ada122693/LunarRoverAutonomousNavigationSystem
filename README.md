# LunarRoverAutonomousNavigationSystem

This code is released under the MIT License. See LICENSE file for details.

A navigation system for autonomous lunar rovers. This system is created by Ada Field while on a team of Rutgers students, the Lunar project is in collaboration with students from Rowan.

This is meant to be run on a central system, reducing cost and risk to individual rovers through proactively pathfinding and testing more in depth than the
individual rovers can. It is intended for rovers capable of performing SLAM or a premapped area, and rovers capable of doing reactive avoidance in conjunction with the
proactive avoidance provided by this.

## general info:

reeds-shepp curves are done with the ompl library

kd trees are done using nanoflann

constants and config must be changed to suit your use case, config.c contains the majority of platform-specific code but constants.c includes certain constants that may change depending on your environment such as lunar_weight may need to be changed to the earth weight for testing with earth gravity
constants in haphazard package and shim should also be updated appropriately
and constants in haphazard package should be tuned when implementing

shim/rover_chim.cpp also includes constants, some are the same as in the constants/ folder so check all 3 before changing any one, as this file is compiled and run separately

## Pathfinding:

Pathfinding currently is done through Hybrid A\* with reeds-shepp. ref. doi:10.1177/0278364909359210
Cost in this algorithm combines energy cost and risk.
Battery power estimates in general are not yet precise, but the energy use is fairly precise.

Inertia and slope tipover risk is calculated per-node, so you can end up in problematic situations where accumulated inertia causes a tip. You solve this by either integrating a larger window to calculate tipover (very costly space and performance-wise) or by increasing safety margins for tipover and decreasing MARGINAL_VELOCITY (lowers optimality of paths potentially).
Currently, it uses very cautious measurements for tipover, but this can be adjusted by changing TIP_MARGIN_HEURISTIC. TIP_MARGIN_HEURISTIC is currently not derived from actual statistical variance. You derive it by doing
Hmax(which is 1) - desired confidence level \* variability.
The current number, 0.87, is extremely cautious, and this should be calculated later based on actual variability in the system.

You should also use tip ratio to calculate your SAFE*HAVEN_SLOPE_THRESHOLD which determines where it is safe to wait for other rovers to pass.
theta =
arcsin(
(tip margin heuristic * wheelbase _ gravity) /
sqrt(
(tip margin heuristic _ wheelbase _ gravity)^2 + (2 _ cog height _ gravity)^2
)
) -
arctan(
(2 _ accel (brake, worst case hard stop) _ cog height)
/ (tip margin heuristic _ wheelbase \_ gravity)
)

## Deconfliction:

deconfliction is done through a greedy algorithm where it lays a discretized space-time schedule along a discretized spacetime grid. The code already provides paths in order of priority, so the highest priority gets to go its planned path first, then next have to utilize waiting at safe locations to avoid conflicts.

The size of time slots and spatial sectors needs to be large enough to account for performance to prevent this from degenerating into a kinematics check and to prevent rovers from stopping or attempting to avoid each other when they are in their respective cells.

Deconfliction is time-based, so either you need a large TIME_SLOT_SIZE or set the agents to prefer or use a velocity and adjust your TIME_SLOT_SIZE accordingly. This can be done on nav2 by setting desired_linear_velocity and speed_limit. A high TIME_SLOT_SIZE is safer and accounts for more drift, a low TIME_SLOT_SIZE increases throughput.

when implementing don't just consider the typical step size and node pool size for pathfinding, also consider the size of these deconfliction structures

## Instructions:

Two types of instruction formats can be constructed. One, which integrates with the testing simply outputs node-by-node position and quaternion information, then later is split into segments using wait hooks. This is well suited to work in conjunction with local reactive avoidance and dynamic obstacles.
The other is path segments which tell velocity duration and heading changes of each segment and are organized by changes, also providing wait hooks. This is well suited for direct instructions but not for integrating with local reactive avoidance and dynamic obstacles. It can also be more subject to physical drift.

Currently I am using the using pose-based node-by-node instructions (nav_generateinstructions_global)

file that constructs the instructions: `src/nav/nav_generateinstructions_global.c`
instructions are position and orientation
wait hooks just are the index of the instruction and duration

currently I just commented out the other method, so this is what you do if you want to switch to the other one:

- uncomment it in `src/nav/nav_generateinstructions.c`
- comment out `src/nav/nav_generateinstructions_global.c`

- uncomment `src/io/construct_full_instructions_generic.c` this is a helper file for inserting waits and is irrelevant in the pose-based version which utilizes a different system as described in the io section of the readme
- comment out the platform specific code in io and switch it to the platform you're using

- uncomment the call in `src/send/send_main.c`
- comment out the other call there too

- also in `src/nav/main.c` follow the same process and switch the include

- uncomment both files in `CMakeLists.txt` (lines 65 and 92) and comment out the global version

- in `include/structs.h` comment out the new Instruction/WaitHookInfo (lines 104-128) and uncomment the old ones (lines 90-101)
- update any function calls from `generate_instructions_global` to `generate_instructions` (they are just in send_main.c, nav_main.c, and io module as mentioned, there's a bunch of platform specific stuff in io so you need to go through it anyway)

- update CMakeLists.txt

Additionally work must be done where appropriate to implement this method itself in the code beyond enabling it

## Objectives and Power Stations:

Receiving objectives and power stations is just done through a txt file you write to before running the program for testing, in order to implement in a larger or more complete system, you need to actually send them in live.

Power Stations are done through one kd tree. This allows for quick searching for the nearest power station when one is needed.

Objectives are done through a set of kd trees for differing priority. Max priority levels for the objectives should be pretty low, 5 is good. The program grabs the nearest objectives at each priority, weighs them based on distance and priority, and selects the best given your weights. This weight can be changed to improve either throughput or lower starvation.
Complete starvation is avoided because each time an objective is properly scheduled, it is removed from the list of objectives. Once a tree to the left (higher priority) runs out of nodes, the next tree to the right is now that priority and so on. This way I implement aging and prefer higher priority paths.

Blacklisting: blacklisting is done when a location cannot go to an objective. Since locations are searched from specific set positions, like a power dock or objective, there's a limited amount of start locations to objectives that can reasonably exist. This allows me to create a blacklist on each objective based on the starting location (with an additional margin added to prevent nearby searches from being done). When an objective is nearest in a kd-tree but the start location is blacklisted, it just gets the next nearest objective on that tree if its available.

## requests

requests can be formatted like this:

```bash
ros2 service call /add_request robot_interfaces/srv/AddRequest "{rover_id: 1, req_flag: 1, x: 10.0, y: 20.0, yaw_rads: 0.5, battery_level: 40000.0}"
```

battery_level is in joules

## shim

this is for requests from the rover to the system
just run it separately with

```bash
./rover_shim [rover_id]
```

## Structure notes

move robot_interface and haphazard to your ws directory next to LunarRoverAutonomousNavigation
but not shim

move your png and world to the home directory if you're using yahboom, if not, change the paths

## testing

1. Install all required dependencies
2. ensure you have all appropriate png or world or have used the pgm method (may require some work in the code due to starting locations, direction of x,y etc)
3. ensure you have appropriate objective and rover configs before compiling
4. First colcon build the robot_interfaces
5. colcon build the LunarRoverAutonomousNavigation and haphazard, you should use symlinks when building haphazard
6. run the programs in this order:

- gazebo if simulation, otherwise connect to your robot
- your bringup for the robot or simulated robot
- haphazard (or if using nav2, your mapping program, nav2, and your bt node stuff)
- LunarRoverAutonomousNavigation
- rover shim

## Dependencies

This project depends on the following ROS 2 packages:

- rclcpp (BSD 3-Clause)
- rclcpp_action (BSD 3-Clause)
- nav_msgs (BSD 3-Clause)
- geometry_msgs (BSD 3-Clause)
- nanoflann (BSD 3-Clause)
- std_msgs (BSD 3-Clause)
- nav_msgs (BSD 3-Clause)
- OpenCV (Apache 2.0)
- grid_map_ros / grid_map_sdf (BSD 3-Clause)
- OMPL (Open Motion Planning Library, BSD 3-Clause)

BSD-licensed packages retain their copyright notices.
