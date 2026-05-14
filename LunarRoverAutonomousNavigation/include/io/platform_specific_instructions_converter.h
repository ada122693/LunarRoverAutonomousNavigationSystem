#ifndef PLATFORM_SPECIFIC_INSTRUCTIONS_CONVERTER_H
#define PLATFORM_SPECIFIC_INSTRUCTIONS_CONVERTER_H


#include "structs.h"

#ifdef __cplusplus
#include <vector>
#include "robot_interfaces/action/path_schedule_plan.hpp"

    //goal contains an array of paths and an array of waits
    struct PlatformSpecificInstructions {
        int rover_id;
        robot_interfaces::action::PathSchedulePlan::Goal goal;
    };
extern "C" {
#else
typedef struct PlatformSpecificInstructions PlatformSpecificInstructions;
#endif

    struct PlatformSpecificInstructions* convert_to_platform_specific (struct InstructionPackage* package);

    //simply a call to delete what is passed
    void freethepackage (struct PlatformSpecificInstructions* instance);


#ifdef __cplusplus
}
#endif
#endif
