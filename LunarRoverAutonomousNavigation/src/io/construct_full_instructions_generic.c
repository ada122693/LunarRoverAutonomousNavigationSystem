// TEMPORARILY DISABLED - Switched to pose-based instructions (nav_generateinstructions_global)
// This file is for the segment-based instruction format (dheading, velocity, duration)
// To use this method instead, everything is simply commented out:
// here, in send_main.c, CMakeLists.txt, and structs.h (the WaitHookInfo and Instruction structs)

/*
#include "structs.h"
#include "constants/constants.h"
#include <stdlib.h>

//complete instruction package contains an instructions array like
//instructions package, but no wait hooks array or sectorstraversal, etc
//it still has the same metadata
struct CompleteInstructionPackage {
    struct Instruction* instructions;
    struct InstructionMetadata metadata;
};

struct CompleteInstructionPackage construct_full_instructions_generic(struct InstructionPackage *package) {
    //loop through each instruction in the instruction package
    //take all wait hooks which are during this instruction, as evidenced by their instruction
    //index and whether they are real (>0.0)
    //split the instruction at the beforedur of the wait hook
    //make an instruction for velocity 0.0  for the duration of the wait
    //repeat until instruction is complete
    //do the same to the next until they are all done
    
    struct CompleteInstructionPackage result;
    result.metadata = package->metadata;

    struct Instruction *startnode = package->instructions;
    struct Instruction *current = startnode;
    int instruction_index = 0;
    int running_wait_hook_index = 0;

    while (current != NULL) {
        double consumed_duration = 0.0;
        
        for (int i = running_wait_hook_index; i < package->metadata.num_wait_hooks; i++) {
            if (package->wait_hooks[i].instruction_index != instruction_index) {
                break;
            }
            
            if (package->wait_hooks[i].wait_duration > 0.0) {
                //make a copy of current
                struct Instruction *newAfter = malloc(sizeof(struct Instruction));
                if (newAfter == NULL) {
                    return NULL;
                }
                *newAfter = *current;

                newAfter->duration -= package->wait_hooks[i].beforeDur - consumed_duration;
                current->duration = package->wait_hooks[i].beforeDur - consumed_duration;

                //add instruction for velocity 0.0 for wait_duration
                struct Instruction *newWait = malloc(sizeof(struct Instruction));
                if (newWait == NULL) {
                    return NULL;
                }
                newWait->dheading = 0.0;
                newWait->velocity = 0.0;
                newWait->duration = package->wait_hooks[i].wait_duration;
                newWait->next = newAfter;

                current->next = newWait;
                current = newAfter;
                consumed_duration = package->wait_hooks[i].beforeDur;
            }
            running_wait_hook_index++;
        }
        current = current->next;
        instruction_index++;
    }
    result.instructions = startnode;

    return result;
}
*/