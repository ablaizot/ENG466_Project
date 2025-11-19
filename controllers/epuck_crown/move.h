
#define TIME_STEP           64      // Timestep (ms)

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205  // Wheel radius (meters)
#define DELTA_T             (TIME_STEP/1000.0)   // Timestep (seconds)
#define MAX_SPEED         1000     // Maximum speed
#define NUM_ROBOTS 5 // Change this also in the supervisor!

typedef enum {
    STAY            = 1,
    GO_TO_GOAL      = 2,                    // Initial state aliases
    OBSTACLE_AVOID  = 3,
    RANDOM_WALK     = 4,
    DOING_TASK      = 5,
    DISABLED        = 6,
} robot_state_t;

#include <math.h>
#include <stdint.h>



void compute_avoid_obstacle(int *msl, int *msr, int distances[]);
void compute_go_to_goal(int *msl, int *msr);
void update_self_motion(int msl, int msr);


