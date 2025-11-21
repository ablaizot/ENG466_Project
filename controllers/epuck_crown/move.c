#include "move.h"

extern double target[10][3];
extern double my_pos[3];
extern uint16_t robot_id;
float prev_bearing[NUM_ROBOTS] = {0};
extern robot_state_t state;
extern double stat_max_velocity;


void limit(int *number, int limit) {
    if (*number > limit)
        *number = limit;
    if (*number < -limit)
        *number = -limit;
}


void compute_avoid_obstacle(int *msl, int *msr, int distances[])
{
    int th_obstacle = 120;
    int base_speed = 600;
    int turn_speed = 800;
    int edge_speed = 400;
    int corr = 150;

    int front = distances[0] + distances[7];
    int front_left = distances[7] + distances[6];
    int front_right = distances[0] + distances[1];
    
    *msr = base_speed;
    *msl = base_speed;
    
    if (front > 2 * th_obstacle)
    {
        if (front_left < front_right)
        {
            *msr = turn_speed;
            *msl = -turn_speed;
        }
        else
        {
            *msr = -turn_speed;
            *msl = turn_speed;
        }
    }  
        
    else if (front_left > 2 * th_obstacle)
    {
        *msr = edge_speed - corr;
        *msl = edge_speed + corr;
       
    }
    
    else if (front_right > 2 * th_obstacle)
    {
        *msr = edge_speed + corr;
        *msl = edge_speed - corr;
        
    }

    
    limit(msl, MAX_SPEED);
    limit(msr, MAX_SPEED);
    
}

// Computes wheel speed to go towards a goal
void compute_go_to_goal(int *msl, int *msr) 
{
    // // Compute vector to goal
    float a = target[0][0] - my_pos[0];
    float b = target[0][1] - my_pos[1];
    // Compute wanted position from event position and current location
    float x =  a*cosf(my_pos[2]) - b*sinf(my_pos[2]); // x in robot coordinates
    float y =  a*sinf(my_pos[2]) + b*cosf(my_pos[2]); // y in robot coordinates

    float Ku = 0.2;   // Forward control coefficient
    float Kw = 5.0;  // Rotational control coefficient
    float Kd = 0.5;
    float range = 1;//sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);     // Orientation of the wanted position
    
    // Compute forward control
    float u = Ku*range*cosf(bearing);
    
    // Compute rotational control with Damping controller
    float d_bearing = bearing - prev_bearing[robot_id];
    
    if (d_bearing > M_PI) d_bearing -= 2.0 * M_PI;
    if (d_bearing < -M_PI) d_bearing += 2.0* M_PI;
    
    prev_bearing[robot_id] = bearing;
    
    float w = Kw*range*sinf(bearing) + Kd * d_bearing / DELTA_T;
    
    // Convert to wheel speeds!
    float scale_factor = 40.0;
    *msl = scale_factor*(u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    *msr = scale_factor*(u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    limit(msl,MAX_SPEED - 0.01);
    limit(msr,MAX_SPEED - 0.01);
   
}

// Odometry
double update_self_motion(int msl, int msr) {
    double theta = my_pos[2];
  
    // Compute deltas of the robot
    double dr = (double)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double dl = (double)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
    double du = (dr + dl)/2.0;
    double dtheta = (dr - dl)/AXLE_LENGTH;
  
    // Compute deltas in the environment
    double dx = du * cosf(theta);
    double dy = du * sinf(theta);
  
    // Update position
    my_pos[0] += dx;
    my_pos[1] -= dy;
    my_pos[2] -= dtheta;
    
    // Keep orientation within 0, 2pi
    if (my_pos[2] > M_PI) my_pos[2] -= 2.0*M_PI;
    if (my_pos[2] < -M_PI) my_pos[2] += 2.0*M_PI;

    // Keep track of highest velocity for modelling
    double velocity = du * 1000.0 / (double) TIME_STEP;

    if (velocity > 0.001) { // Only print if moving to reduce clutter
        //printf("Robot %d | State: %d | Velocity: %f m/s\n", robot_id, state, velocity);
    }

    
    if (state == GO_TO_GOAL && velocity > stat_max_velocity)
        stat_max_velocity = velocity;
    return velocity;
    
}