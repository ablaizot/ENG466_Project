/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:        epuck_crown.c
 * author:      
 * description: E-puck file for market-based task allocations (DIS lab05)
 *
 * $Revision$	february 2016 by Florian Maushart
 * $Date$
 * $Author$      Last update 2024 by Wanting Jin
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <webots/radio.h>
#include <webots/motor.h>
#include <webots/led.h>
  
#include <webots/supervisor.h> 
#include "../epuck_crown/move.h"

#include "../auct_super_short_multi/message.h" 
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag leds[10];


#define DEBUG 1
#define RX_PERIOD           2    // time difference between two received elements (ms) (1000)



#define INVALID          -999
#define BREAK            -999 //for physics plugin

#define NUM_ROBOTS 5 // Change this also in the supervisor!
#define EVENT_RANGE (0.09)
#define LOCAL_COMMUNICATION_RANGE 0.3 // Communication range for local communication
#define COMMON_CHANNEL 1 // Common channel for local communication


#define MAX_WORK_TIME (120.0*1000) // 120s of maximum work time
#define MAX_SIMULATION_TIME (180.0*1000) // 180s of simulation time
#define MAX_TASKS 3
#define RATE_OF_MOVEMENT 5.0 // how much time required to travel 1 unit of distance (2 seconds per meter travelled)

#define AVG_TASK_PER_SECOND (300.0/(MAX_SIMULATION_TIME*NUM_ROBOTS))*1000


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST 120   // minimum value of all sensor inputs combined to change to obstacle avoidance mode



#define DEFAULT_STATE (STAY)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */

#define NB_SENSORS           8
#define BIAS_SPEED           400

// Weights for the Braitenberg algorithm
// NOTE: Weights from reynolds2.h
int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18};


// The state variables
int clock;
uint16_t robot_id;          // Unique robot ID
robot_state_t state;                 // State of the robot
double my_pos[3];           // X, Z, Theta of this robot
char target_valid;          // boolean; whether we are supposed to go to the target
double target[10][3];       // x and z coordinates of target position (max 10 targets)
bid_t target_bids[10];    // bids for each target
int lmsg, rmsg;             // Communication variables
int indx;                   // Event index to be sent to the supervisor

float buff[99];             // Buffer for physics plugin

double stat_max_velocity;
int worked_time = 0;
bool initial_complete = false;

// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag, local_emitter_tag, local_receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}



double dist(double x0, double y0, double x1, double y1) {
    return sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
}

typedef struct { double x, y; } Vec2;

int orientation(Vec2 a, Vec2 b, Vec2 c) {
    double v = (b.y - a.y) * (c.x - b.x) -
               (b.x - a.x) * (c.y - b.y);
    if (v > 0) return 1;   // clockwise
    if (v < 0) return 2;   // counter-clockwise
    return 0;              // collinear
}
int onSegment(Vec2 a, Vec2 x, Vec2 b) {
    return (x.x <= fmax(a.x, b.x) && x.x >= fmin(a.x, b.x) &&
            x.y <= fmax(a.y, b.y) && x.y >= fmin(a.y, b.y));
}
int segmentsIntersect(Vec2 A, Vec2 B, Vec2 C, Vec2 D) {
    if (orientation(A, B, C) != orientation(A, B, D) && orientation(C, D, A) != orientation(C, D, B)) 
        return 1;
    return 0;
}
int intersectionPoint(Vec2 A, Vec2 B, Vec2 C, Vec2 D, Vec2 *out) {
    double x1=A.x, y1=A.y, x2=B.x, y2=B.y;
    double x3=C.x, y3=C.y, x4=D.x, y4=D.y;

    double denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if (fabs(denom) < 1e-9) return 0; // Parallel or collinear

    double px = ((x1*y2 - y1*x2)*(x3 - x4) -
                 (x1 - x2)*(x3*y4 - y3*x4)) / denom;

    double py = ((x1*y2 - y1*x2)*(y3 - y4) -
                 (y1 - y2)*(x3*y4 - y3*x4)) / denom;

    Vec2 P = {px, py};

    if (!onSegment(A, P, B) || !onSegment(C, P, D)) return 0;

    *out = P;
    return 1;
}

// calculate the value of waiting and not working for a time
double calculate_time_value(float time) {
    float work_time_remain = MAX_WORK_TIME - worked_time;
    float simulation_time_remain = MAX_SIMULATION_TIME - clock;
    float time_factor = (simulation_time_remain - work_time_remain) / (MAX_SIMULATION_TIME - MAX_WORK_TIME);
    //printf("Waiting evaluation: factor %0.2f, time %0.2f, total %f\n", time_factor, time, AVG_TASK_PER_SECOND * time * time_factor);
    return AVG_TASK_PER_SECOND * time * time_factor;
}

double calculate_distance_walls(double start_x, double start_y, double target_x, double target_y) {
    Vec2 startPos = {start_x, start_y};
    Vec2 targetPos = {target_x, target_y};

    Vec2 line1_A = {-0.45-0.1875, 0.0}; // wall 1
    Vec2 line1_B = {-0.45+0.1875, 0.0};

    Vec2 line2_A = {0.125, 0.225-0.425}; // wall 2
    Vec2 line2_B = {0.125, 0.225+0.425};
    

    Vec2 hit;

    double distance = dist(start_x, start_y, target_x, target_y);

    if (segmentsIntersect(startPos, targetPos, line1_A, line1_B)) {
        if (intersectionPoint(startPos, targetPos, line1_A, line1_B, &hit)) {
            //printf("Robot %d, crossed line 1 at: (%.2f, %.2f)\n", robot_id, hit.x, hit.y);
            distance += dist(hit.x, hit.y, line1_B.x, line1_B.y) * 2;
        }
    } 

    if (segmentsIntersect(startPos, targetPos, line2_A, line2_B)) {
        if (intersectionPoint(startPos, targetPos, line2_A, line2_B, &hit)) {
            //printf("Robot %d, crossed line 2 at: (%.2f, %.2f)\n", robot_id, hit.x, hit.y);
            distance += dist(hit.x, hit.y, line2_A.x, line2_A.y) * 2;
        }
    } 
    
    return distance;
}



void recalculate_marginal_bids()
{
    int targets_added = 0;
    // Find the current route length
    for (targets_added = 0; targets_added < MAX_TASKS; targets_added++) {
        if (target[targets_added][2] == INVALID) {
            break;
        }
    }
    
    if (targets_added == 0) return;

    // The bid for every task in the final route must be its cost of removal (its current marginal cost).
    for (int q = 0; q < targets_added; q++) {
        int event_id_to_check = (int)round(target[q][2]);
        
        int bid_idx = -1;
        for (int i = 0; i < 10; i++) {
            if (target_bids[i].event_id == event_id_to_check) {
                bid_idx = i;
                break;
            }
        }

        if (bid_idx != -1) {
            double task_completion_time;
            if (target_bids[bid_idx].event_type == 0) {
                task_completion_time = (robot_id % 2 == 0) ? 9 : 3;
            } else {
                task_completion_time = (robot_id % 2 == 0) ? 1 : 5;
            }

            double pre_x = (q == 0) ? my_pos[0] : target[q - 1][0];
            double pre_y = (q == 0) ? my_pos[1] : target[q - 1][1];

            double current_marginal_cost;
            
            if (q == targets_added - 1) {
                // Last task: Marginal cost is just travel from previous task + completion time.
                double d_pre_to_i = calculate_distance_walls(pre_x, pre_y, target[q][0], target[q][1]);
                current_marginal_cost = (RATE_OF_MOVEMENT * d_pre_to_i) + task_completion_time;
            } else {
                // Task in the middle: Recalculate based on Savings Heuristic.
                double post_x = target[q + 1][0];
                double post_y = target[q + 1][1];
                
                double d_pre_to_i = calculate_distance_walls(pre_x, pre_y, target[q][0], target[q][1]);
                double d_i_to_post = calculate_distance_walls(target[q][0], target[q][1], post_x, post_y);
                double d_pre_to_post = calculate_distance_walls(pre_x, pre_y, post_x, post_y);
                
                // Marginal Cost = (Travel(pre->i) + Completion(i) + Travel(i->post)) - Travel(pre->post)
                current_marginal_cost = (RATE_OF_MOVEMENT * (d_pre_to_i + d_i_to_post) 
                                        + task_completion_time)
                                        - (RATE_OF_MOVEMENT * d_pre_to_post);
            }
            
            // Update the bid and claim the task (since this calculation is only run for tasks IN our route)
            target_bids[bid_idx].value = current_marginal_cost;
            target_bids[bid_idx].winner_id = robot_id;
            target_bids[bid_idx].pos_x = my_pos[0]; 
            target_bids[bid_idx].pos_y = my_pos[1];
        }
    }
}

void greedy_route_choice()
{
    int i, k, q;
    int targets_added = 0;
    
    // 1. Determine the Current Route Length
    for (targets_added = 0; targets_added < MAX_TASKS; targets_added++) {
        if (target[targets_added][2] == INVALID) {
            break;
        }
    }
    
    // EXIT CONDITION: If the target list is already full (targets_added == MAX_TASKS), return.
    if (targets_added == MAX_TASKS) {
        target_valid = 1;
        recalculate_marginal_bids();
        return;
    }
    
    // Tracks which events (by target_bids index) are currently in the route.
    bool added_events[10] = {0}; 
    
    // Mark tasks already in the current route as 'added'.
    for (q = 0; q < targets_added; q++) {
        int event_id = (int)round(target[q][2]);
        for (i = 0; i < 10; i++) {
            if (target_bids[i].event_id == event_id) {
                added_events[i] = true;
                break;
            }
        }
    }

    // Main loop: iteratively find and insert the single best task
    while (targets_added < MAX_TASKS)
    {
        // Search variables for the best insertion
        int best_task_idx = -1;          // Index in target_bids of the task to insert
        int best_insertion_slot = -1;    // The optimal position (0 to targets_added) in the current route
        double min_marginal_cost = 1.0/0.0; // Infinity
        
        // 2. Evaluate all unassigned, claimed tasks for optimal insertion point
        for (i = 0; i < 10; i++)
        {  
              //printf("Robot %d: BID time VALUE %f and winner id %d \n", robot_id, target_bids[i].value, target_bids[i].winner_id);
            // Only consider tasks that are valid, not yet in the route, AND we are the current winner
            if (!added_events[i] && target_bids[i].event_id != INVALID && target_bids[i].winner_id == robot_id )
            {
                double task_completion_time;
                if (target_bids[i].event_type == 0) {
                    task_completion_time = (robot_id % 2 == 0) ? 9 : 3;
                } else {
                    task_completion_time = (robot_id % 2 == 0) ? 1 : 5;
                }
                
                // Try inserting task i into every possible slot k
                for (k = 0; k <= targets_added; k++)
                {
                    // a) Determine the 'pre' position (start of segment)
                    double pre_x = (k == 0) ? my_pos[0] : target[k - 1][0];
                    double pre_y = (k == 0) ? my_pos[1] : target[k - 1][1];
                    
                    // b) Calculate distance from pre to i 
                    double d_pre_to_i = calculate_distance_walls(pre_x, pre_y, 
                                                                 target_bids[i].event_x, 
                                                                 target_bids[i].event_y);
                    
                    // c) Calculate the marginal cost of insertion
                    double marginal_cost_k;
                    
                    if (k < targets_added) {
                        // Insertion between two existing tasks (or start/first task)
                        double post_x = target[k][0];
                        double post_y = target[k][1];
                        
                        double d_pre_to_post = calculate_distance_walls(pre_x, pre_y, post_x, post_y);
                        double d_i_to_post = calculate_distance_walls(target_bids[i].event_x, 
                                                                    target_bids[i].event_y, 
                                                                    post_x, post_y);
                        
                        // Marginal Cost: (Travel(pre->i) + Completion(i) + Travel(i->post)) - Travel(pre->post)
                        marginal_cost_k = (RATE_OF_MOVEMENT * (d_pre_to_i + d_i_to_post) 
                                        + task_completion_time)
                                        - (RATE_OF_MOVEMENT * d_pre_to_post);
                    } else {
                        // Insertion at the end of the route
                        // Marginal Cost: Travel(pre->i) + Completion(i)
                        marginal_cost_k = (RATE_OF_MOVEMENT * d_pre_to_i) + task_completion_time;
                    }
                    
                    // d) Check if this insertion is the best so far for *any* task
                    if (marginal_cost_k < min_marginal_cost) {
                        min_marginal_cost = marginal_cost_k;
                        best_task_idx = i;
                        best_insertion_slot = k;
                    }
                }
            }
        }
        
        // 3. Commit or Stop
        
        // If no profitable task found, stop.
        if (best_task_idx == -1 || calculate_time_value(min_marginal_cost) >= 1) {
            break; 
        }

        // Commit: Shift existing tasks to make space for the new one
        for (k = targets_added; k > best_insertion_slot; k--) {
            target[k][0] = target[k - 1][0];
            target[k][1] = target[k - 1][1];
            target[k][2] = target[k - 1][2]; 
        }
        
        // Insert the best task into its optimal slot
        target[best_insertion_slot][0] = target_bids[best_task_idx].event_x;
        target[best_insertion_slot][1] = target_bids[best_task_idx].event_y;
        target[best_insertion_slot][2] = target_bids[best_task_idx].event_id;
        
        // Mark the task as added
        added_events[best_task_idx] = true;
        targets_added++;
    }
    
    // Clear any tasks that might have been in the route but were implicitly removed
    for (q = targets_added; q < MAX_TASKS; q++) {
        target[q][2] = INVALID;
    }
    
    if (targets_added > 0){
        recalculate_marginal_bids();
    }

    
    // Finalize the route validity flag
    target_valid = (targets_added > 0) ? 1 : 0;
}
/*void greedy_route_choice()
{
    // Initialize parameters
    int i, k, q;
    int targets_added = 0;
    bool added_events[10] = {0};
    int max_iterations = 50;
    int iteration = 0;
    
    printf("=== Starting greedy_route_choic ===\n");
    
    for (i = 0; i< MAX_TASKS; i++){
            if (target[i][2] == INVALID){
                targets_added = i;
                break;
            }
            for (k=0; k<10; k++){
                if (target[i][2] == target_bids[k].event_id){
                     added_events[k] = true;   
                }
            }
            
        }

    // Main loop: build route one task at a time
    while (targets_added < MAX_TASKS && iteration < max_iterations)
    {
        iteration++;
        
        // Evaluate all candidate tasks for optimal insertion
        int best_task_idx = -1;
        int best_insertion_slot = -1; // The optimal position in the current route
        double best_marginal_cost = 1e9; // The lowest bid found
        
        // For each available task 
        for (i = 0; i < 10; i++)
        {
            if (!added_events[i] && target_bids[i].event_id != INVALID)
            {
                // Calculate completion time for task i
                double completion_time_i;
                if (target_bids[i].event_type == 0) {
                    completion_time_i = (robot_id % 2 == 0) ? 9 : 3;
                } else {
                    completion_time_i = (robot_id % 2 == 0) ? 1 : 5;
                }
                
                double min_marginal_i = 1e9;
                int optimal_slot_i = -1;
                
                // Try inserting task i into every possible slot k
                // k = 0: Insert between my_pos and first target
                // k = targets_added: Insert at the end of the route
                for (k = 0; k <= targets_added; k++)
                {
                    double marginal_cost_k = 0.0;
                    double d_pre_to_i, d_i_to_post;
                    
                    // Determine the position 
                    double pre_x = (k == 0) ? my_pos[0] : target[k - 1][0];
                    double pre_y = (k == 0) ? my_pos[1] : target[k - 1][1];
                    
                    // Calculate distance from pre to i 
                    d_pre_to_i = calculate_distance_walls(pre_x, pre_y, 
                                                         target_bids[i].event_x, 
                                                         target_bids[i].event_y);
                    
                    // Total time added by this insertion
                    double time_added_to_i = RATE_OF_MOVEMENT * d_pre_to_i + completion_time_i;
                    
                    if (k < targets_added) {
                        // Insertion replaces an existing position
                     
                        // position after
                        double post_x = target[k][0];
                        double post_y = target[k][1];
                        
                        double d_pre_to_post = calculate_distance_walls(pre_x, pre_y, post_x, post_y);
                        
                        // Calculate distance from i to after
                        d_i_to_post = calculate_distance_walls(target_bids[i].event_x, 
                                                              target_bids[i].event_y, 
                                                              post_x, post_y);
                        
                        // Marginal Cost
                        marginal_cost_k = (RATE_OF_MOVEMENT * (d_pre_to_i + d_i_to_post) 
                                            + completion_time_i)
                                        - (RATE_OF_MOVEMENT * d_pre_to_post);
                    } else {
                        // Last slot 
                        marginal_cost_k = time_added_to_i;
                    }
                    
                    // Compare it to current best one
                    if (marginal_cost_k < min_marginal_i) {
                        min_marginal_i = marginal_cost_k;
                        optimal_slot_i = k;
                    }
                } 
                
                // marginal cost as bidding
                if (min_marginal_i < target_bids[i].value || target_bids[i].winner_id == robot_id) {
                
                    //printf("Time value greedy %f \n" , calculate_time_value(min_marginal_i))
                    // Check if value is good enoug   
                    if (calculate_time_value(min_marginal_i) < 1) {
                        
                        // Compare it to alreday existing bids
                        if (min_marginal_i < best_marginal_cost) {
                            best_marginal_cost = min_marginal_i;
                            best_task_idx = i;
                            best_insertion_slot = optimal_slot_i;
                        }
                    }
                }
            }
        }
        
        // If no good task found, stop
        if (best_task_idx == -1) {
            break;
        }
        
        // Shift existing target
        for (k = targets_added; k > best_insertion_slot; k--) {
            target[k][0] = target[k - 1][0];
            target[k][1] = target[k - 1][1];
            target[k][2] = target[k - 1][2];
        }
        
        // Insert the best task into its optimal slot
        target[best_insertion_slot][0] = target_bids[best_task_idx].event_x;
        target[best_insertion_slot][1] = target_bids[best_task_idx].event_y;
        target[best_insertion_slot][2] = target_bids[best_task_idx].event_id;
        
        // Update our bid
        target_bids[best_task_idx].value = best_marginal_cost;
        target_bids[best_task_idx].winner_id = robot_id;

        added_events[best_task_idx] = true;
        targets_added++;
    }
    
    target_valid = (targets_added > 0) ? 1 : 0;
    printf("Robot %d: Route complete with %d targets\n", robot_id, targets_added);
}*/

/*void greedy_route_choice()
{//Function taht sets a new greedy list of targets with only information about current target_bids

    // Initialize parameters
        int i;
        int targets_added = 0;
        bool added_events[10] = {0};
        double pos_x = my_pos[0];
        double pos_y = my_pos[1];
        double completion_time = 1.0;
        double prev_best_task_time = 0.0;
        int prev_best_idx = -1;

    // Find first invalid target so that the 
    for (i = 0; i < MAX_TASKS; i++) 
    {
        if(target[i][2] == INVALID){
            targets_added = i; // Set number of initial targets added
            if (i > 0){        
                pos_x = target[i-1][0]; //Set curr pos as target before invalid 
                pos_y = target[i-1][0];
                added_events[(int)target[i-1][2]] = true;
            }
            break;
        }
    }

    while (targets_added < MAX_TASKS) //Continue until all spots assigned
    {
        double best_task_time = 30.0; 
        int best_idx = -1;

        for (i = 0; i < 10; i++)
        {
            if (!added_events[i] && target_bids[i].event_id != INVALID) //Make sure not used and not invalid
            {
                // Calculate time to perform taks from last x,y position
                double d = calculate_distance_walls(pos_x, pos_y, target_bids[i].event_x, target_bids[i].event_y);

                if (target_bids[i].event_type == 0) { // Type A task
                    completion_time = (robot_id % 2 == 0) ? 9 : 3;
                } 
                else {// Type B task
                    completion_time = (robot_id % 2 == 0) ? 1 : 5;
                }

                double task_time = RATE_OF_MOVEMENT * d + completion_time; //Final which is compared to the previous best task time
                
              
                if (task_time < best_task_time) 
                {
                    best_task_time = task_time; //Update best performin
                    best_idx = i; //Index in targets bid list
                }
              
            }
        }
        // If no task is good enough braek the loop and stop searching, wait for a new event
        if (best_idx == -1 || calculate_time_value(best_task_time) > 1){
            break;
        }
        
        // Add the current target to target list
        target[targets_added][0] = target_bids[best_idx].event_x;
        target[targets_added][1] = target_bids[best_idx].event_y;
        target[targets_added][2] = target_bids[best_idx].event_id;
        
        
        if (targets_added > 0){
            double d_skip;
        
            double cost_with = prev_best_task_time + best_task_time;
            
            if (targets_added == 1){
                d_skip = calculate_distance_walls(my_pos[0], my_pos[1], target[1][0], target[1][1]);
            } else {
                d_skip = calculate_distance_walls(target[targets_added - 2][0], target[targets_added- 2][1], 
                                                         target[targets_added][0], target[targets_added][1]);
            }
            double cost_without = RATE_OF_MOVEMENT * d_skip + completion_time;
            double marginal_cost = cost_with - cost_without;
            
            target_bids[prev_best_idx].value = marginal_cost;
            target_bids[prev_best_idx].winner_id = robot_id;
            
         }
         
        prev_best_task_time = best_task_time;
        prev_best_idx = best_idx;
        
        // Update the position to be the one of the added target
        pos_x = target_bids[best_idx].event_x;
        pos_y = target_bids[best_idx].event_y;

        // Update bids in target_bids for the targets actually wanted to complete
        target_bids[best_idx].value = best_task_time;
        target_bids[best_idx].winner_id = robot_id;
        
        added_events[best_idx] = true;
        targets_added++; //Next iteration

    }
    target_valid = (targets_added > 0) ? 1 : 0; //If no target good, set target valid to 0. 

}*/

// Check if we received a message and extract information
static void receive_updates() 
{
    message_t msg;
    int target_list_length = 0;
    int i;
    int k;

    while (wb_receiver_get_queue_length(receiver_tag) > 0) {
        const message_t *pmsg = wb_receiver_get_data(receiver_tag);
        
        // save a copy, cause wb_receiver_next_packet invalidates the pointer
        memcpy(&msg, pmsg, sizeof(message_t));
        wb_receiver_next_packet(receiver_tag);
        //printf("Received message for robot %d\n", msg.robot_id);
        
        // print message
            //printf("Message details: event_id=%d, event_state=%d, event_x=%.2f, event_y=%.2f, event_index=%d\n",
                //msg.event_id, msg.event_state, msg.event_x, msg.event_y, msg.event_index);

        // double check this message is for me
        // communication should be on specific channel per robot
        // channel = robot_id + 1, channel 0 reserved for physics plguin
        if(msg.robot_id != robot_id) {
            //fprintf(stderr, "Warning: robot_id %d doesn't match receiver %d, skipping message\n", msg.robot_id, robot_id);
            continue; // Skip this message instead of crashing
        }

        //find target list length
        i = 0;
        while(round(target[i][2]) != INVALID){ i++;}
        target_list_length = i;  
        
        if(target_list_length == 0) target_valid = 0;   

        
        // Event state machine
        if(msg.event_state == MSG_EVENT_GPS_ONLY)
        {
            //printf("Original position of robot %d: %1.3f, %1.3f angle %1.3f\n",robot_id, my_pos[0], my_pos[1], my_pos[2]);
            my_pos[0] = msg.robot_x;
            my_pos[1] = msg.robot_y;
            my_pos[2] = msg.heading;
            
            //printf("New position of robot %d: %1.3f, %1.3f angle %1.3f\n",robot_id, my_pos[0], my_pos[1], my_pos[2]);
            continue;
        }
        else if(msg.event_state == MSG_QUIT)
        {
            // Set speed
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);
            wb_robot_step(TIME_STEP);
            exit(0);
        }
        else if(msg.event_state == MSG_EVENT_DONE)      
        {
            bool new_target = false;
            // If event is done, delete it from array 
            for(i=0; i<=target_list_length; i++)
            {
                if((int)round(target[i][2]) == msg.event_id) 
                { //look for correct id (in case wrong event was done first)
                    new_target = true;
                    for(; i<=target_list_length; i++)
                    { //push list to the left from event index
                        target[i][0] = target[i+1][0];
                        target[i][1] = target[i+1][1];
                        target[i][2] = target[i+1][2];
                    }
                    target[i+1][2] = INVALID;
                    if(target_list_length-1 == 0) target_valid = 0; //used in general state machine 
                    target_list_length = target_list_length-1;    
                }
            }

            // adjust target list length

            // do the same for target bids
            for(i=0; i<10; i++)
            {
                if(target_bids[i].event_id == msg.event_id)
                {
                    for(; i<9; i++)
                    {
                        target_bids[i] = target_bids[i+1];
                    }
                    target_bids[9].event_id = INVALID;
                    break;
                }
            } 
            
            if (new_target){
              new_target = false;
              greedy_route_choice();
            }          

        }
        /*else if(msg.event_state == MSG_EVENT_WON)
        {
            // insert event at index
            for(i=target_list_length; i>=msg.event_index; i--)
            {
                target[i+1][0] = target[i][0];
                target[i+1][1] = target[i][1];
                target[i+1][2] = target[i][2];
            }
            target[msg.event_index][0] = msg.event_x;
            target[msg.event_index][1] = msg.event_y;
            target[msg.event_index][2] = msg.event_id;
            target_valid = 1; //used in general state machine
            target_list_length = target_list_length+1;
        }
        // check if new event is being auctioned*/
        else if(msg.event_state == MSG_EVENT_NEW)
        {   
            int i; 

            //Insert events in event list without order
            int insert_pos = 0;
            for (i = 0; i < 10; i++) 
            {
                if (target_bids[i].event_id == INVALID) 
                {
                    insert_pos = i;
                    break;
                }
            }
            
            //Initially set ourself as winner of event but with value of inifity

            target_bids[insert_pos].event_id = msg.event_id;
            target_bids[insert_pos].event_x = msg.event_x;
            target_bids[insert_pos].event_y = msg.event_y;
            target_bids[insert_pos].event_type = msg.event_type;
            target_bids[insert_pos].robot_id = robot_id;
            target_bids[insert_pos].winner_id = robot_id;
            target_bids[insert_pos].value = 1.0/0.0; //inf
            target_bids[insert_pos].pos_x = my_pos[0]; //inf
            target_bids[insert_pos].pos_y = my_pos[1]; //inf
            
            

        
            // If first ten are complete then start with initial greedy route choice
            // Otherwise go to single insertion if new event is announced
            if (!initial_complete)
            {
                if (msg.event_id < 9)
                {
                    return;
                }
                greedy_route_choice();
                initial_complete = true;
                return; 
            }
            
            greedy_route_choice();
            
            
            
            // ///*** BEST TACTIC ***///

            /*indx = 0;
            double d = calculate_distance_walls(my_pos[0], my_pos[1], msg.event_x, msg.event_y);

            int completion_time;
            if (msg.event_type == 0) {     // Type A task
                completion_time = (robot_id % 2 == 0) ? 9: 3; // 3 or 9 seconds
            } else {                        // Type B task
                completion_time = (robot_id % 2 == 0) ? 1: 5; // 5 or 1 seconds
            }


            if (target_list_length > 0) {
                for (i = 0; i < target_list_length; i++) {
                    if (i == 0) {
                        double dbeforetogoal = calculate_distance_walls(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
                        double daftertogoal  = calculate_distance_walls(target[i][0], target[i][1], msg.event_x, msg.event_y);
                        double dbeforetodafter = calculate_distance_walls(my_pos[0], my_pos[1], target[i][0], target[i][1]);
                        d = dbeforetogoal + daftertogoal - dbeforetodafter;
                    } else {
                        double dbeforetogoal = calculate_distance_walls(target[i-1][0], target[i-1][1], msg.event_x, msg.event_y);
                        double daftertogoal  = calculate_distance_walls(target[i][0], target[i][1], msg.event_x, msg.event_y);
                        double dbeforetodafter = calculate_distance_walls(target[i-1][0], target[i-1][1], target[i][0], target[i][1]);
                        if ((dbeforetogoal + daftertogoal - dbeforetodafter) < d) {
                            d = dbeforetogoal + daftertogoal - dbeforetodafter;
                            indx = i;
                        }
                        if (i == target_list_length - 1) {
                            if (daftertogoal < d) {
                                d = daftertogoal;
                                indx = i + 1;
                            }
                        }
                    }
                    
                }
            }
            
            double task_time = RATE_OF_MOVEMENT * d + completion_time;
            
            printf("time value new event %f \n", calculate_time_value(task_time));

            // don't bid if waiting is better
            if (calculate_time_value(task_time) > 1)
                task_time = 1.0/0.0;

            // don't bid if DISABLED or has already maximum tasks planned
            if (target_list_length >= MAX_TASKS || state == DISABLED)
                task_time = 1.0/0.0;
            
            // Create a new bid with the task time
            //bid_t new_bid = {robot_id, msg.event_id, task_time, msg.event_x, msg.event_y, msg.event_type};
            //printf("Robot %d calculated bid for event %d with value %.2f\n", robot_id, new_bid.event_id, new_bid.value);

            //Insert new task at best calculated index in target list and in target bids if task time is low enough

            if (task_time < 1.0/0.0)
            {
                for(i=target_list_length; i>=indx; i--)
                {
                    target[i+1][0] = target[i][0];
                    target[i+1][1] = target[i][1];
                    target[i+1][2] = target[i][2];   
                }

                target[indx][0] = msg.event_x;
                target[indx][1] = msg.event_y;
                target[indx][2] = msg.event_id;
                target_valid = 1; //used in general state machine
                target_list_length = target_list_length+1;

                target_bids[insert_pos].value = task_time;
            }   


            
        target_valid = (target_list_length > 0) ? 1 : 0;*/
        }
    }
    
    
    // Communication with physics plugin (channel 0)            
        i = 0; k = 1;
        while((int)round(target[i][2]) != INVALID){i++;}
        target_list_length = i; 
        if(target_list_length > 0)
        {        
            // Line from my position to first target
            wb_emitter_set_channel(emitter_tag,0); 
            buff[0] = BREAK; // draw new line
            buff[1] = my_pos[0]; 
            buff[2] = my_pos[1];
            buff[3] = target[0][0];
            buff[4] = target[0][1];
            // Lines between targets
            for(i=5;i<5*target_list_length-1;i=i+5)
            {
                buff[i] = BREAK;
                buff[i+1] = buff[i-2]; 
                buff[i+2] = buff[i-1];
                buff[i+3] = target[k][0]; 
                buff[i+4] = target[k][1];
                k++;  
            }
            // send, reset channel        
            if(round(target[0][2]) == INVALID){ buff[0] = my_pos[0]; buff[1] = my_pos[1];}
            wb_emitter_send(emitter_tag, &buff, (5*target_list_length)*sizeof(float));
           // printf("Sent path from robot %d on channel %d\n", robot_id, wb_emitter_get_channel(emitter_tag));               
            wb_emitter_set_channel(emitter_tag,robot_id+1);      
        }
}




/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* RESET and INIT (combined in function reset()) */
void reset(void) 
{
    wb_robot_init();
    int i;

  //get motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  for (int i = 0; i < 10; i++) {
    char ledname[16];
    sprintf(ledname, "led%d", i);
    leds[i] = wb_robot_get_device(ledname);
  }

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
    char s[4] = "ps0";
    for(i=0; i<NB_SENSORS;i++) 
    {
        // the device name is specified in the world file
        ds[i]=wb_robot_get_device(s);      
        s[2]++; // increases the device number
        wb_distance_sensor_enable(ds[i],64);
    } 

    clock = 0;
    indx = 0;
    
    // Init target positions to "INVALID"
    for(i=0;i<10;i++){ 
        target[i][0] = 0;
        target[i][1] = 0;
        target[i][2] = INVALID;
        target_bids[i].robot_id = robot_id;
        target_bids[i].event_id = INVALID;
        target_bids[i].value = -1;
        target_bids[i].event_x = 0;
        target_bids[i].event_y = 0;
    }


    // Start in the DEFAULT_STATE
    state = DEFAULT_STATE;

    // read robot id and state from the robot's name
    char* robot_name; 
    robot_name = (char*) wb_robot_get_name();
    int tmp_id;
    if (sscanf(robot_name, "e-puck%d", &tmp_id)) {robot_id = (uint16_t)tmp_id;} 
    else {fprintf(stderr, "ERROR: couldn't parse my id %s \n", robot_name); exit(1);}

    
    for (int i = 0; i < 8; i++) {
        wb_led_set(leds[i], robot_id%2);
    } 

    // Am I used in this simulation?
    if (robot_id >= NUM_ROBOTS) {
        fprintf(stderr, "Robot %d is not needed. exiting ...\n", robot_id); 
        wb_robot_cleanup(); 
        exit(0);
    }

    // Link with webots nodes and devices (attention, use robot_id+1 as channels, because
    // channel 0 is reseved for physics plugin)
    emitter_tag = wb_robot_get_device("emitter");
    wb_emitter_set_channel(emitter_tag, robot_id+1);

    local_emitter_tag = wb_robot_get_device("local_emitter");
    wb_emitter_set_range(local_emitter_tag, LOCAL_COMMUNICATION_RANGE);
    wb_emitter_set_channel(local_emitter_tag, COMMON_CHANNEL);

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id+1);

    local_receiver_tag = wb_robot_get_device("local_receiver");
    wb_receiver_enable(local_receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(local_receiver_tag, COMMON_CHANNEL);
    
    // Seed random generator
    srand(getpid());

    // Reset stats
    stat_max_velocity = 0.0;
}


void update_state(int *distances)
{
    // compute distance to goal
    float a = target[0][1] - my_pos[0];
    float b = target[0][1] - my_pos[1];
    float task_dist = sqrt(a*a + b*b);
    int obstacle_detected_now = 0;
    
    wb_led_set(leds[8], 0);

    if (state == DISABLED)
        return;

    if (worked_time >= MAX_WORK_TIME) {
        WbNodeRef self = wb_supervisor_node_get_self();
        WbFieldRef translation = wb_supervisor_node_get_field(self, "translation");
        const double new_pos[3] = { 0.5-0.25*robot_id, -0.75, 0 };
        wb_supervisor_field_set_sf_vec3f(translation, new_pos);
        printf("Disabled robot %d\n",robot_id);
        state = DISABLED;
        for (int i=0; i<10; i++) { // unbid for all event -> send -1 as bid
            if (round(target[i][2]) == INVALID)
                continue;
            
            const bid_t my_bid = {robot_id, round(target[i][2]), -1, i};
            printf("Robot %d abandonning event %1.0f :(\n", robot_id, target[i][2]);
            wb_emitter_set_channel(emitter_tag, robot_id+1);
            wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));
        }
        return;
    }

    if (target_valid && dist(my_pos[0], my_pos[1], target[0][0], target[0][1]) < EVENT_RANGE)
    {
        state = DOING_TASK;
        return;
    }
    

    for(int i=0; i < NB_SENSORS; i++) 
    {
        distances[i] = wb_distance_sensor_get_value(ds[i]);
        if (distances[i] > STATECHANGE_DIST) 
        {
            obstacle_detected_now = 1;
        }
    }
    
    if (obstacle_detected_now) {
        state = OBSTACLE_AVOID;  
    } else if (target_valid) {
        state = GO_TO_GOAL;
    } else {
        state = DEFAULT_STATE;
    }
}

void broadcast_targets()
{
    int i;
    int target_list_length = 0;

    //find target list length
    i = 0;
    
    // Broadcast target list nearby robots channel 1 is the common channel
    wb_emitter_set_channel(local_emitter_tag, COMMON_CHANNEL);
    wb_emitter_set_range(local_emitter_tag, LOCAL_COMMUNICATION_RANGE);
    
    greedy_route_choice();
    recalculate_marginal_bids();

    // broadcast target bids
    for (i = 0; i < 10; i++) {
        if (target_bids[i].event_id != INVALID && target_bids[i].value >= 0 && state != DISABLED) {
            wb_emitter_send(local_emitter_tag, &target_bids[i], sizeof(bid_t));
            //printf("Robot %d broadcasted bid for event %d with value %.2f\n", robot_id, target_bids[i].event_id, target_bids[i].value);
            //printf("Size of bid_t: %zu\n", sizeof(bid_t));
        }
    }
}

void receive_local_bids()
{
    greedy_route_choice();
    recalculate_marginal_bids();
    
    bid_t received_bid;
    int i, j, x, q;
    bool new_route = false;
    
    // Check for messages on the local/common channel
    while (wb_receiver_get_queue_length(local_receiver_tag) > 0) 
    {
        const bid_t *pbid = wb_receiver_get_data(local_receiver_tag);
        
        // Save a copy, cause wb_receiver_next_packet invalidates the pointer
        memcpy(&received_bid, pbid, sizeof(bid_t));
        wb_receiver_next_packet(local_receiver_tag);
        
        // Ignore our own bids and bid that are not from sender id robot
        if (received_bid.robot_id == robot_id || received_bid.winner_id == robot_id /*|| received_bid.winner_id != received_bid.robot_id*/ ) {
            continue;
        }
        
        // printf("Robot %d received bid from robot %d for event %d with value %.2f\n", 
        //        robot_id, received_bid.robot_id, received_bid.event_id, received_bid.value);
        
        // Check if this bid is for an event we're also bidding on
        for (i = 0; i < 10; i++) 
        {
            if (target_bids[i].event_id == received_bid.event_id && target_bids[i].event_id != INVALID) 
            {
                // Found matching event
                
                // If the other robot has a better (lower) bid, we should give up this event
                if (received_bid.value < target_bids[i].value && received_bid.value >= 0) 
                {
                    //printf("Robot %d: Other robot %d has better bid (%.2f < %.2f) for event %d, removing from my list\n",
                           //robot_id, received_bid.winner_id, received_bid.value, target_bids[i].value, received_bid.event_id);
                    // Setting winner bid in target_list to        
                    target_bids[i].value = received_bid.value;
                    target_bids[i].winner_id = received_bid.winner_id;
                    
                    // Remove this bid from our list by shifting remaining bids left
                    /*for (j = i; j < 9; j++) 
                    {
                        target_bids[j] = target_bids[j + 1];
                    }
                    target_bids[9].event_id = INVALID;
                    target_bids[9].value = -1;*/

                    // Check if it is is in current target list
                    for (x = 0; x < MAX_TASKS; x++)
                    {
                        if (target[x][2] == round(received_bid.event_id))
                        { 
                              double d_self = calculate_distance_walls(my_pos[0], my_pos[1], target[x][0], target[x][1]);
                              double d_other = calculate_distance_walls(received_bid.pos_x, received_bid.pos_y, target[x][0], target[x][1]);
                              
                              //if (d_self < d_other){
                                     //target_bids[i].value = received_bid.value - 0.001;
                                     //target_bids[i].winner_id = robot_id;
                               //} else {
                                     new_route = true;
                                     for (q = x; q < MAX_TASKS; q++){
                                             target[q][2] = INVALID;
                                     }
                                     break;
                               //}
                                     
                        }
                    }
                    
                }
                break; // Exit for loop when finding match that changes path
            }
        }
    }

    if (new_route) {
        new_route = false; 
        greedy_route_choice();
    }
}



void run(int ms)
{
    float msl_w, msr_w;
    // Motor speed and sensor variables	
    int msl=0,msr=0;                // motor speed left and right
    int distances[NB_SENSORS];  // array keeping the distance sensor readings
    //int sum_distances=0;        // sum of all distance sensor inputs, used as threshold for state change.  	

    // Other variables
    //int sensor_nb;

    // Add the weighted sensors values
    /*for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {  
        distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);
        sum_distances += distances[sensor_nb];
    }*/

    broadcast_targets();

    // Listen to bids from other robots on common channel
    receive_local_bids();

    // Get info from supervisor
    receive_updates();

    // State may change because of obstacles
    update_state(distances);

    // Set wheel speeds depending on state
    switch (state) {
        case STAY:
            msl = 0;
            msr = 0;
            break;

        case DOING_TASK:
            msl = 0;
            msr = 0;
            break;

        case DISABLED:
            msl = 0;
            msr = 0;
            break;

        case GO_TO_GOAL:
            compute_go_to_goal(&msl, &msr);
            break;

        case OBSTACLE_AVOID:
            compute_avoid_obstacle(&msl, &msr, distances);
            break;

        case RANDOM_WALK:
            msl = 400;
            msr = 400;
            break;

        default:
            printf("Invalid state: robot_id %d \n", robot_id);
    }
    // Set the speed
    
    //printf("Robot %d speeds %d, %d\n", robot_id, msl, msr);
    msl_w = msl*MAX_SPEED_WEB/1000;
    msr_w = msr*MAX_SPEED_WEB/1000;
    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    update_self_motion(msl, msr);

    if (state != STAY && state != DISABLED) {
        worked_time += ms;
        //printf("Worked for %d \n", worked_time);
    }

    // Update clock
    clock += ms;
}


// MAIN
int main(int argc, char **argv) 
{
    reset();
  
    // RUN THE MAIN ALGORIHM
    while (wb_robot_step(TIME_STEP) != -1) {run(TIME_STEP);}
    wb_robot_cleanup();

    return 0;
}