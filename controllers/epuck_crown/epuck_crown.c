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

#include "../auct_super/message.h" 
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag leds[10];


#define DEBUG 1
#define TIME_STEP           64      // Timestep (ms)
#define RX_PERIOD           2    // time difference between two received elements (ms) (1000)

#define AXLE_LENGTH         0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS     0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205  // Wheel radius (meters)
#define DELTA_T             TIME_STEP/1000   // Timestep (seconds)
#define MAX_SPEED         800     // Maximum speed

#define INVALID          -999
#define BREAK            -999 //for physics plugin

#define NUM_ROBOTS 5 // Change this also in the supervisor!
#define EVENT_RANGE (0.1)

#define MAX_WORK_TIME (120.0*1000) // 120s of maximum work time
#define MAX_SIMULATION_TIME (180.0*1000) // 180s of simulation time
#define MAX_TASKS 3
#define RATE_OF_MOVEMENT 2.0 // how much time required to travel 1 unit of distance (2 seconds per meter travelled)

#define AVG_TASK_PER_SECOND (20.0/(MAX_SIMULATION_TIME*NUM_ROBOTS)*1000)


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* Collective decision parameters */

#define STATECHANGE_DIST 90   // minimum value of all sensor inputs combined to change to obstacle avoidance mode

typedef enum {
    STAY            = 1,
    GO_TO_GOAL      = 2,                    // Initial state aliases
    OBSTACLE_AVOID  = 3,
    RANDOM_WALK     = 4,
    DOING_TASK      = 5,
    DISABLED        = 6,
} robot_state_t;

#define DEFAULT_STATE (STAY)

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* e-Puck parameters */

#define NB_SENSORS           8
#define BIAS_SPEED           300

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
int lmsg, rmsg;             // Communication variables
int indx;                   // Event index to be sent to the supervisor

float buff[99];             // Buffer for physics plugin

double stat_max_velocity;
int worked_time = 0;


// Proximity and radio handles
WbDeviceTag emitter_tag, receiver_tag;
static WbDeviceTag ds[NB_SENSORS];  // Handle for the infrared distance sensors
// static WbDeviceTag radio;            // Radio


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* helper functions */

// Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}

void limit(int *number, int limit) {
    if (*number > limit)
        *number = limit;
    if (*number < -limit)
        *number = -limit;
}

double dist(double x0, double y0, double x1, double y1) {
    return sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
}


// calculate the value of waiting and not working for a time
double calculate_time_value(float time) {
    float work_time_remain = MAX_WORK_TIME - worked_time;
    float simulation_time_remain = MAX_SIMULATION_TIME - clock;
    float time_factor = (simulation_time_remain - work_time_remain) / (MAX_SIMULATION_TIME - MAX_WORK_TIME);
    //printf("Waiting evaluation: factor %0.2f, time %0.2f, total %f\n", time_factor, time, AVG_TASK_PER_SECOND * time * time_factor);
    return AVG_TASK_PER_SECOND * time * time_factor;
}

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
            fprintf(stderr, "Invalid message: robot_id %d "  "doesn't match receiver %d\n", msg.robot_id, robot_id);
            //return;
            exit(1);
        }

        //find target list length
        i = 0;
        while(round(target[i][2]) != INVALID){ i++;}
        target_list_length = i;  
        
        if(target_list_length == 0) target_valid = 0;   

        
        // Event state machine
        if(msg.event_state == MSG_EVENT_GPS_ONLY)
        {
            my_pos[0] = msg.robot_x;
            my_pos[1] = msg.robot_y;
            my_pos[2] = msg.heading;
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
            // If event is done, delete it from array 
            for(i=0; i<=target_list_length; i++)
            {
                if((int)round(target[i][2]) == msg.event_id) 
                { //look for correct id (in case wrong event was done first)
                    for(; i<=target_list_length; i++)
                    { //push list to the left from event index
                        target[i][0] = target[i+1][0];
                        target[i][1] = target[i+1][1];
                        target[i][2] = target[i+1][2];
                    }
                    target[target_list_length+1][2] = INVALID;
                }
            }
            // adjust target list length
            if(target_list_length-1 == 0) target_valid = 0; //used in general state machine 
            target_list_length = target_list_length-1;    
        }
        else if(msg.event_state == MSG_EVENT_WON)
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
        // check if new event is being auctioned
        else if(msg.event_state == MSG_EVENT_NEW)
        {                
            // ///*** BEST TACTIC ***///

            indx = 0;
            double d = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);

            uint64_t completion_time;
            if (msg.event_type == 0) {     // Type A task
                completion_time = (robot_id % 2 == 0) ? 9: 3; // 3 or 9 seconds
            } else {                        // Type B task
                completion_time = (robot_id % 2 == 0) ? 1: 5; // 5 or 1 seconds
            }

            if (target_list_length > 0) {
                for (i = 0; i < target_list_length; i++) {
                    if (i == 0) {
                        double dbeforetogoal = dist(my_pos[0], my_pos[1], msg.event_x, msg.event_y);
                        double daftertogoal  = dist(target[i][0], target[i][1], msg.event_x, msg.event_y);
                        double dbeforetodafter = dist(my_pos[0], my_pos[1], target[i][0], target[i][1]);
                        d = dbeforetogoal + daftertogoal - dbeforetodafter;
                    } else {
                        double dbeforetogoal = dist(target[i-1][0], target[i-1][1], msg.event_x, msg.event_y);
                        double daftertogoal  = dist(target[i][0], target[i][1], msg.event_x, msg.event_y);
                        double dbeforetodafter = dist(target[i-1][0], target[i-1][1], target[i][0], target[i][1]);
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

            // don't bid if waiting is better
            if (calculate_time_value(task_time) > 1)
                task_time = 1.0/0.0;

            // don't bid if DISABLED or has already maximum tasks planned
            if (target_list_length >= MAX_TASKS || state == DISABLED)
                task_time = 1.0/0.0;
        
            const bid_t my_bid = {robot_id, msg.event_id, task_time, indx};
                
            // Send my bid to the supervisor
            // print bid
            //printf("Robot %d bidding %.2f for event %d at index %d\n", robot_id, d, msg.event_id, indx);
        
            wb_emitter_set_channel(emitter_tag, robot_id+1);
            wb_emitter_send(emitter_tag, &my_bid, sizeof(bid_t));
            // print emmitter channel
            //printf("Sent bid from robot %d on channel %d\n", robot_id, wb_emitter_get_channel(emitter_tag));            
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

    receiver_tag = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver_tag, RX_PERIOD); // listen to incoming data every 1000ms
    wb_receiver_set_channel(receiver_tag, robot_id+1);
    
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

    if (_sum_distances > STATECHANGE_DIST && state == GO_TO_GOAL) {
        state = OBSTACLE_AVOID;
    } else if (target_valid && task_dist >= EVENT_RANGE) {

int obstacle_detected_now = 0;

    if (target_valid && dist(my_pos[0], my_pos[1], target[0][0], target[0][1]) < 0.1)
    {
    state = STAY;
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
    
    if (obstacle_detected_now) 
    {
        state = OBSTACLE_AVOID;  
    }
    
    else if (target_valid)
    {
        state = GO_TO_GOAL;
    }
    else
    {
        state = DEFAULT_STATE;
    }
}

// Odometry
void update_self_motion(int msl, int msr) {
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
    if (my_pos[2] > 2*M_PI) my_pos[2] -= 2.0*M_PI;
    if (my_pos[2] < 0) my_pos[2] += 2.0*M_PI;

    // Keep track of highest velocity for modelling
    double velocity = du * 1000.0 / (double) TIME_STEP;
    if (state == GO_TO_GOAL && velocity > stat_max_velocity)
        stat_max_velocity = velocity;
}


// Compute wheel speed to avoid obstacles
/*void compute_avoid_obstacle(int *msl, int *msr, int distances[]) 
{
    int d1=0,d2=0;       // motor speed 1 and 2     
    int sensor_nb;       // FOR-loop counters    

    for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++)
    {   
       d1 += (distances[sensor_nb]) * Interconn[sensor_nb];
       d2 += (distances[sensor_nb]) * Interconn[sensor_nb + NB_SENSORS];
    }
    d1 /= 80; d2 /= 80;  // Normalizing speeds

    *msr = d1+BIAS_SPEED; 
    *msl = d2+BIAS_SPEED; 
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
}*/


void compute_avoid_obstacle(int *msl, int *msr, int distances[])
{
    int th_obstacle = 80;
    int th_free = 75;
    int turn_speed = 300;
    int forward_speed = 100;

    int front = distances[0] + distances[7];
    int back =  distances[3] + distances[4];
    int left = distances[5] + distances[6];
    int right = distances[1] + distances[2];
    
    
    *msr = forward_speed;
    *msl = forward_speed;
    
    if (front > 2*th_obstacle)
    {
        if (left < right)
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
    else if (left + front > 4*th_obstacle && left < 2*th_free)
    {
        *msr = forward_speed - 20;
        *msl = forward_speed + 20;   

    }
    else if (right + front > 4*th_obstacle && left < 2*th_free)
    {
        *msr = forward_speed + 20;
        *msl = forward_speed - 20;   
    }
    
    else if (left > 2*th_obstacle && right < 2*th_free)
    {
        *msr = forward_speed - 10;
        *msl = forward_speed + 10;   

    }
    else if (right > 2*th_obstacle && left < 2*th_free)
    {
        *msr = forward_speed + 10;
        *msl = forward_speed - 10;   
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
    float Kw = 10.0;  // Rotational control coefficient
    float range = 1; //sqrtf(x*x + y*y);   // Distance to the wanted position
    float bearing = atan2(y, x);     // Orientation of the wanted position
    
    // Compute forward control
    float u = Ku*range*cosf(bearing);
    // Compute rotational control
    float w = Kw*range*sinf(bearing);
    
    // Convert to wheel speeds!
    *msl = 20*(u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    *msr = 20*(u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
   
}

// RUN e-puck
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
    msl_w = msl*MAX_SPEED_WEB/1000;
    msr_w = msr*MAX_SPEED_WEB/1000;
    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);
    update_self_motion(msl, msr);

    if (state != STAY && state != DISABLED) {
        worked_time += ms;
        //printf("Worked for %0.2f \n", worked_time);
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
