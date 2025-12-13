from random import uniform
import math
from bidding import *
import heapq
from plotting import *
import numpy as np

DEBUG = False

def random_pos():
    return uniform(0, ARENA_SIZE) - ARENA_SIZE / 2

class Statistics:
    def __init__(self):
        self.completed_tasks = 0
        self.robot_working = []
        self.task_times = []
        self.idle_times = []
        self.received_bids = []

class Task:
    def __init__(self, id):
        self.pos = (random_pos(), random_pos())
        
        type_probality = uniform(0,1)
        for a in range(NUMBER_OF_TYPES):
            cummulative_probability = sum(SPAWN_PROBABILITIES[:a+1])
            if type_probality <= cummulative_probability:
                self.type = a
                break
        self.completed = False
        self.id = id
        self.assigned_to = None
        self.total_worked_time = 0


class Robot:
    def __init__(self, robot_type, auctioneer, id):
        self.pos = (random_pos(), random_pos())
        self.tasks = []
        self.type = robot_type
        self.worked_time = 0
        self.working_now = False
        self.stopped_working_time = 0
        self.auctioneer = auctioneer
        self.id = id
        self.task_worked_time = 0
        self.waypoints = []

    def compute_best_insertion(self, task):
        if len(self.tasks) == 0:
            return 0, distance_walls(self.pos, task.pos)  # no targets

        d = float('inf')
        indx = -1
        
        for i in range(len(self.tasks)):
            if i == 0:
                dbeforetogoal = distance_walls(self.pos, task.pos)
                daftertogoal = distance_walls(task.pos, self.tasks[0].pos)
                dbeforetodafter = distance_walls(self.pos, self.tasks[0].pos)
            else:
                dbeforetogoal = distance_walls(self.tasks[i-1].pos, task.pos)
                daftertogoal = distance_walls(task.pos, self.tasks[i].pos)
                dbeforetodafter = distance_walls(self.tasks[i-1].pos, self.tasks[i].pos)

            val = dbeforetogoal + daftertogoal - dbeforetodafter

            if val < d:
                d = val
                indx = i

            # Check last element → append at end
            if i == len(self.tasks) - 1:
                if daftertogoal < d:
                    d = daftertogoal
                    indx = i + 1

        return indx, d

    # calculate work time for a given task and bid with time
    def bid_for_task(self, task):
        if len(self.tasks) >= MAX_TASKS_ASSIGNED:
            return -1, -1  # Cannot take more tasks

        task_time = TASK_WORK_TIMES[task.type][self.type]
        index, distance = self.compute_best_insertion(task)
        work_time = task_time + distance / ROBOT_SPEED

        time_value = calculate_time_value(work_time, self.worked_time, self.auctioneer.clock())

        self.auctioneer.simulation.statistics.received_bids += [work_time]
        
        if self.worked_time + work_time >= MAX_WORK_TIME:
            return -1, -1  # Cannot take this task
        
        if time_value > 1: # Not profitable => could do more than one task in given time 
            return -1, -1 
        if DEBUG:
            print(f"Robot {self.id} bidding for task {task.id} with work time {work_time:.2f}, time value {time_value:.2f}, at index {index}")
        
        return work_time, index
    
    def step(self, dt):
        if not self.working_now:
            return
        
        self.tasks[0].total_worked_time += dt

        if len(self.waypoints) > 0:
            target = self.waypoints[0]
            dir_x = target[0] - self.pos[0]
            dir_y = target[1] - self.pos[1]
            distance = math.sqrt(dir_x**2 + dir_y**2)

            if distance < ROBOT_SPEED * dt:
                self.pos = target
                self.waypoints.pop(0)
            else:
                norm_dir_x = dir_x / distance
                norm_dir_y = dir_y / distance
                self.pos = (self.pos[0] + norm_dir_x * ROBOT_SPEED * dt,
                            self.pos[1] + norm_dir_y * ROBOT_SPEED * dt) 
                
        if self.pos == self.tasks[0].pos:
            if self.task_worked_time < TASK_WORK_TIMES[self.tasks[0].type][self.type]:
                self.task_worked_time += dt
            else:
                self.task_worked_time = 0
                task = self.tasks.pop(0)
                self.auctioneer.task_completed(task) 
                if len(self.tasks) > 0:
                    self.waypoints = get_path_waypoints(self, self.tasks[0])
                else:
                    self.working_now = False
                    self.stopped_working_time = self.auctioneer.clock()

        
        if len(self.waypoints) == 0 and self.working_now and len(self.tasks) > 0:
            # Move towards next task if no waypoints left
            next_task = self.tasks[0]
            self.waypoints = get_path_waypoints(self, next_task)
    

class Auctioneer:
    def __init__(self, simulation=None):
        self.robots = []
        for robot_type in range(NUMBER_OF_TYPES):
            self.robots += [Robot(robot_type, self, len(self.robots)+1+i) for i in range(ROBOT_COUNTS[robot_type])]

        self.tasks = []
        self.tasks_to_auction = []
        self.simulation = simulation

        for _ in range(TASK_COUNT):
            self.create_task()
        self.auction_all_tasks()

    def clock(self):
        return self.simulation.clock

    def create_task(self):
        task = Task(len(self.tasks)+1)
        self.tasks.append(task)
        self.tasks_to_auction.append(task)
        return task
    
    def auction_task(self, task):
        best_bid = float('inf')
        best_robot = None
        best_index = -1
        for robot in self.robots:
            bid, index = robot.bid_for_task(task)
            if bid != -1 and bid < best_bid:
                best_bid = bid
                best_robot = robot
                best_index = index
                
        if best_robot is not None:
            best_robot.tasks.insert(best_index, task)
            best_robot.worked_time += best_bid
            if not best_robot.working_now:
                self.simulation.statistics.idle_times += [self.clock() - best_robot.stopped_working_time]
            best_robot.working_now = True
            task.assigned_to = best_robot
            if DEBUG:
                print(f"Task {task.id} assigned to Robot {best_robot.id} at index {best_index}")
            best_robot = get_path_waypoints(best_robot, task)
            return True

    def auction_all_tasks(self):
        index = 0
        while index < len(self.tasks_to_auction):
            task = self.tasks_to_auction[index]
            assigned = self.auction_task(task)
            if assigned:
                self.tasks_to_auction.remove(task)
            else:
                index += 1  
    
    def task_completed(self, task):
        task.completed = True
        self.simulation.statistics.task_times += [task.total_worked_time]
        robot = task.assigned_to
        if robot:
            if DEBUG:
                print(f"Task {task.id} completed by Robot {robot.id}")
            if len(robot.tasks) == 0:
                robot.working_now = False
        self.simulation.statistics.completed_tasks += 1

        self.create_task() # create a new task when one is completed
        self.auction_all_tasks()

    

class Simulation:
    def __init__(self):
        self.clock = 0
        self.dt = 0.032
        self.statistics = Statistics()
        self.auctioneer = Auctioneer(self)

    def run_simulation(self):
        while self.clock <= MAX_SIMULATION_TIME:
            for robot in self.auctioneer.robots:
                robot.step(self.dt)
            self.clock += self.dt
            self.statistics.robot_working += [[robot.working_now for robot in self.auctioneer.robots]]



if __name__ == "__main__":
    simulation = Simulation()
    simulation.run_simulation()
    print(f"Simulation completed. Total completed tasks: {simulation.statistics.completed_tasks}")

    #plot_active_robots(simulation.statistics.robot_working, simulation.dt)

    average_robot_working = np.array(simulation.statistics.robot_working)*1
    worked_times = []
    received_bids = []
    idle_times = []

    tasks = 0
    NUM_SIMUL = 100
    import time
    start_time = time.time()
    for a in range(NUM_SIMUL):
        simulation = Simulation()
        simulation.run_simulation()
        tasks += simulation.statistics.completed_tasks
        average_robot_working += np.array(simulation.statistics.robot_working)*1
        worked_times += simulation.statistics.task_times
        received_bids += simulation.statistics.received_bids
        idle_times += simulation.statistics.idle_times

    print(f"Run time is {(time.time()-start_time)/NUM_SIMUL}")


    print(f"Completed all simulations, average tasks: {tasks/NUM_SIMUL}")
    histogram(received_bids)
    histogram(worked_times)    
    print("Work time mean:", np.mean([time for time in worked_times if time >10]))
    zero_prop = sum([1 for value in idle_times if value <0.1]) / len([1 for value in idle_times if value >=0.1])
    print("Work right after change:", zero_prop, "%")
    idle_times = [value for value in idle_times if value <= 30 and value >=0.1]
    print("Idle mean time", np.mean(idle_times))
    histogram(idle_times)
    import numpy as np
    

    # This is used to get the distribution for the Mili model
    # max_value = 60
    # distribution = np.array([0 for _ in range(10*max_value+1)], dtype=float)
    # for value in worked_times:
    #     index = int(round(value*10))
    #     if index <= 10*max_value:
    #         distribution[index] += 1
    # distribution /= len(worked_times)
    # np.save("distribution.npy", distribution)

    plot_active_robots(average_robot_working/(NUM_SIMUL+1.0), simulation.dt)