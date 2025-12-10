from constants import *
from plotting import *
import random

class Robot:
    def __init__(self, simulation):
        self.working = 0
        self.simulation = simulation
        self.work_to_do = 0
        self.worked_time = 0

    def calculate_time_value(self, time, worked_time, clock):
        work_time_remain = MAX_WORK_TIME - worked_time
        simulation_time_remain = MAX_SIMULATION_TIME - clock

        time_factor = (simulation_time_remain - work_time_remain) / (MAX_SIMULATION_TIME - MAX_WORK_TIME)

        return AVG_TASK_PER_SECOND * time * time_factor
    
    def step(self, ms):
        if self.worked_time >= MAX_WORK_TIME:
            self.working = 0
            return

        if self.working:
            self.worked_time += ms
            self.work_to_do -= ms

            if self.work_to_do <= 0:
                self.work_to_do = 0
                self.working = 0
        
        else:
            task_time = self.simulation.get_task_time()
            time_value = self.calculate_time_value(task_time, self.worked_time, self.simulation.clock)
            if time_value <= 1:
                self.working = 1
                self.work_to_do = task_time
        

class Simulation:
    def __init__(self):
        self.clock = 0
        self.dt = 0.032
        self.robots = [Robot(self) for _ in range(sum(ROBOT_COUNTS))] 
        self.stat_robot_working = []
        
    def run_simulation(self):
        while self.clock <= MAX_SIMULATION_TIME:
            for robot in self.robots:
                robot.step(self.dt)
            self.clock += self.dt
            self.stat_robot_working += [[robot.working for robot in self.robots]]

    def get_task_time(self):
        robots_working = sum([robot.working for robot in self.robots]*1)
        return 6.5*(AVG_TASK_TIME + random.random() * 10)/(TASK_COUNT - robots_working)
        

simulation = Simulation()
simulation.run_simulation()

plot_active_robots(simulation.stat_robot_working, simulation.dt)

average_robot_working = np.array(simulation.stat_robot_working)*1

NUM_SIMUL = 100
for a in range(NUM_SIMUL):
    simulation = Simulation()
    simulation.run_simulation()
    average_robot_working += np.array(simulation.stat_robot_working)*1


plot_active_robots(average_robot_working/(NUM_SIMUL+1.0), simulation.dt)