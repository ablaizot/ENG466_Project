from constants import *
from plotting import *
import random
import distribution

class Robot:
    def __init__(self, simulation):
        self.working = 0
        self.simulation = simulation
        self.work_to_do = 0
        self.offered_task_time = 9999999
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
                self.simulation.completed_tasks += 1
                self.simulation.generate_task()
                self.offered_task_time = self.simulation.get_task_time()
        
        else:
            time_value = self.calculate_time_value(self.offered_task_time, self.worked_time, self.simulation.clock)
            can_finish = MAX_WORK_TIME - self.worked_time >= self.offered_task_time
            if time_value <= 1 and can_finish:
                self.working = 1
                self.simulation.task_time_values += [self.offered_task_time]
                self.work_to_do = self.offered_task_time
        

class Simulation:
    def __init__(self):
        self.clock = 0
        self.dt = 0.032
        self.robots = [Robot(self) for _ in range(sum(ROBOT_COUNTS))] 
        self.stat_robot_working = []
        self.completed_tasks = 0
        self.task_time_values = []
        for i in range(10):
            self.generate_task()
        
    def run_simulation(self):
        while self.clock <= MAX_SIMULATION_TIME:
            for robot in self.robots:
                robot.step(self.dt)
            self.clock += self.dt
            self.stat_robot_working += [[robot.working for robot in self.robots]]

    def get_task_time(self):
        robots_working = sum([robot.working for robot in self.robots]*1)
        #time = 6.5*(AVG_TASK_TIME/2 + abs(random.gauss(AVG_TASK_TIME/2, 3))) / (TASK_COUNT - robots_working)
        task_quality = ((TASK_COUNT - robots_working) / TASK_COUNT) ** 0.5
        time = (AVG_TASK_TIME/2 + AVG_TASK_TIME*random.random()**0.5) * task_quality
        time = distribution.draw()
        return time
    
    def generate_task(self):
        for robot in self.robots:
            new_offer = self.get_task_time()
            if not robot.working:
                robot.offered_task_time = min(new_offer, robot.offered_task_time)
                #robot.offered_task_time = new_offer
                robot.step(0)
                if robot.working:
                    break
    
if __name__ == "__main__":
    simulation = Simulation()
    simulation.run_simulation()

    print(f"Simulation completed. Total completed tasks: {simulation.completed_tasks}")
    plot_active_robots(simulation.stat_robot_working, simulation.dt)

    average_robot_working = np.array(simulation.stat_robot_working)*1

    histogram(simulation.task_time_values)

    tasks = 0
    NUM_SIMUL = 1000
    import time
    start_time = time.time()
    for a in range(NUM_SIMUL):
        simulation = Simulation()
        simulation.run_simulation()
        tasks += simulation.completed_tasks
        average_robot_working += np.array(simulation.stat_robot_working)*1


    print(f"Run time is {(time.time()-start_time)/NUM_SIMUL}")

    print(f"Completed hundred simulations, average tasks: {tasks/NUM_SIMUL}")
    plot_active_robots(average_robot_working/(NUM_SIMUL+1.0), simulation.dt)