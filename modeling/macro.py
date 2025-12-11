from constants import *
from plotting import *

class Simulation:
    def __init__(self):
        self.idle = [sum(ROBOT_COUNTS)]
        self.working = [0]
        self.started_working = []
        self.clock = 0
        self.dt = 0.032
        self.max_work_time = 120*sum(ROBOT_COUNTS)
        self.worked_time = 0

    def step(self):
        p_work = 0.01 * (max(0, self.max_work_time - self.worked_time)/self.max_work_time)**0.2
        begin_work = p_work * self.idle[-1]
        self.started_working += [begin_work]

        p_stop = 0.003
        min_work_time = int(10 / self.dt)
        available_to_stop = 0
        index = 0
        while min_work_time + index < len(self.started_working):
            available_to_stop += self.started_working[-1-min_work_time-index] * (1-p_stop)**index
            index += 1
        stop_work = available_to_stop * p_stop

        self.idle += [self.idle[-1] - begin_work + stop_work]
        self.working += [self.working[-1] + begin_work - stop_work]

        self.worked_time += self.working[-1] * self.dt

    def run_simulation(self):
        while self.clock <= MAX_SIMULATION_TIME:
            self.step()
            self.clock += self.dt


simulation = Simulation()
simulation.run_simulation()

#print(simulation.working)

plot_macro(simulation.working, simulation.dt)