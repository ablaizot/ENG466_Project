from constants import *
from plotting import *

class Simulation:
    def __init__(self):
        self.N = sum(ROBOT_COUNTS)
        self.idle = [1.5]
        self.working = [3.5]
        self.started_working = [3.5]
        self.clock = 0
        self.dt = 0.032
        self.work_fallof = 11.5 # set this, so the total worktime is 120 in the end
        self.max_work_time = (MAX_WORK_TIME-self.work_fallof)*self.N
        self.worked_time = 0

    def step(self):
        task_quality = self.dt/12#0.01
        #end_timer = (max(0, self.max_work_time - self.worked_time)/self.max_work_time)**0.2
        end_timer = (self.max_work_time >= self.worked_time)*1

        simulation_time_remain = (MAX_SIMULATION_TIME - self.clock) * self.N
        work_time_remain = (self.N*MAX_WORK_TIME - self.worked_time)
        wait_quality = (simulation_time_remain - work_time_remain) / (MAX_SIMULATION_TIME - MAX_WORK_TIME) / self.N

        p_work = task_quality / wait_quality * end_timer
        #print(self.clock, p_work)
        begin_work = p_work * self.idle[-1]
        self.started_working += [begin_work]

        p_stop = self.dt/16
        # available_to_stop = 0
        # index = int(10/self.dt)
        # max_work_time = int(40/self.dt)
        # while index < len(self.started_working) and index <= max_work_time:
        #     available_to_stop += self.started_working[-1-index] * (1-p_stop)**(index)
        #     index += 1
        #print(available_to_stop/self.working[-1])
        stop_work = self.working[-1] * p_stop

        falloff_detect = int(20/ self.dt)
        falloff = 0
        for a in range(1, falloff_detect):
            falloff += (p_stop ** a) * (1-p_stop) ** (falloff_detect-a)

        begin_work += 0.5 * stop_work * end_timer

        self.idle += [self.idle[-1] - begin_work + stop_work]
        self.working += [self.working[-1] + begin_work - stop_work]
        self.worked_time += self.working[-1] * self.dt

    def run_simulation(self):
        while self.clock <= MAX_SIMULATION_TIME-self.dt:
            self.step()
            self.clock += self.dt
        print(f"Worked in total {self.worked_time/5}")


if __name__ == "__main__":
    simulation = Simulation()
    simulation.run_simulation()

    #print(simulation.working)

    plot_macro(simulation.working, simulation.dt)