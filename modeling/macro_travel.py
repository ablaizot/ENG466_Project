from constants import *
from plotting import *

class Simulation:
    def __init__(self):
        self.start_timer = 30
        self.N = sum(ROBOT_COUNTS)
        self.idle = [self.N]
        self.travelling = [0]
        self.doing = [0]
        self.started_doing = [0]
        self.clock = -self.start_timer # some time to get it stable
        self.dt = 0.032
        self.work_fallof = 14.5 # set this, so the total worktime is 120 in the end
        self.max_work_time = (MAX_WORK_TIME-self.work_fallof)*self.N
        self.worked_time = 0
        self.working = []

        self.p_stop = self.dt/16.2 # This is the mean time of travel
        self.p_work = self.dt/11.8 # Mean time of waiting
        self.work_time = int(9/self.dt)

    def step(self):
        end_timer = (self.max_work_time >= self.worked_time)*1

        simulation_time_remain = (MAX_SIMULATION_TIME - self.clock) * self.N
        work_time_remain = (self.N*MAX_WORK_TIME - self.worked_time)
        wait_quality = (simulation_time_remain - work_time_remain) / (MAX_SIMULATION_TIME - MAX_WORK_TIME) / self.N

        p_get_task = self.p_work * end_timer / wait_quality
        p_finish_travelling = self.p_stop

        start_travelling = p_get_task * self.idle[-1]
        start_doing = p_finish_travelling * self.travelling[-1]
        stop_working = self.started_doing[-1-self.work_time] if self.work_time < len(self.started_doing) else 0

        start_travelling += 0.5 * stop_working * end_timer

        self.started_doing += [start_doing]
        self.idle += [self.idle[-1] - start_travelling + stop_working]
        self.doing += [self.doing[-1] + start_doing - stop_working]
        self.travelling += [self.travelling[-1] + start_travelling - start_doing]

        if self.clock >= -self.dt:
            self.working += [self.doing[-1] + self.travelling[-1]]
            self.worked_time += (self.doing[-1]+self.travelling[-1]) * self.dt

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