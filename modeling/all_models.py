import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import micro_simple
import micro
import macro_travel as macro
from plotting import *



macro_simulator = macro.Simulation()
macro_simulator.run_simulation()


micro_simulator = micro.Simulation()
micro_simulator.run_simulation()
mili_simulator = micro_simple.Simulation()
mili_simulator.run_simulation()

micro_robot_working = np.array(micro_simulator.statistics.robot_working)*1
mili_robot_working = np.array(mili_simulator.stat_robot_working)*1

tasks = 0
NUM_SIMUL = 100
for a in range(NUM_SIMUL):
    micro_simulator = micro.Simulation()
    micro_simulator.run_simulation()
    mili_simulator = micro_simple.Simulation()
    mili_simulator.run_simulation()

    micro_robot_working += np.array(micro_simulator.statistics.robot_working)*1
    mili_robot_working += np.array(mili_simulator.stat_robot_working)*1

plot_all(micro_robot_working/(NUM_SIMUL+1.0), mili_robot_working/(NUM_SIMUL+1.0), macro_simulator.working, 0.032)