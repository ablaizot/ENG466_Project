import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import argparse


#### HOW TO USE? Run this file with 'python process.py + mode'


def compute_work_fraction(run):
    """
    Given one run, compute per-step work fraction:
    work_fraction[t] = % of the interval robot spent working
    """
    records = run["records"]

    work_fractions = []

    for i in range(1, len(records)):
        prev = records[i-1]
        curr = records[i]

        # Time step in milliseconds
        dt = curr["sim_ms"] - prev["sim_ms"]       # should be ~1024ms

        # Work increase
        dwork = curr["worked_ms"] - prev["worked_ms"]

        # Fraction of the step the robot was working
        frac = dwork / dt if dt > 0 else 0
        if frac < 0:
            frac = 0
        if frac > 1:
            frac = 1

        work_fractions.append(frac)

    return np.array(work_fractions)


def compute_run_statistics(run):
    """
    Given one run = { "records": [...], "termination": {...} },
    compute:
      - percentages of time in 4 states
      - average non-zero velocity
    """

    # Termination stats are absolute times in milliseconds
    t = run["termination"]
    moving = t["stat_moving_time"]
    avoiding = t["stat_avoiding_time"]
    task = t["stat_task_time"]
    idle = t["stat_idle_time"]

    total_time = moving + avoiding + task + idle

    pct_moving = moving / total_time
    pct_avoiding = avoiding / total_time
    pct_task = task / total_time
    pct_idle = idle / total_time

    # Average non-zero velocity
    speeds = [abs(rec["speed"]) for rec in run["records"] if abs(rec["speed"]) > 0.0001]
    if speeds:
        avg_speed = float(np.mean(speeds))
    else:
        avg_speed = 0.0

    return {
        "pct_moving": pct_moving,
        "pct_avoiding": pct_avoiding,
        "pct_task": pct_task,
        "pct_idle": pct_idle,
        "avg_speed": avg_speed
    }

def load_events_counts(path="events_handled.txt"):
    """Load number of events per simulation from file."""
    with open(path, "r") as f:
        return [int(line.strip()) for line in f if line.strip()]


def load_robot_file(filename, expected_runs):
    """Load and parse a robotX.txt file containing multiple runs."""

    runs = []
    current_run = {
        "records": [],   # list of dicts
        "termination": None
    }

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Check for termination line
            if line.startswith("TERMINATED"):
                # Format: TERMINATED a;b;c;d
                _, rest = line.split(" ", 1)
                a, b, c, d = map(int, rest.split(";"))

                current_run["termination"] = {
                    "stat_moving_time": a,
                    "stat_avoiding_time": b,
                    "stat_task_time": c,
                    "stat_idle_time": d
                }

                runs.append(current_run)

                # Start next run
                current_run = {"records": [], "termination": None}
                continue

            # Otherwise this is a normal data record
            # Format: worked/sim; speed
            # Example: "19200/19456; 0.014355"
            try:
                part1, speed_str = line.split(";")
                worked_str, sim_str = part1.split("/")

                current_run["records"].append({
                    "worked_ms": int(worked_str),
                    "sim_ms": int(sim_str),
                    "speed": float(speed_str)
                })
            except ValueError:
                print(f"WARNING: Could not parse line: {line}")

    # Sanity check
    if len(runs) != expected_runs:
        print(f"WARNING: In {filename}: expected {expected_runs} runs, but parsed {len(runs)}")

    return runs


# ------------ PARSE COMMAND LINE ARGUMENTS ------------


if len(sys.argv) != 2:
    print("Usage: python process.py <mode>")
    print("  mode: 'normal' or 'short' (for short-range epuck)")
    sys.exit(1)

mode = sys.argv[1].lower()

if mode not in ['normal', 'short', 'short_multi']:
    print("Error: mode must be 'normal', 'short_multi' or 'short'")
    sys.exit(1)

print(f"=== ANALYZING {mode.upper()} RANGE DATA ===\n")

# ------------ MAIN LOADING LOGIC ------------
events_file = "events_handled.txt"
if mode == 'short':
    events_file = "short_events_handled.txt"
elif mode == 'short_multi':
    events_file = "short_multi_events_handled.txt"
events_counts = load_events_counts(events_file)
expected_runs = len(events_counts)

all_robots = {}

for i in range(5):
    robot_file = f"robot{i}.txt"

    if mode == 'short':
        robot_file = f"short_robot{i}.txt"
    elif mode == 'short_multi':
        robot_file = f"short_multi_robot{i}.txt"
    if os.path.exists(robot_file):
        runs = load_robot_file(robot_file, expected_runs)
        all_robots[i] = runs
    else:
        print(f"WARNING: File {robot_file} not found.")


# Now `all_robots` is a dict:
# {
#   0: [ run0, run1, run2, ... ],
#   1: [ run0, run1, run2, ... ],
#   ...
#   4: [ ... ]
# }


# --------------------------------------------------------------
# COMPUTE STATS FOR ALL ROBOTS AND ALL RUNS
# --------------------------------------------------------------

all_stats = []   # flatten all robots + runs into one list

for robot_id, runs in all_robots.items():
    for run in runs:
        stats = compute_run_statistics(run)
        all_stats.append(stats)

# Convert to arrays for mean statistics
pct_moving_all = np.array([s["pct_moving"] for s in all_stats])
pct_avoiding_all = np.array([s["pct_avoiding"] for s in all_stats])
pct_task_all = np.array([s["pct_task"] for s in all_stats])
pct_idle_all = np.array([s["pct_idle"] for s in all_stats])
avg_speed_all = np.array([s["avg_speed"] for s in all_stats])

# --------------------------------------------------------------
# Print final aggregated statistics
# --------------------------------------------------------------

print("=== AVERAGED STATISTICS OVER ALL ROBOTS AND RUNS ===")
print(f"Percentage moving:   {pct_moving_all.mean() * 100:.2f}%")
print(f"Percentage avoiding: {pct_avoiding_all.mean() * 100:.2f}%")
print(f"Percentage task:     {pct_task_all.mean() * 100:.2f}%")
print(f"Percentage idle:     {pct_idle_all.mean() * 100:.2f}%")
print("")
print(f"Average velocity (non-zero entries only): {avg_speed_all.mean():.5f}")

events_arr = np.array(events_counts)

mean_tasks = events_arr.mean()
std_tasks  = events_arr.std(ddof=1)   # sample standard deviation
print()
print("=== TASK COMPLETION STATISTICS ===")
print(f"Mean tasks completed: {mean_tasks:.2f}")
print(f"Standard deviation:   {std_tasks:.2f}")


# -----------------------------------------------------------
# CHOOSE RUN
# -----------------------------------------------------------

parser = argparse.ArgumentParser(description="Plot number of robots working for a given run.")

# 1. ADD 'mode' as the first positional argument (must be a string)
parser.add_argument("mode", type=str, 
                    help="Mode of data to analyze ('normal', 'short', or 'short_multi')")

# 2. KEEP 'run_index' as the second positional argument (must be an integer)
parser.add_argument("run_index", type=int, 
                    help="Index of the run to plot (0-based)")

args = parser.parse_args()

# Extract the variables that the rest of the script expects
mode = args.mode.lower()
run_index = args.run_index

# You need to ensure the mode check is still performed, 
# as argparse won't validate the string content itself.
if mode not in ['normal', 'short', 'short_multi']:
    print("Error: mode must be 'normal', 'short_multi' or 'short'")
    sys.exit(1)

# Collect work fractions for all robots
robot_curves = []

for robot_id, runs in all_robots.items():
    if run_index < len(runs):
        wf = compute_work_fraction(runs[run_index])
        robot_curves.append(wf)
    else:
        print(f"Robot {robot_id} missing run {run_index}.")

# Ensure all curves have the same length (truncate to shortest)
min_len = min(len(c) for c in robot_curves)
robot_curves = [c[:min_len] for c in robot_curves]

# Sum: how many robots are working at each step
total_working = np.sum(robot_curves, axis=0)
tasks_completed = events_counts[run_index]

# -----------------------------------------------------------
# COMPUTE X-AXIS IN SECONDS
# -----------------------------------------------------------

# We'll use robot 0 as reference
ref_run = all_robots[0][run_index]
sim_times = [rec["sim_ms"] for rec in ref_run["records"][1:min_len+1]]  # skip first delta
x_seconds = np.array(sim_times) / 1000.0  # convert ms -> s

# -----------------------------------------------------------
# PLOT: robots working vs. real seconds with fixed y-axis and x-ticks
# -----------------------------------------------------------

individual_run_curves = {} 

# Iterate over all runs that were loaded (up to expected_runs)
for run_idx in range(expected_runs):
    run_curves = []
    
    # Collect data for all robots for the current run_idx
    for robot_id, runs in all_robots.items():
        if run_idx < len(runs):
            wf = compute_work_fraction(runs[run_idx])
            run_curves.append(wf)
        else:
            # Handle the case where a robot file might be missing a run
            print(f"Robot {robot_id} missing run {run_idx}.")

    # If any data was collected for this run:
    if run_curves:
        # Ensure all curves for THIS run have the same length (truncate to shortest)
        min_len = min(len(c) for c in run_curves)
        run_curves = [c[:min_len] for c in run_curves]

        # Calculate the sum (Total Working Robots) for THIS run
        total_working_for_run = np.sum(run_curves, axis=0)
        
        # Store the total working curve for this run
        individual_run_curves[run_idx] = total_working_for_run


# --- 2. ALIGN ALL RUNS FOR AVERAGING ---

# Find the minimum length across all collected runs
all_lengths = [len(curve) for curve in individual_run_curves.values()]
if not all_lengths:
    print("Error: No data found for any runs to average.")
    sys.exit(1)
    
min_total_len = min(all_lengths)

# Truncate all individual run curves to the minimum length
aligned_runs = []
for curve in individual_run_curves.values():
    aligned_runs.append(curve[:min_total_len])

# Stack the aligned runs vertically to create a 2D array
stacked_runs = np.stack(aligned_runs, axis=0)


# --- 3. CALCULATE THE AVERAGE CURVE ---

# The average total working robots at each time step
average_working = np.mean(stacked_runs, axis=0)


# --- 4. COMPUTE X-AXIS (using the first run as reference for time) ---

# We'll use robot 0 and run 0 as the time reference
ref_run = all_robots[0][0]
sim_times = [rec["sim_ms"] for rec in ref_run["records"][1:min_total_len+1]] 
x_seconds = np.array(sim_times) / 1000.0  # convert ms -> s


# -----------------------------------------------------------
# PLOT GENERATION
# -----------------------------------------------------------

plt.figure(figsize=(10, 5))

# Individual in grey
for run_curve in aligned_runs:
    plt.plot(x_seconds, run_curve, color='gray', alpha=0.6, linewidth=1, label='_nolegend_')

# Average Run 
plt.plot(x_seconds, average_working, color='C0', linewidth=3, label='Average')


plt.xlabel("Time (s)")
plt.ylabel("Average number of robots working")
plt.ylim(0, 5.5)  # fixed y-axis
plt.legend()

# x_axis
max_time = int(np.ceil(x_seconds[-1]))
plt.xticks(np.arange(0, max_time + 1, 20))


plt.grid(True, linestyle='--', alpha=0.4)
plt.tight_layout()
plt.show()



"""plt.figure(figsize=(10, 4))
plt.plot(x_seconds, total_working, linewidth=2)
plt.xlabel("Time (s)")
plt.ylabel("Average number of robots working")
plt.ylim(0, 5.5)  # fixed y-axis for consistency

# Set x-axis ticks every 20 seconds
max_time = int(np.ceil(x_seconds[-1]))
plt.xticks(np.arange(0, max_time + 1, 20))

plt.title(f"Run {run_index}: robots working over time — tasks completed = {tasks_completed}")
plt.grid(True)
plt.tight_layout()
plt.show()"""
