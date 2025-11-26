import os
import sys
import numpy as np
import matplotlib.pyplot as plt

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

if mode not in ['normal', 'short']:
    print("Error: mode must be 'normal' or 'short'")
    sys.exit(1)

print(f"=== ANALYZING {mode.upper()} RANGE DATA ===\n")

# ------------ MAIN LOADING LOGIC ------------

events_counts = load_events_counts("events_handled.txt")
expected_runs = len(events_counts)

all_robots = {}

for i in range(5):
    robot_file = f"robot{i}.txt"

    if mode == 'short':
        robot_file = f"short_robot{i}.txt"
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

run_index = 1  # <--- CHANGE THIS to visualize another run

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
# PLOT
# -----------------------------------------------------------

plt.figure(figsize=(10, 4))
plt.plot(total_working)
plt.xlabel("Time step (≈1.024 s each)")
plt.ylabel("Average number of robots working")
plt.ylim(0, 5.5)  # fixed y-axis range for consistency
plt.title(f"Run {run_index} ({mode} mode): robots working over time — tasks completed = {tasks_completed}")
plt.grid(True)
plt.tight_layout()
plt.show()