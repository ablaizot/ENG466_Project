import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import glob

def load_task_completion_times(mode):
    """
    Load task completion times from task_completion_timeX.txt files.
    If mode is 'short', looks for short_task_completion_timeX.txt (if they exist)
    or just assumes the standard naming if that's how the simulation outputs them.
    Based on the file list, it seems they are named task_completion_timeX.txt.
    """
    all_times = []
    
    # Determine file pattern based on mode if necessary. 
    # Assuming the user wants to plot from the files present in the directory.
    # If there are specific 'short_' versions, we should target them.
    # Based on previous `ls`, we saw `task_completion_timeX.txt`.
    
    pattern = "task_completion_time*.txt"
    if mode == 'short':
         # Check if short versions exist, otherwise fallback or assume shared
         if glob.glob("short_task_completion_time*.txt"):
             pattern = "short_task_completion_time*.txt"
    
    files = glob.glob(pattern)
    
    if not files:
        print(f"No task completion time files found matching {pattern}")
        return []

    for filename in files:
        try:
            with open(filename, 'r') as f:
                times = [float(line.strip()) for line in f if line.strip()]
                # Convert ms to seconds if the values are large (e.g. > 1000)
                # The sample showed 8704, which is likely ms.
                times_s = [t / 1000.0 for t in times]
                all_times.extend(times_s)
        except Exception as e:
            print(f"Error reading {filename}: {e}")
            
    return all_times

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

# Removed the conflicting load_task_completion_times definition here
# as it was redefining the one we added at the top of the file.

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
events_file = "events_handled.txt"
if mode == 'short':
    events_file = "short_events_handled.txt"
events_counts = load_events_counts(events_file)
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
# PLOT: Histogram of task completion times from files
# -----------------------------------------------------------

task_times_from_files = load_task_completion_times(mode)

if task_times_from_files:
    plt.figure(figsize=(10, 6))
    plt.hist(task_times_from_files, bins=30, edgecolor='black', alpha=0.7, color='skyblue')
    plt.xlabel("Task Completion Time (s)")
    plt.ylabel("Frequency")
    plt.title(f"Histogram of Task Completion Times (from files) - {mode} mode")
    
    # Add mean and std dev to plot
    mean_time = np.mean(task_times_from_files)
    std_time = np.std(task_times_from_files)
    plt.axvline(mean_time, color='red', linestyle='dashed', linewidth=1, label=f'Mean: {mean_time:.2f}s')
    plt.legend()
    
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
else:
    print("No task completion time data found in files.")

# -----------------------------------------------------------
# PLOT AVERAGE CURVE ACROSS ALL RUNS
# -----------------------------------------------------------

all_run_curves = []

for r in range(expected_runs):
    # Collect work fractions for all robots in run r
    robot_curves_this_run = []
    valid_run = True
    for robot_id, runs in all_robots.items():
        if r < len(runs):
            wf = compute_work_fraction(runs[r])
            robot_curves_this_run.append(wf)
        else:
            valid_run = False
            break
    
    if valid_run and robot_curves_this_run:
        # Truncate to shortest robot in this run
        if len(robot_curves_this_run) > 0:
            min_len = min(len(c) for c in robot_curves_this_run)
            robot_curves_this_run = [c[:min_len] for c in robot_curves_this_run]
            
            # Sum robots for this run
            total_working_run = np.sum(robot_curves_this_run, axis=0)
            all_run_curves.append(total_working_run)

if all_run_curves:
    # Truncate to shortest run to average them
    final_min_len = min(len(c) for c in all_run_curves)
    all_run_curves_truncated = [c[:final_min_len] for c in all_run_curves]
    
    # Average
    avg_curve = np.mean(all_run_curves_truncated, axis=0)
    
    # Plot
    plt.figure(figsize=(10, 4))
    
    # Plot individual runs faintly
    for c in all_run_curves:
        plt.plot(c, color='gray', alpha=0.15)
        
    plt.plot(avg_curve, color='blue', linewidth=2, label='Average')
    
    plt.xlabel("Time step (≈1.024 s each)")
    plt.ylabel("Number of robots working")
    plt.ylim(0, 5.5)  # fixed y-axis range for consistency
    plt.title(f"Average robots working over {len(all_run_curves)} runs ({mode} mode)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
else:
    print("No valid runs found to plot.")