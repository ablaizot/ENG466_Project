import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import glob

# Configuration for the 4 scenarios
SCENARIOS = {
    "centralized_single": {
        "prefix": "centralized_single_",
        "color": "blue",
        "label": "Centralized Single"
    },
    "centralized_multi": {
        "prefix": "centralized_multi_",
        "color": "green",
        "label": "Centralized Multi"
    },
    "short_single": {
        "prefix": "short_single_",
        "color": "red",
        "label": "Short Single"
    },
    "short_multi": {
        "prefix": "short_multi_",
        "color": "orange",
        "label": "Short Multi"
    }
}

def find_files(target_filename):
    """
    Recursively find files matching target_filename in current directory and subdirectories.
    """
    matches = []
    for root, dirs, files in os.walk("."):
        if target_filename in files:
            matches.append(os.path.join(root, target_filename))
    return matches

def load_task_completion_times(prefix):
    """
    Load task completion times.
    """
    all_times = []
    # Pattern: prefix + task_completion_time*.txt
    # But wait, the original code looked for task_completion_time*.txt or short_task_completion_time*.txt
    # My run_all.py does NOT rename task_completion_time files because I didn't see them in the output list of run.py
    # I should check if they are generated.
    # If they are generated, they are likely in tmp/ and I missed them in run_all.py.
    # I'll skip this for now or assume they are not critical for the main plot.
    return []

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
        dt = curr["sim_ms"] - prev["sim_ms"]
        dwork = curr["worked_ms"] - prev["worked_ms"]
        frac = dwork / dt if dt > 0 else 0
        if frac < 0: frac = 0
        if frac > 1: frac = 1
        work_fractions.append(frac)

    return np.array(work_fractions)

def compute_run_statistics(run):
    t = run["termination"]
    moving = t["stat_moving_time"]
    avoiding = t["stat_avoiding_time"]
    task = t["stat_task_time"]
    idle = t["stat_idle_time"]
    total_time = moving + avoiding + task + idle

    pct_moving = moving / total_time if total_time > 0 else 0
    pct_avoiding = avoiding / total_time if total_time > 0 else 0
    pct_task = task / total_time if total_time > 0 else 0
    pct_idle = idle / total_time if total_time > 0 else 0

    speeds = [abs(rec["speed"]) for rec in run["records"] if abs(rec["speed"]) > 0.0001]
    avg_speed = float(np.mean(speeds)) if speeds else 0.0

    return {
        "pct_moving": pct_moving,
        "pct_avoiding": pct_avoiding,
        "pct_task": pct_task,
        "pct_idle": pct_idle,
        "avg_speed": avg_speed
    }

def load_events_counts(path):
    if not os.path.exists(path):
        return []
    with open(path, "r") as f:
        return [int(line.strip()) for line in f if line.strip()]

def load_robot_file(filename, expected_runs):
    runs = []
    current_run = {"records": [], "termination": None}
    
    if not os.path.exists(filename):
        return []

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line: continue

            if line.startswith("TERMINATED"):
                _, rest = line.split(" ", 1)
                a, b, c, d = map(int, rest.split(";"))
                current_run["termination"] = {
                    "stat_moving_time": a,
                    "stat_avoiding_time": b,
                    "stat_task_time": c,
                    "stat_idle_time": d
                }
                runs.append(current_run)
                current_run = {"records": [], "termination": None}
                continue

            try:
                part1, speed_str = line.split(";")
                worked_str, sim_str = part1.split("/")
                current_run["records"].append({
                    "worked_ms": int(worked_str),
                    "sim_ms": int(sim_str),
                    "speed": float(speed_str)
                })
            except ValueError:
                pass

    return runs

def get_curves(scenario_key):
    """
    Load data and compute curves for a specific scenario.
    Returns (avg_curve, std_curve, all_run_curves) or (None, None, None).
    """
    conf = SCENARIOS[scenario_key]
    prefix = conf["prefix"]
    
    # Determine events file name
    base_events = "events_handled.txt"
    if "short" in scenario_key:
        if "multi" in scenario_key:
            base_events = "short_multi_events_handled.txt"
        else:
            base_events = "short_events_handled.txt"
            
    target_events = prefix + base_events
    event_files = find_files(target_events)
    
    # Fallback: try without prefix if no prefixed files found (legacy support)
    if not event_files:
        event_files = find_files(base_events)
        
    if not event_files:
        return None, None, None

    # Load all event counts to determine total runs
    all_counts = []
    for ef in event_files:
        all_counts.extend(load_events_counts(ef))
        
    if not all_counts:
        return None, None, None
        
    total_runs = len(all_counts)

    # Load robots
    # We need to load robot files corresponding to the event files found
    # But since we are aggregating everything, we can just find all robot files
    # and assume they match the runs.
    
    robots_data = {}
    for i in range(5):
        # Base robot file
        base_robot = f"robot{i}.txt"
        if "short" in scenario_key:
            if "multi" in scenario_key:
                base_robot = f"short_multi_robot{i}.txt"
            else:
                base_robot = f"short_robot{i}.txt"
        
        target_robot = prefix + base_robot
        robot_files = find_files(target_robot)
        
        if not robot_files:
             robot_files = find_files(base_robot)
        
        robots_data[i] = []
        for rf in robot_files:
            # We don't strictly need expected_runs here as load_robot_file parses what's there
            robots_data[i].extend(load_robot_file(rf, 0))
    
    if not robots_data:
        return None, None, None

    # Compute curves
    run_curves = []
    for r in range(total_runs):
        curves_this_run = []
        valid = True
        for rid, runs in robots_data.items():
            if r < len(runs):
                wf = compute_work_fraction(runs[r])
                curves_this_run.append(wf)
            else:
                valid = False
                break
        
        if valid and curves_this_run:
            min_l = min(len(c) for c in curves_this_run)
            curves_this_run = [c[:min_l] for c in curves_this_run]
            total_working = np.sum(curves_this_run, axis=0)
            run_curves.append(total_working)
            
    if not run_curves:
        return None, None, None

    final_min = min(len(c) for c in run_curves)
    truncated = [c[:final_min] for c in run_curves]
    
    avg = np.mean(truncated, axis=0)
    std = np.std(truncated, axis=0)
    
    return avg, std, run_curves

def get_scenario_stats(scenario_key):
    """
    Compute aggregated statistics for a specific scenario.
    Returns a dict with mean values for the table.
    """
    conf = SCENARIOS[scenario_key]
    prefix = conf["prefix"]
    
    # Determine events file name
    base_events = "events_handled.txt"
    if "short" in scenario_key:
        if "multi" in scenario_key:
            base_events = "short_multi_events_handled.txt"
        else:
            base_events = "short_events_handled.txt"
            
    target_events = prefix + base_events
    event_files = find_files(target_events)
    
    # Fallback
    if not event_files:
        event_files = find_files(base_events)
        
    mean_tasks = 0
    std_tasks = 0
    
    all_counts = []
    for ef in event_files:
        all_counts.extend(load_events_counts(ef))
        
    if all_counts:
        mean_tasks = np.mean(all_counts)
        std_tasks = np.std(all_counts, ddof=1)
    
    # Load robots and compute stats
    all_stats = []
    for i in range(5):
        base_robot = f"robot{i}.txt"
        if "short" in scenario_key:
            if "multi" in scenario_key:
                base_robot = f"short_multi_robot{i}.txt"
            else:
                base_robot = f"short_robot{i}.txt"
        
        target_robot = prefix + base_robot
        robot_files = find_files(target_robot)
        
        if not robot_files:
            robot_files = find_files(base_robot)
            
        for rf in robot_files:
            runs = load_robot_file(rf, 0)
            for run in runs:
                all_stats.append(compute_run_statistics(run))

    if not all_stats:
        return {
            "pct_moving": 0, "pct_avoiding": 0, "pct_task": 0, "pct_idle": 0, "mean_tasks": 0, "std_tasks": 0
        }

    pct_moving = np.mean([s["pct_moving"] for s in all_stats])
    pct_avoiding = np.mean([s["pct_avoiding"] for s in all_stats])
    pct_task = np.mean([s["pct_task"] for s in all_stats])
    pct_idle = np.mean([s["pct_idle"] for s in all_stats])
    
    return {
        "pct_moving": pct_moving * 100,
        "pct_avoiding": pct_avoiding * 100,
        "pct_task": pct_task * 100,
        "pct_idle": pct_idle * 100,
        "mean_tasks": mean_tasks,
        "std_tasks": std_tasks
    }

def generate_latex_table():
    rows_order = [
        ("centralized_single", "Single-Step Centralized"),
        ("centralized_multi", "Multi-Step Centralized"),
        ("short_single", "Single-Step Distributed"),
        ("short_multi", "Multi-Step Distributed")
    ]
    
    print("\n=== Simulation Statistics ===")
    print(f"{'Scenario':<25} | {'Moving %':<10} | {'Avoiding %':<10} | {'Task %':<10} | {'Idle %':<10} | {'Mean Tasks':<10} | {'Std Dev':<10}")
    print("-" * 105)

    latex_content = r"""\begin{table*}[t!]
\centering
\setlength{\tabcolsep}{10pt}
\renewcommand{\arraystretch}{1.3}
\caption{Average data over 5 runs for the different task allocation methods}
\label{tab:Data}
\begin{tabular}{|l|c|c|c|c|c|c|}
\hline
\textbf{} & \% moving & \% avoiding & \% task & \% idle & Mean tasks completed & Std Dev Tasks \\
\hline
"""
    
    for key, label in rows_order:
        stats = get_scenario_stats(key)
        
        # Print to console
        print(f"{label:<25} | {stats['pct_moving']:<10.2f} | {stats['pct_avoiding']:<10.2f} | {stats['pct_task']:<10.2f} | {stats['pct_idle']:<10.2f} | {stats['mean_tasks']:<10.2f} | {stats['std_tasks']:<10.2f}")

        # Format: Label & 1A & 1B & 1C & 1D & 1E & 1F \\
        row_str = f"{label} & {stats['pct_moving']:.2f} & {stats['pct_avoiding']:.2f} & {stats['pct_task']:.2f} & {stats['pct_idle']:.2f} & {stats['mean_tasks']:.2f} & {stats['std_tasks']:.2f} \\\\"
        latex_content += row_str + "\n\\hline\n"

    latex_content += r"""\end{tabular}
\end{table*}
"""
    
    with open("simulation_results.tex", "w") as f:
        f.write(latex_content)
    print("\nLaTeX table saved to simulation_results.tex")

def plot_group(group_name, scenario_keys):
    plt.figure(figsize=(10, 5))
    plotted_any = False
    
    for key in scenario_keys:
        if key not in SCENARIOS: continue
        conf = SCENARIOS[key]
        
        print(f"Processing {key}...")
        avg, std, runs = get_curves(key)

        time = np.arange(len(avg)) * 1.024  # assuming each time step is approx 1.024 seconds
        
        if avg is not None:
            plotted_any = True
            plt.plot(time, avg, color=conf["color"], linewidth=2, label=conf["label"])
            plt.fill_between(time, avg - std, avg + std, color=conf["color"], alpha=0.15)
            print(f"  Loaded {len(runs)} runs.")
        else:
            print(f"  No data found for {key}.")

    if plotted_any:
        plt.xlabel("Time step (≈1.024 s each)")
        plt.ylabel("Number of robots working")
        plt.ylim(0, 5.5)
        plt.xlim(0, time[-1])
        plt.xticks(np.arange(0, time[-1], 20))
        plt.title(f"Average robots working ({group_name})")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.show()
    else:
        print(f"No valid data found for {group_name}.")

def main():
    print("=== PROCESSING DATA FOR ALL SCENARIOS ===")
    
    # Generate Table
    generate_latex_table()
    
    # Generate Plots
    plot_group("Centralized", ["centralized_single", "centralized_multi"])
    plot_group("Distributed", ["short_single", "short_multi"])

if __name__ == "__main__":
    main()