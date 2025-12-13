import os
import shutil
import subprocess
import time
import sys
import argparse
import datetime

# Path to Webots on Windows
WEBOTS_PATH = r"C:\Users\aymer\AppData\Local\Programs\Webots\msys64\mingw64\bin\webotsw.exe"
WEBOTS_HOME = r"C:\Users\aymer\AppData\Local\Programs\Webots"
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.join(PROJECT_ROOT, "controllers")
#export PATH=$PATH:/C/Program\ Files/Webots/msys64/mingw64/bin:/C/Program\ Files/Webots/msys64/mingw64/bin

# Configuration
BASE_DATA_DIR = "data"
TIMESTAMP = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
DATA_DIR = os.path.join(BASE_DATA_DIR, TIMESTAMP)
TMP_DIR = "tmp"

ALL_SCENARIOS = [
    {
        "name": "centralized_single",
        "world": "worlds/project4_centralized_single.wbt",
        "controller": "epuck_crown_single",
        "output_files": ["events_handled.txt"] + [f"robot{i}.txt" for i in range(5)],
        "prefix": "centralized_single_"
    },
    {
        "name": "centralized_multi",
        "world": "worlds/project4_centralized_multi.wbt",
        "controller": "epuck_crown",
        "output_files": ["events_handled.txt"] + [f"robot{i}.txt" for i in range(5)],
        "prefix": "centralized_multi_"
    },
    {
        "name": "short_single",
        "world": "worlds/project4_thin_short_single.wbt",
        "controller": "short_range_epuck2",
        "output_files": ["short_events_handled.txt"] + [f"short_robot{i}.txt" for i in range(5)],
        "prefix": "short_single_"
    },
    {
        "name": "short_multi",
        "world": "worlds/project4_thin_short_multi.wbt",
        "controller": "short_range_epuck_multi",
        "output_files": ["short_multi_events_handled.txt"] + [f"short_multi_robot{i}.txt" for i in range(5)],
        "prefix": "short_multi_"
    }
]

def compile_controller(controller_name):
    """
    Compiles the controller using make.
    """
    controller_path = os.path.join(CONTROLLERS_DIR, controller_name)
    
    # Set WEBOTS_HOME environment variable
    env = os.environ.copy()
    env["WEBOTS_HOME"] = WEBOTS_HOME
    
    # Add msys paths to Windows PATH to ensure DLLs are found
    msys_bin = os.path.join(WEBOTS_HOME, "msys64", "usr", "bin")
    mingw_bin = os.path.join(WEBOTS_HOME, "msys64", "mingw64", "bin")
    env["PATH"] = f"{mingw_bin};{msys_bin};{env['PATH']}"
    
    print(f"Compiling {controller_name}...")
    try:
        # Run make clean
        subprocess.check_call(["make", "clean"], cwd=controller_path, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Run make
        subprocess.check_call(["make"], cwd=controller_path, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        print(f"Error compiling {controller_name}: {e}")
        return False
    return True

def run_scenario(scenario, runs):
    print(f"=== Running Scenario: {scenario['name']} ===")
    
    # Compile controller
    if "controller" in scenario:
        if not compile_controller(scenario["controller"]):
            print("Compilation failed! Aborting scenario.")
            return

    # Clean tmp files
    for f in scenario["output_files"] + ["webots_done"]:
        path = os.path.join(TMP_DIR, f)
        if os.path.exists(path):
            os.remove(path)

    for i in range(1, runs + 1):
        print(f"  Run {i}/{runs}")
        
        # Launch Webots
        process = subprocess.Popen([
            WEBOTS_PATH,
            "--minimize",
            "--mode=fast",
            "--no-rendering",
            scenario["world"]
        ])

        done_file = os.path.join(TMP_DIR, "webots_done")

        # Wait until /tmp/webots_done appears
        while not os.path.exists(done_file):
            time.sleep(1)

        # Kill Webots
        subprocess.call(
            ["taskkill", "/IM", "webotsw.exe", "/F"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # Remove webots_done for next run
        if os.path.exists(done_file):
            os.remove(done_file)

    # Move data to data dir with prefix
    if not os.path.isdir(DATA_DIR):
        os.makedirs(DATA_DIR)
        
    print(f"  Saving data to {DATA_DIR}...")
    for fname in scenario["output_files"]:
        src = os.path.join(TMP_DIR, fname)
        if os.path.exists(src):
            # Determine new name
            # If fname starts with prefix already (unlikely given the config), handle it?
            # Actually, the files in tmp have fixed names. We want to rename them.
            # But wait, if we run multiple runs, the controllers APPEND to the files ("a" mode).
            # So we just need to move the accumulated file after all runs.
            
            # We want to rename e.g. "robot0.txt" to "centralized_single_robot0.txt"
            # But wait, "short_robot0.txt" to "short_single_short_robot0.txt"? 
            # Maybe just "short_single_robot0.txt" is cleaner.
            
            # Let's just prepend the prefix to the original filename for simplicity/uniqueness
            dst_name = scenario["prefix"] + fname
            dst = os.path.join(DATA_DIR, dst_name)
            
            # If destination exists, remove it first (fresh start for this batch)
            if os.path.exists(dst):
                os.remove(dst)
                
            shutil.move(src, dst)
        else:
            print(f"  WARNING: Output file {fname} not found in {TMP_DIR}")

def main():
    start_time = time.time()
    parser = argparse.ArgumentParser(description="Run Webots scenarios.")
    parser.add_argument("--runs", type=int, default=5, help="Number of runs per scenario (default: 5)")
    parser.add_argument("--scenario", type=str, default="all", 
                        choices=["all", "centralized_single", "centralized_multi", "short_single", "short_multi"],
                        help="Which scenario to run (default: all)")
    
    args = parser.parse_args()

    scenarios_to_run = []
    if args.scenario == "all":
        scenarios_to_run = ALL_SCENARIOS
    else:
        scenarios_to_run = [s for s in ALL_SCENARIOS if s["name"] == args.scenario]

    if not scenarios_to_run:
        print(f"No scenario found matching '{args.scenario}'")
        return

    print(f"Starting batch execution: {len(scenarios_to_run)} scenario(s), {args.runs} runs each.")

    for scenario in scenarios_to_run:
        run_scenario(scenario, args.runs)
    print("\nAll scenarios completed.")
    end_time = time.time()
    duration = end_time - start_time
    print(f"Total execution time: {duration:.2f} seconds ({duration/60:.2f} minutes)")


    print("Running data processing...")
    subprocess.run([sys.executable, "process.py"], cwd=BASE_DATA_DIR)



if __name__ == "__main__":
    main()
