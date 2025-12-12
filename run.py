import os
import shutil
import subprocess
import time
import tempfile
import sys

# Path to Webots on Windows
WEBOTS_PATH = r"C:\Users\aymer\AppData\Local\Programs\Webots\msys64\mingw64\bin\webotsw.exe"

def main():
    if len(sys.argv) != 3:
        print("Usage: python run.py <number_of_runs> <mode>")
        print("  mode: 'normal', 'short' (for short-range epuck) or 'short_multi'")
        sys.exit(1)

    runs = int(sys.argv[1])
    mode = sys.argv[2].lower()
    
    if mode not in ['normal', 'short', 'short_multi']:
        print("Error: mode must be 'normal', 'short' or 'short_multi'")
        sys.exit(1)
    
    # Select world file based on mode
    if mode == 'short':
        world_file = "worlds/project4_thin_short.wbt"
    elif mode == 'short_multi':
        world_file = "worlds/project4_thin_short_multi.wbt"
    else:
        world_file = "worlds/project4_thin.wbt"

    # Use the system temp directory on Windows
    tmp = 'tmp'
    tmp_files = []
    if mode == 'short':
        tmp_files = [
            "short_events_handled.txt",
            "short_robot0.txt",
            "short_robot1.txt",
            "short_robot2.txt",
            "short_robot3.txt",
            "short_robot4.txt",
            "webots_done"
        ]
    elif mode == 'short_multi':
        tmp_files = [
            "short_multi_events_handled.txt",
            "short_multi_robot0.txt",
            "short_multi_robot1.txt",
            "short_multi_robot2.txt",
            "short_multi_robot3.txt",
            "short_multi_robot4.txt",
            "webots_done"
        ]
    else:
        tmp_files = [
            "events_handled.txt",
            "robot0.txt",
            "robot1.txt",
            "robot2.txt",
            "robot3.txt",
            "robot4.txt",
            "webots_done",
            "task_completion_time0.txt",
            "task_completion_time1.txt",
            "task_completion_time2.txt",
            "task_completion_time3.txt",
            "task_completion_time4.txt"

        ]

    print("All temporary files will be deleted (if they exist)")
    for f in tmp_files:
        path = os.path.join(tmp, f)
        if os.path.exists(path):
            os.remove(path)

    print(f"Launching Webots {runs} times in batch mode ({mode} mode)")

    for i in range(1, runs + 1):
        print(i)

        # Launch Webots
        process = subprocess.Popen([
            WEBOTS_PATH,
            "--minimize",
            "--mode=fast",
            "--no-rendering",
            world_file
        ])

        done_file = os.path.join(tmp, "webots_done")

        # Wait until /tmp/webots_done appears
        while not os.path.exists(done_file):
            time.sleep(1)

        # Kill Webots
        subprocess.call(
            ["taskkill", "/IM", "webotsw.exe", "/F"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # Remove webots_done
        if os.path.exists(done_file):
            os.remove(done_file)

    # Create data directory if needed
    data_dir = "data"
    if not os.path.isdir(data_dir):
        os.mkdir(data_dir)

    print("Copying results to data/")

    for fname in tmp_files[:-1]:  # exclude webots_done
        src = os.path.join(tmp, fname)
        if os.path.exists(src):
            shutil.copy(src, data_dir)

    print("Done.")

if __name__ == "__main__":
    main()
