import os
import shutil
import subprocess
import time
import tempfile
import sys

# Path to Webots on Windows
WEBOTS_PATH = r"C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe"

def main():
    if len(sys.argv) != 2:
        print("Usage: python run_webots_batch.py <number_of_runs>")
        sys.exit(1)

    runs = int(sys.argv[1])

    # Use the system temp directory on Windows
    tmp = 'tmp'

    tmp_files = [
        "events_handled.txt",
        "robot0.txt",
        "robot1.txt",
        "robot2.txt",
        "robot3.txt",
        "robot4.txt",
        "webots_done"
    ]

    print("All temporary files will be deleted (if they exist)")
    for f in tmp_files:
        path = os.path.join(tmp, f)
        if os.path.exists(path):
            os.remove(path)

    print(f"Launching Webots {runs} times in batch mode")

    for i in range(1, runs + 1):
        print(i)

        # Launch Webots
        process = subprocess.Popen([
            WEBOTS_PATH,
            "--minimize",
            "--mode=fast",
            "--no-rendering",
            "worlds/project4_thin.wbt"
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
