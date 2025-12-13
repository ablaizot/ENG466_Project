import numpy as np
import matplotlib.pyplot as plt

def plot_active_robots(working_matrix, dt):
    """
    working_matrix : 2D numpy array (n_robots, n_timesteps), bool
    dt             : sampling period in seconds (float)
    """

    # Number of robots working at each 
    working_matrix = np.array(working_matrix)
    active_counts = np.sum(working_matrix, axis=1)

    # Create timestamps in seconds
    timesteps = working_matrix.shape[0]
    t = np.arange(timesteps) * dt

    # Plot
    plt.figure(figsize=(10, 4))
    plt.plot(t, active_counts)
    plt.xlabel("Time [s]")
    plt.ylabel("Number of robots working")
    plt.title("Active robots over time")

    # Set x-axis ticks every 20 seconds
    max_t = t[-1]
    plt.xticks(np.arange(0, max_t + 1, 20))

    plt.tight_layout()
    plt.show()

def plot_macro(working, dt):
    # Create timestamps in seconds
    timesteps = len(working)
    t = np.arange(timesteps) * dt

    # Plot
    plt.figure(figsize=(10, 4))
    plt.plot(t, working)
    plt.xlabel("Time [s]")
    plt.ylabel("Number of robots working")
    plt.title("Active robots over time")

    # Set x-axis ticks every 20 seconds
    max_t = t[-1]
    plt.xticks(np.arange(0, max_t + 1, 20))

    plt.tight_layout()
    plt.show()


def histogram(data):
    plt.hist(data, bins=61)      # number of bins can be adjusted
    plt.xlabel("Value")
    plt.ylabel("Frequency")
    plt.title("Histogram of Float Values")
    plt.show()

def plot_all(micro_matrix, mili_matrix, macro, dt):
    micro_active = np.sum(np.array(micro_matrix), axis=1)
    mili_active = np.sum(np.array(mili_matrix), axis=1)
    t = np.arange(len(macro)) * dt

    # Plot
    plt.figure(figsize=(10, 4))
    plt.plot(t, micro_active, label="Micro")
    plt.plot(t, mili_active, label="Mili")
    plt.plot(t, macro, label="Macro")
    plt.xlabel("Time [s]")
    plt.ylabel("Number of robots working")
    plt.title("Active robots over time")
    plt.legend()

    # Set x-axis ticks every 20 seconds
    max_t = t[-1]
    plt.xticks(np.arange(0, max_t + 1, 20))

    plt.tight_layout()
    plt.show()