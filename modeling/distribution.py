from plotting import *
import numpy as np
import random

distribution = np.load("modeling/distribution_all.npy")
distribution /= sum(distribution)

import matplotlib.pyplot as plt
# plt.plot(distribution)
# plt.show()

def draw():
    chance = random.random() 
    cumulative = 0
    for i, value in enumerate(distribution):
        cumulative += value
        if cumulative >= chance:
            return i + random.random()-0.5
    #return 5+(1-random.random()**) * 50
