import glob
import os
import sys
import argparse
import time
from datetime import datetime
import random
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
import open3d as o3d

filename = './logs/time_signal.txt'

with open(filename, 'r') as filename:
   time_signals = [[float(num) for num in line.strip().split(' ')] for line in filename]

for i in time_signals:
    plt.plot(i)
plt.show()
