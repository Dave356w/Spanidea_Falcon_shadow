#!/usr/bin/python3
import pandas as pd
from matplotlib import pyplot as plt
import sys
import math
from collections import deque

file = "./test.log"
if len(sys.argv) == 2:
    file = sys.argv[1]

df = pd.read_csv(file, header=None, index_col=None, skiprows=15, skipfooter=0)#skiprows=200, skipfooter=4000

plt.figure()

plt.subplot(411)
plt.grid(visible=1, axis='both')
plt.plot(df[1])
plt.plot(df[2])
plt.title('Raw sensor digital data', {'fontsize': 10})

plt.subplot(412)
plt.grid(visible=1, axis='both')
plt.plot(df[3])
plt.plot(df[4], label='After removing gravity pull and drift adjustment')
plt.legend(loc=10)
plt.title('Acc in m/s2', {'fontsize': 10})

plt.subplot(413)
plt.grid(visible=1, axis='both')
plt.plot(df[6])
plt.plot(df[7], label='threshold calculated to detect the deceleration')
plt.legend(loc=1)
plt.title('velocity in m/s', {'fontsize': 10})

plt.subplot(414)
plt.grid(visible=1, axis='both')
plt.plot(df[8], label='Motion state: 0-Not moving, 1-Movement detected, 2-Moving, 3-Decelerating, 4-Stopped, 5-Error/Reset')
plt.plot(df[9], label='Alarm state: 0 - No alarm, 1 - alarm')
plt.legend(loc=1)
plt.title('Motion state and Alarm state', {'fontsize': 10})

# plt.subplot(413)
# plt.grid(visible=1, axis='both')
# plt.plot(df[11])
# plt.title('Pressure sensor data', {'fontsize': 10})

plt.show()