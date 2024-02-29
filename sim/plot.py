#!/usr/bin/env python3

from matplotlib import legend
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/tmp/sim.csv", index_col="time")

ax_l = df["position"].plot()
df["setpoint"].plot(ax=ax_l, style=":")

ax_r = ax_l.twinx()

# df[["pid_p", "pid_i", "pid_d", "throttle"]].plot(ax=ax_r, secondary_y=True)

plt.show()
