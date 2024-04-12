#!/usr/bin/env python3

from matplotlib import legend
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/tmp/sim.csv", index_col="time")

ax_l = df["position"].plot()
df["setpoint"].plot(ax=ax_l, style=":")
df["observer_p"].plot(ax=ax_l, style=":")

ax_r = ax_l.twinx()

# df["velocity"].plot(ax=ax_r)
# df["observer_v"].plot(ax=ax_r, style=":")
# (df["pid_pos_out"] / 2400).plot(ax=ax_r, style=":")


# df["pid_pos_out"].plot(ax=ax_r)
# df["pid_vel_out"].plot(ax=ax_r, secondary_y=True)
# df["throttle"].plot(ax=ax_r)
# df["throttle_raw"].plot(ax=ax_r)
# df[["pid_vel_p", "pid_vel_i", "pid_vel_d"]].plot(ax=ax_r, secondary_y=True)

ax_l.legend()
ax_r.legend(loc=1)
plt.show()
