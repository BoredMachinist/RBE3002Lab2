import numpy as np
import matplotlib.pyplot as plt

import math
from src.pathPlanning import trajectory
from src.pathPlanning import trajectoryGenerator






print("helloString")

burgerConfig = trajectoryGenerator.Config(.01, 0.22, 2.5, 10)

traj = trajectoryGenerator.generate(burgerConfig, 0.0, .1, 0.0)

print("Generated")

fig = plt.figure(figsize=(8,12), dpi=160)
pos = fig.add_subplot(3,1,1)
vel = fig.add_subplot(3,1,2)
acc = fig.add_subplot(3,1,3)

pos.title.set_text("Pos")
pos.set_xlabel("Time (s)")
pos.set_ylabel("Distance (m)")

vel.title.set_text("Velocity")
vel.set_xlabel("Time (s)")
vel.set_ylabel("Velocity (m/s)")

acc.title.set_text("Acceleration")
acc.set_xlabel("Time (s)")
acc.set_ylabel("Acceleration (m/s^2)")

t = 0
for seg in traj.segments:
    pos.plot(t, seg.pos, markersize=2, marker='o')
    vel.plot(t, seg.vel, markersize=2, marker='o')
    acc.plot(t, seg.acc, markersize=2, marker='o')

    t += seg.dt

plt.show()