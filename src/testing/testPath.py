import numpy as np
import matplotlib.pyplot as plt
import math

from src.pathPlanning import pathTarget
from src.pathPlanning import pathGenerator
from src.pathPlanning import trajectoryGenerator

burgerConfig = trajectoryGenerator.Config(.05, 0.22, 2.5, 10)

path = pathTarget.PathTargetList()

path.addTarget(0, 0, 0)
path.addTarget(.5, .3, math.pi/4)
path.addTarget(.7, .2, -1*math.pi/6)

traj = pathGenerator.generateFromPath(path, burgerConfig)

f, (pos, theta) = plt.subplots(1, 2, gridspec_kw={'width_ratios': [3, 1]})

f.suptitle("Trajectory Planning in 2D space (one point per .05 s)")

pos.title.set_text("2D Position")
pos.set_xlabel("X (m)")
pos.set_ylabel("Y (m)")

theta.title.set_text("Heading")
theta.set_xlabel("Heading (radians)")
theta.set_ylabel("Time (s)")

t = 0
for seg in traj.segments:
    pos.plot(seg.x, seg.y, markersize=2, marker='o')
    theta.plot(seg.heading, t, markersize=2, marker='o')

    t += seg.dt

for point in path.targets:
    pos.plot(point.x, point.y, markersize=3, marker='x')

plt.show()



