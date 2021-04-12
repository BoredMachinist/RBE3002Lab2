
## A representation of a single target along the computed trajectory
class Segment:

    def __init__(self, *args):
        if len(args) == 0:
            self.pos = 0  # Somewhat misleading, represents distance traveled along a path
            self.vel = 0  # fairly self evident
            self.acc = 0  # same dv/dt
            self.jerk = 0  # da/dt
            self.heading = 0  # target orientation around Z axis world frame
            self.dt = 0  # Time step
            self.x = 0
            self.y = 0  # target positions in world frame

        if len(args) == 8:
            self.pos = args[0] # Somewhat misleading, represents distance traveled along a path
            self.vel = args[1] # fairly self evident
            self.acc = args[2] # same dv/dt
            self.jerk = args[3] # da/dt
            self.heading = args[4] #target orientation around Z axis world frame
            self.dt = args[5] # Time step
            self.x = args[6]
            self.y = args[7] # target positions in world frame
        if len(args) == 5: #(pos, vel, acc, jerk, dt)
            self.pos = args[0]  # Somewhat misleading, represents distance traveled along a path
            self.vel = args[1]  # fairly self evident
            self.acc = args[2]  # same dv/dt
            self.jerk = args[3]  # da/dt
            self.heading = 0  # target orientation around Z axis world frame
            self.dt = args[4]  # Time step
            self.x = 0
            self.y = 0  # target positions in world frame

## A collection of targets arranged in a path with corresponding velocities
class Trajectory:
    def __init__(self, size):
        self.segments = [] # The collection of points along the trajectory
        self.segments = [Segment() for i in range(size)]

    def append(self, toAppend):
        self.segments.append(toAppend.segments)

    def get_num_segments(self):
        return len(self.segments)

    def get_segment(self, index):
        return self.segments[index]
