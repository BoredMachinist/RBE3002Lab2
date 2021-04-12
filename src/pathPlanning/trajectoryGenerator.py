from ..pathPlanning import trajectory
import math


# Reresents robot specific config, should probably do clever ROS stuff to load this
class Config:
    def __init__(self, dt, max_vel, max_acc, max_jerk):
        self.dt = dt
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_jerk = max_jerk





def firSecondOrder(tF1, tF2, dt, start_vel, max_vel, total_impulse, length):
    """Implementation of a FIR filter applying two functions to the input, f1, and f2... f1 acts as a low pass filter
    on the output rate of change. Therefore acts as a acceleration limit, f2 is another low pass filter applied to the
    output of f1 therfore acting as a jerk limit

    Principals discussed here: https://www.chiefdelphi.com/t/motion-profiling/115133/18

    Parameters
    ----------
    tF1 : int
        The length of the filter for acceleration,
    tF2 : int
        The length of the filter for jerk
    dt : float
        itp, itteration time of the filter in this case the timestep between trajectory points, should be made as close
        as possible to sending frequency of the controller
    start_vel : float
        The velocity to start the trajectory from, in FIR terms the initial value
    max_vel : float
        The maximum velocity for the output to reach, in FIR terms the amount to scale output by. It is important to note
        that for short paths this is likely not the maximum velocity of the robot.
    total_impulse : float
        The length of signal upon which to apply positive filtering, can be non integer will be rounded up
    length : int
        The total signal length over which to apply filtering, length*dt should give total travel time.

    Returns
    -------
    trajectory.Trajectory
        A purely scalar trajectory (using trajectory.Segment 5 argument construction)
    """

    # The trajectory to be output
    trajectory_out = trajectory.Trajectory(length)

    # Create first segment
    last = trajectory.Segment(0, start_vel, 0, 0, dt)

    ## Buffer of f1 float
    f1 = [0 for i in range(length)]

    f2 = 0

    for i in range(length):

        print("Step: " + str(i) + " out of " + str(length))

        # COunt down total impulse, slightly modified to account for non int inputs
        input = min(total_impulse, 1)
        if (input < 1):
            # After the period of acceleration it should begin deccelerating
            input-=1
            total_impulse = 0
        else:
            total_impulse -= input

        f1_last = 1
        if i > 0:
            f1_last = f1[i-1]
        else:
            f1_last = f1[0]

        f1[i] = max(0.0, min(tF1, f1_last + input)) # low pass filtering on f1

        f2 = 0
        # Itterate over existing f1 values to apply low pass on f1 output
        for j in range(tF2):
            if i - j == 0:
                break

            f2 += f1[i - j] # add the last tF2 elements from f1 to the running total

        f2 /= tF1 #Scale by the length of the first linear filter

        trajectory_out.segments[i].vel = f2 / tF2 * max_vel # Velocity is normalized sum of f2 scaled by max velocity

        # Integrate to solve for position using trapezoidal integration
        trajectory_out.segments[i].pos = (last.vel + trajectory_out.segments[i].vel) / 2.0 * dt + last.pos

        # Acceleration and jerk are diferences in vel, and acc with respect to dt
        trajectory_out.segments[i].acc = (trajectory_out.segments[i].vel - last.vel) / dt
        trajectory_out.segments[i].jerk = (trajectory_out.segments[i].acc - last.acc) / dt
        trajectory_out.segments[i].dt = dt

        last = trajectory_out.segments[i]

    return trajectory_out


# Creating a S shaped trajectory to achieve maximum velocity whilst accounting for jerk
def generate(config, start_vel, goal_pos, goal_vel):
    """Generates a scalar trajectory of length goal_pos


    """

    # Implementation of a s curve based trajectory, currently only supports point to point 0 vel to 0 vel movement
    #   may require implementing trapezoidal motion profile in the future

    # mathematics ased of this paper http://www.et.byu.edu/~ered/ME537/Notes/Ch5.pdf

    # define a maximum velocity to be achieved by the robot over the trajectory, for longer trajectories will bemax_vel
    #   however on shorter trajectories this will be limeted by acceleration and jerk
    adjusted_max_vel = min(config.max_vel, (-1 * math.pow(config.max_acc, 2) + math.sqrt(math.pow(config.max_acc, 4) + 4 * math.pow(config.max_jerk, 2) * config.max_acc * goal_pos)) / (2 * config.max_jerk))

    # Calculate lenght of linear filters for acceleration and jerk along with the impulse
    tf1 = math.ceil((adjusted_max_vel / config.max_acc) / config.dt)
    tf2 = math.ceil((config.max_acc / config.max_jerk) / config.dt)
    # The impulse is basicly calculated from the time to make the distance if at max velocity the whole time
    impulse = (goal_pos / adjusted_max_vel) / config.dt

    # The time therfore is the sum of the previous values
    time = math.ceil(tf1 + tf2 + impulse)

    trajectory = firSecondOrder(tf1, tf2, config.dt, 0, adjusted_max_vel, impulse, time)

    return trajectory