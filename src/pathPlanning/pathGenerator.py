from ..pathPlanning import pathTarget
from ..pathPlanning import trajectoryGenerator
from ..pathPlanning import spline


def generateFromPath(path, config):
    """
    Generates a trajectory over a seuence of targets
    :param path: pathTarget.PathTargetList
        The sequence of pathTargets to visit should include robot start and end pos
    :param config: trajectoryGenerator.Cofig
        A configuration relating to the specific robot
    :return: Trajectory or None
        A series of points along a trajectory seperated by a common time interval with complete x, y, and heading info,
        If returns None one of the desired waypoints is not achievable with as few steps to reach it
    """
    if path.numTargets() < 2:
        return None

    splines = [None for i in range(path.numTargets() - 1)]
    splineLengths = [0.0 for i in range(len(splines))]
    totalDistance = 0.0

    for i in range(len(splines)):
        newSpline = spline.generate_cubic_spline_targets(path.getTarget(i), path.getTarget(i+1))
        if newSpline == False:
            print("Error generating smooth path, check pathTargets for achievaility")
            return None

        splines[i] = newSpline
        splineLengths[i] = splines[i].calculate_length()
        print("Len: " + str(splineLengths[i]))
        totalDistance += splineLengths[i]

    trajectory = trajectoryGenerator.generate(config, 0.0, totalDistance, 0.0)

    cur_spline = 0
    cur_spline_start_pos = 0.0
    length_splines_finished = 0.0

    for i in range(trajectory.get_num_segments()):

        cur_pos = trajectory.get_segment(i).pos

        found_spline = False
        while not found_spline:
            cur_pos_relative = cur_pos - cur_spline_start_pos

            if cur_pos_relative <= splineLengths[cur_spline]:
                percentage = splines[cur_spline].get_percentage_for_distance(cur_pos_relative)

                trajectory.get_segment(i).heading = splines[cur_spline].angle_at(percentage)
                (x, y) = splines[cur_spline].get_pos(percentage)
                trajectory.get_segment(i).x = x
                trajectory.get_segment(i).y = y
                found_spline = True

            elif cur_spline < len(splines)- 1:
                length_splines_finished += splineLengths[cur_spline]
                cur_spline_start_pos = length_splines_finished
                cur_spline += 1

            else:
                trajectory.get_segment(i).heading = splines[len(splines) - 1].angle_at(1.0)

                (x,y) = splines[cur_spline].get_pos(1.0)
                trajectory.get_segment(i).x = x
                trajectory.get_segment(i).y = y

                found_spline = True

    return trajectory
