import math


def boundBetweenPIandNegativePI(radians):
    while (radians >= math.pi):
        radians -= 2 * math.pi
    while (radians < -math.pi):
        radians += 2 * math.pi

    return radians


def bound_between_zero_and_two_pi(radians):
    while radians > 2 * math.pi:
        radians -= 2 * math.pi
    while radians <= 0:
        radians += 2 * math.pi

    return radians


def getDifRadians(f, t):
    return boundBetweenPIandNegativePI(t - f)


def almostEqual(n1, n2):
    return abs(n1 - n2) < math.pow(10, -6)
