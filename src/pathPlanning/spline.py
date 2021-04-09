import math


class Spline:
    def __init__(self,):


def generate_quitic_spline(x0, y0, theta0, x1, y1, theta1):
    x_offset = startX
    y_offset = startY

    x1_hat = math.sqrt((x1-x0)*(X1-x0)+(y1-y0)*(y1-y0))
    if (x1_hat == 0):
        return False

    knot_distance = x1_hat
    theta_offset = math.atan(y1-y0, x1-x0)

    theta0_hat=theta_offset-theta0