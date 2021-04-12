import math
from ..util import mathUtil
from ..pathPlanning import pathTarget

class Spline:
    def __init__(self, x_offset, y_offset, knot_distance, theta_offset, a, b, c, d, e):
        self.x_offset = x_offset #the location from world frame to spline frame
        self.y_offset = y_offset
        self.knot_distance = knot_distance #Key definition of the spline
        self.theta_offset = theta_offset
        self.a = a #ax^5
        self.b = b #bx^4
        self.c = c #cx^3
        self.d = d #dx^2
        self.e = e #ex
        #f will always be 0
        self.arc_len = -1 # Serves to store calculated length and as a flag against calculating twice

    def __str__(self):
        return "Spline: " + str(self.a) + "x^5 + " + str(self.b) + "x^4 + " + str(self.c) + "x^3 + " + str(self.d) + "x&2 " + str(self.e) + "x\n" + "Located at: (" + str(self.x_offset) + "," + str(self.y_offset) + ")"

    def get_pos(self, percentage):
        #Limit to percentage values
        percentage = max(min(percentage, 1), 0)

        #define x and y vector in spline frame
        x_hat = percentage * self.knot_distance
        y_hat = self.a * math.pow(x_hat, 5) + self.b * math.pow(x_hat, 4) + self.c * math.pow(x_hat ,3) + self.d * math.pow(x_hat, 2) + self.e * x_hat

        #finally translate and rotate from spline frame to world frame
        cos_theta = math.cos(self.theta_offset)
        sin_theta = math.sin(self.theta_offset)

        x = x_hat * cos_theta - y_hat * sin_theta + self.x_offset
        y = x_hat * sin_theta + y_hat * cos_theta + self.y_offset

        return (x, y)

    def derivative_at(self, percentage):
        percentage = max(min(percentage, 1.0), 0.0)

        x_hat = percentage * self.knot_distance
        yp_hat = (5 * self.a * x_hat + 4 * self.b) * math.pow(x_hat, 3) + 3 * self.c * math.pow(x_hat, 2) + 2 * self.d * x_hat + self.e

        return yp_hat

    def calculate_length(self):
        if self.arc_len >= 0:
            return self.arc_len

        k_num_samples = 100000
        calc_len = 0.0
        integrand = last_integrand = math.sqrt(1 + math.pow(self.derivative_at(0),2)) / k_num_samples

        for i in range(1, k_num_samples+1):
            t = i / k_num_samples
            dydt = self.derivative_at(t)
            integrand = math.sqrt(1+math.pow(dydt, 2)) / k_num_samples
            calc_len += (integrand + last_integrand) / 2
            last_integrand = integrand

        self.arc_len = self.knot_distance * calc_len

        return self.arc_len

    def get_percentage_for_distance(self, distance):
        k_num_samples = 100000
        calc_len = last_calc_len = 0.0
        t = dydt = 0.0
        integrand = last_integrand = math.sqrt(1 + math.pow(self.derivative_at(0), 2)) / k_num_samples

        distance /= self.knot_distance

        for i in range(1, 1 + k_num_samples):
            t = i / k_num_samples
            dydt = self.derivative_at(t)
            integrand = math.sqrt(1 + math.pow(dydt, 2)) / k_num_samples
            calc_len += (integrand + last_integrand) / 2
            if calc_len > distance:
                break

            last_integrand = integrand
            last_calc_len = calc_len

        interpolated = t
        if (calc_len != last_calc_len):
            interpolated += ((distance - last_calc_len) / (calc_len - last_calc_len) - 1) / k_num_samples

        return interpolated

    def angle_at(self, percentage):
        return mathUtil.bound_between_zero_and_two_pi(math.atan(self.derivative_at(percentage)) + self.theta_offset)

def generate_quitic_spline(x0, y0, theta0, x1, y1, theta1):
    # The location of the base of the spline
    x_offset = x0
    y_offset = y0

    x1_hat = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
    if (x1_hat == 0):
        return False

    # More definitionn of world frame to spline frame
    knot_distance = x1_hat
    theta_offset = math.atan2(y1 - y0, x1 - x0)


    theta0_hat = mathUtil.getDifRadians(theta_offset, theta0)
    theta1_hat = mathUtil.getDifRadians(theta_offset, theta1)

    print(theta_offset)
    print(theta0_hat)
    print(theta1_hat)

    ## Cannot solve a spline where the eventual angle is perpendicular to the line between points
    if mathUtil.almostEqual(theta0_hat, math.pi / 2) or mathUtil.almostEqual(theta1_hat, math.pi / 2):
        print("eventual angle too steep")
        return False

    ## Cannot handle end angles facing initial angles, this is intended and should be countered by defining multiple waypoints
    if abs(mathUtil.getDifRadians(theta0_hat, theta1_hat)) > math.pi / 2:
        print("Doubling back")
        return False

    ## Take derrivatives of angles
    yp0_hat = math.tan(theta0_hat)
    yp1_hat = math.tan(theta1_hat)

    ##Now defining the components of the spline from solving the sequential ddifferential eqyations
    a = -(3 * (yp0_hat + yp1_hat)) / math.pow(x1_hat, 4)
    b = (8 * yp0_hat + 7 * yp1_hat) / math.pow(x1_hat, 3)
    c = -(6 * yp0_hat + 4 * yp1_hat) / math.pow(x1_hat, 2)
    d = 0
    e = yp0_hat

    return Spline(x_offset, y_offset, knot_distance, theta_offset, a, b, c, d, e)


# Applies a hermitic cubic spline fit given start and end positions
def generate_cubic_spline(x0, y0, theta0, x1, y1, theta1):
    # The location of the base of the spline
    x_offset = x0
    y_offset = y0

    x1_hat = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
    if (x1_hat == 0):
        return False

    # More definitionn of world frame to spline frame
    knot_distance = x1_hat
    theta_offset = math.atan2(y1 - y0, x1 - x0)


    theta0_hat = mathUtil.getDifRadians(theta_offset, theta0)
    theta1_hat = mathUtil.getDifRadians(theta_offset, theta1)

    print(theta_offset)
    print(theta0_hat)
    print(theta1_hat)

    ## Cannot solve a spline where the eventual angle is perpendicular to the line between points
    if mathUtil.almostEqual(theta0_hat, math.pi / 2) or mathUtil.almostEqual(theta1_hat, math.pi / 2):
        print("eventual angle too steep")
        return False

    ## Cannot handle end angles facing initial angles, this is intended and should be countered by defining multiple waypoints
    if abs(mathUtil.getDifRadians(theta0_hat, theta1_hat)) > math.pi / 2:
        print("Doubling back")
        return False

    ## Take derrivatives of angles
    yp0_hat = math.tan(theta0_hat)
    yp1_hat = math.tan(theta1_hat)

    ##Now defining the components of the spline from solving the sequential ddifferential eqyations
    a = 0
    b = 0
    c = (yp1_hat + yp0_hat) / math.pow(x1_hat, 2)
    d = -(2*yp0_hat + yp1_hat) / x1_hat
    e = yp0_hat

    return Spline(x_offset, y_offset, knot_distance, theta_offset, a, b, c, d, e)


def generate_cubic_spline_targets(target_start, target_end):
    return generate_cubic_spline(target_start.x, target_start.y, target_start.theta, target_end.x, target_end.y, target_end.theta)
