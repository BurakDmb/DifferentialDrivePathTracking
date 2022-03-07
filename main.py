import numpy as np


class State():
    def __init__(self, x_, y_, theta_):
        if not (x_ is None or y_ is None or theta_ is None):
            self.x = x_
            self.y = y_
            self.theta = theta_
        else:
            self.x = 0
            self.y = 0
            self.theta = 0

    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.theta)


class Controller():
    def __init__(
            self, start_, goal_, R_=0.0325, L_=0.1,
            kP=1.0, kI=0.01, kD=0.01, dT=0.1, v=1.0,
            arrive_distance=1):

        self.current = start_
        self.goal = goal_
        self.R = R_  # in meter
        self.L = L_  # in meter

        self.E = 0   # Cummulative error
        self.old_e = 0  # Previous error

        self.Kp = kP
        self.Ki = kI
        self.Kd = kD

        self.desiredV = v
        self.dt = dT  # in second
        self.arrive_distance = arrive_distance
        return

    def uniToDiff(self, v, w):
        vR = (2*v + w*self.L)/(2*self.R)
        vL = (2*v - w*self.L)/(2*self.R)
        return vR, vL

    def diffToUni(self, vR, vL):
        v = self.R/2*(vR+vL)
        w = self.R/self.L*(vR-vL)
        return v, w

    def iteratePID(self):
        # Difference in x and y
        d_x = self.goal.x - self.current.x
        d_y = self.goal.y - self.current.y

        # Angle from robot to goal
        g_theta = np.arctan2(d_y, d_x)

        # Error between the goal angle and robot angle
        alpha = g_theta - self.current.theta
        # alpha = g_theta - math.radians(90)
        e = np.arctan2(np.sin(alpha), np.cos(alpha))

        e_P = e
        e_I = self.E + e
        e_D = e - self.old_e

        # This PID controller only calculates the angular
        # velocity with constant speed of v
        # The value of v can be specified by giving in parameter or
        # using the pre-defined value defined above.
        w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D

        w = np.arctan2(np.sin(w), np.cos(w))

        self.E = self.E + e
        self.old_e = e
        v = self.desiredV

        return v, w

    def fixAngle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def makeAction(self, v, w):
        x_dt = v*np.cos(self.current.theta)
        y_dt = v*np.sin(self.current.theta)
        theta_dt = w

        self.current.x = self.current.x + x_dt * self.dt
        self.current.y = self.current.y + y_dt * self.dt
        self.current.theta = self.fixAngle(
            self.current.theta + self.fixAngle(theta_dt * self.dt))
        return

    def isArrived(self):
        # print("Arrive check:", str(abs(self.current.x - self.goal.x)),
        #       str(abs(self.current.y - self.goal.y)))
        current_state = np.array([self.current.x, self.current.y])
        goal_state = np.array([self.goal.x, self.goal.y])
        difference = current_state - goal_state

        distance_err = difference @ difference.T
        if distance_err < self.arrive_distance:
            return True
        else:
            return False

    def runPID(self, parkour=None):
        x = [self.current.x]
        y = [self.current.y]
        theta = [self.current.theta]
        while(not self.isArrived()):
            v, w = self.iteratePID()
            self.makeAction(v, w)
            x.append(self.current.x)
            y.append(self.current.y)
            theta.append(self.current.theta)
            if parkour:

                parkour.drawPlot(x, y, theta)
            # time.sleep(self.dt)

            # Print or plot some things in here
            # Also it can be needed to add some max iteration for
            # error situations and make the code stable.
            # print(self.current.x, self.current.y, self.current.theta)
        return x, y, theta


def trackRoute(start, targets):
    current = start
    x = []
    y = []
    theta = []
    for target in targets:
        controller = Controller(current, target)
        x_, y_, theta_ = controller.runPID()
        x.extend(x_)
        y.extend(y_)
        theta.extend(theta_)
        current = controller.current
    return x, y, theta


# Starting point of the code
def main():
    # Define dt time, create controller, define start and goal points
    # In every iteration, get an action from PID and make the action,
    # after that, sleep for dt. Repeat that loop until reaching the goal state.

    start = State(-20.0, 15.0, np.radians(90))
    targets = [
        State(0, 20, 0),
        State(20, 10, 0),
        State(0, 5, 0),
        State(-10, -15, 0),
        State(0, -10, 0),
        State(8, -10, 0)]
    simulateParkour = True
    if simulateParkour:
        from Parkour import Parkour
        x, y, theta = trackRoute(start, targets)

        parkour = Parkour()
        parkour.drawPlot(x, y, theta, show=True)


if __name__ == "__main__":
    main()
