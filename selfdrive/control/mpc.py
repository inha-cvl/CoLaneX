import cvxpy
import math
import numpy as np
import control.utils.cubic_spline_planner as cubic_spline_planner
from control.utils.angle import angle_mod

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None


class MPC(object):
    def __init__(self, NX, NU, T, max_iter, du_th, n_ind_search, dt, max_accel, \
                back_to_wheel, wheel_len, wheel_width, tread, \
                vehicle_length, vehicle_width, wheelbase,steer_max, dsteer_max, max_velocity):
        self.NX = NX
        self.NU = NU
        self.T = T
        self.max_iter = max_iter
        self.du_th = du_th
        self.n_ind_search = n_ind_search
        self.dt = dt
        self.max_accel = max_accel
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.back_to_wheel = back_to_wheel
        self.wheel_len = wheel_len
        self.wheel_width = wheel_width
        self.tread = tread
        self.wheelbase = wheelbase
        self.steer_max = np.deg2rad(steer_max) #maximum steering angle
        self.dsteer_max = np.deg2rad(dsteer_max) #maximum steering speed
        self.max_velocity = max_velocity/3.6
        self.min_velocity = 0

        # mpc parameters
        self.R = np.diag([0.01, 0.01])  # input cost matrix
        self.Rd = np.diag([0.01, 1.0])  # input difference cost matrix
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
        self.Qf = self.Q  # state final matrix

        self.path = []
        self.path_updated = False
        self.cx = None
        self.cy = None
        self.cyaw = None
        self.ck = None
        self.sp = None
        self.target_ind = None
        self.state = None
        self.oa = None
        self.odelta = None

    def update_path(self, path):
        self.path = path
        self.path_updated = True

    def pi_2_pi(self, angle):
        return angle_mod(angle)
    

    def get_linear_model_matrix(self, v, phi, delta):

        A = np.zeros((self.NX, self.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.dt * math.cos(phi)
        A[0, 3] = - self.dt * v * math.sin(phi)
        A[1, 2] = self.dt * math.sin(phi)
        A[1, 3] = self.dt * v * math.cos(phi)
        A[3, 2] = self.dt * math.tan(delta) / self.wheelbase

        B = np.zeros((self.NX, self.NU))
        B[2, 0] = self.dt
        B[3, 1] = self.dt * v / (self.wheelbase * math.cos(delta) ** 2)

        C = np.zeros(self.NX)
        C[0] = self.dt * v * math.sin(phi) * phi
        C[1] = - self.dt * v * math.cos(phi) * phi
        C[3] = - self.dt * v * delta / (self.wheelbase * math.cos(delta) ** 2)

        return A, B, C

    def update_state(self, state, a, delta):

        # input check
        if delta >= self.steer_max:
            delta = self.steer_max
        elif delta <= -self.steer_max:
            delta = -self.steer_max

        state.x = state.x + state.v * math.cos(state.yaw) * self.dt
        state.y = state.y + state.v * math.sin(state.yaw) * self.dt
        state.yaw = state.yaw + state.v / self.wheelbase * math.tan(delta) * self.dt
        state.v = state.v + a * self.dt

        if state.v > self.max_velocity:
            state.v = self.max_velocity
        elif state.v < self.min_velocity:
            state.v = self.min_velocity

        return state


    def get_nparray_from_matrix(self, x):
        return np.array(x).flatten()

    def calc_nearest_index(self, state, cx, cy, cyaw, pind):

        dx = [state.x - icx for icx in cx[pind:(pind + self.n_ind_search)]]
        dy = [state.y - icy for icy in cy[pind:(pind + self.n_ind_search)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind


    def predict_motion(self, x0, oa, od, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.T + 1)):
            state = self.update_state(state, ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar

    def iterative_linear_mpc_control(self, xref, x0, dref, oa, od):
        """
        MPC control with updating operational point iteratively
        """
        ox, oy, oyaw, ov = None, None, None, None

        if oa is None or od is None:
            oa = [0.0] * self.T
            od = [0.0] * self.T

        for i in range(self.max_iter):
            xbar = self.predict_motion(x0, oa, od, xref)
            poa, pod = oa[:], od[:]
            oa, od = self.linear_mpc_control(xref, xbar, x0, dref)
            if oa is None or od is None:
                break
            du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
            if du <= self.du_th:
                break
        else:
            print("Iterative is max iter")

        return oa, od


    def linear_mpc_control(self, xref, xbar, x0, dref):
        """
        linear mpc control

        xref: reference point
        xbar: operational point
        x0: initial state
        dref: reference steer angles
        """

        x = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))

        cost = 0.0
        constraints = []

        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)

            if t != 0:
                cost += cvxpy.quad_form(xref[:, t] - x[:, t], self.Q)

            A, B, C = self.get_linear_model_matrix(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

            if t < (self.T - 1):
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                                self.dsteer_max * self.dt]

        cost += cvxpy.quad_form(xref[:, self.T] - x[:, self.T], self.Qf)
        
        constraints += [x[:, 0] == x0]
        constraints += [x[2, :] <= self.max_velocity+5]
        constraints += [x[2, :] >= self.min_velocity]
        constraints += [cvxpy.abs(u[0, :]) <= self.max_accel]
        constraints += [cvxpy.abs(u[1, :]) <= self.steer_max]
  
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve(solver=cvxpy.ECOS, verbose=False)

        print(prob.status)

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            # ox = self.get_nparray_from_matrix(x.value[0, :])
            # oy = self.get_nparray_from_matrix(x.value[1, :])
            # ov = self.get_nparray_from_matrix(x.value[2, :])
            # oyaw = self.get_nparray_from_matrix(x.value[3, :])
            oa = self.get_nparray_from_matrix(u.value[0, :])
            odelta = self.get_nparray_from_matrix(u.value[1, :])

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta = None, None

        return oa, odelta

    def calc_ref_trajectory(self, state, cx, cy, cyaw, ck, sp, dl, pind):
        xref = np.zeros((self.NX, self.T + 1))
        dref = np.zeros((1, self.T + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(state, cx, cy, cyaw, pind)

        if pind >= ind:
            ind = pind

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = sp[ind]
        xref[3, 0] = cyaw[ind]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(self.T + 1):
            travel += abs(state.v) * self.dt
            dind = int(round(travel / dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = sp[ind + dind]
                xref[3, i] = cyaw[ind + dind]
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = sp[ncourse - 1]
                xref[3, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0

        return xref, ind, dref



    def calc_speed_profile(self, cx, cy, cyaw, target_speed):

        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(self.pi_2_pi(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = target_speed

        speed_profile[-1] = 0.0

        return speed_profile


    def smooth_yaw(self, yaw):

        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw


    def get_mpc_result(self, vehicle_state):
        self.state =  State(x=vehicle_state.position.x, y=vehicle_state.position.y, yaw=vehicle_state.heading, v=vehicle_state.velocity/3.6)
        if self.path_updated:
            ax = [pt.x for pt in self.path]
            ay = [pt.y for pt in self.path]
            self.cx, self.cy, self.cyaw, self.ck, s = cubic_spline_planner.calc_spline_course(ax, ay, 0.5)
            target_velocity = 80/3.6
            self.sp = self.calc_speed_profile(self.cx, self.cy, self.cyaw, target_velocity)

            # initial yaw compensation
            if self.state.yaw - self.cyaw[0] >= math.pi:
                self.state.yaw -= math.pi * 2.0
            elif self.state.yaw - self.cyaw[0] <= -math.pi:
                self.state.yaw += math.pi * 2.0

            self.target_ind, _ = self.calc_nearest_index(self.state, self.cx, self.cy, self.cyaw, 0)

            self.odelta, self.oa = None, None
            self.cyaw = self.smooth_yaw(self.cyaw)
            self.path_updated = False

        xref, self.target_ind, dref = self.calc_ref_trajectory(self.state, self.cx, self.cy, self.cyaw, self.ck, self.sp, 0.5, self.target_ind)
        x0 = [vehicle_state.position.x,vehicle_state.position.y, vehicle_state.velocity/3.6, vehicle_state.heading]
        self.oa, self.odelta = self.iterative_linear_mpc_control(xref, x0, dref, self.oa, self.odelta)

        di, ai = 0.0, 0.0
        if self.odelta is not None:
            di, ai = self.odelta[0], self.oa[0]
            self.state = self.update_state(self.state, ai, di)
        return ai, di