#
# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
import numpy as np
import matplotlib.pyplot as plt


class AGV:
    """"
        Test environment for the AGV models; kinematic & dynamic.
    """
    def __init__(self):
        self.m = 1600                   # AGV mass                                                  [kg]
        self.a = 1.4                    # Distance front wheel(s) to centre of gravity (CoG)        [m]
        self.b = 1.6                    # Distance rear wheel(s) to centre of gravity (CoG)         [m]
        self.length = self.a + self.b   # AGV length                                                [m]
        self.steering_angle = 1         # Steering angle                                            [deg]
        self.speed = 0                  # Total speed                                               [m/s]
        self.heading_angle = 0          # Heading angle                                             [rad]

        # todo damping for whole system enough? should we add stuff? how to choose damping
        self.agv_damping_c = 20000      # Damping coefficient for full system dynamics              [kg/s]
        self.acceleration = 0           # Acceleration                                              [m/s^2]
        self.throttle_setpoint = 0      # Throttle setpoint                                         [-]
        self.side_slip = 0              # Vehicle side slip angle                                   [rad]

        self.A_l, self.B_l, self.C_l, self.D_l = self.state_space_longitudinal_motion()

    def state_space_longitudinal_motion(self):
        """"
            State space of a mass that has a resulting acceleration force from F=ma with
            a damper representing the drivetrain, wheel inertias and rest of the system.

            State vector contains longitudinal speed and position.
        """
        A = np.array([[-self.agv_damping_c/self.m, 0],
                      [1, 0]])
        B = np.array([[self.agv_damping_c/self.m],
                      [0]])
        C = np.array([[1, 0],
                      [0, 0]])
        D = np.array([[0],
                      [0]])

        return A, B, C, D

    def kinematic_bicycle_model(self, Ts, steering_angle, throttle_setpoint, dynamic_model):
        """
            Kinematic bicycle model with kinematic or dynamic longitudinal model. Inputs are steering angle and
            acceleration. A sampling rate has to be specified.

        :return: output coordinate of the system [x, y]
        """

        self.steering_angle = steering_angle
        self.throttle_setpoint = throttle_setpoint

        v_x = self.speed * np.cos(self.heading_angle + self.side_slip)
        v_y = self.speed * np.sin(self.heading_angle + self.side_slip)
        omega = (self.speed / self.length) * np.cos(self.side_slip) * np.tan(self.steering_angle)
        self.side_slip = np.arctan((self.b / self.length) * np.tan(self.steering_angle))
        acceleration = self.throttle_to_acceleration(self.throttle_setpoint)

        # Update heading angle and speed
        self.heading_angle = self.heading_angle + omega * Ts

        if dynamic_model:
            # Dynamical longitudinal motion
            self.speed = self.speed + self.run_longitudinal_self(Ts, acceleration)
        else:
            # Kinematic longitudinal motion
            self.speed = self.speed + self.acceleration * Ts

        # Update vehicle position.
        x_delta = v_x * Ts
        y_delta = v_y * Ts

        return x_delta, y_delta

    def run_longitudinal_self(self, Ts, acceleration):
        """"
            Mass-damper system to model the dynamics between throttle setpoint and torque at the wheels. This model
            represents all internal losses in the motor and driveline as well as the resistance forces such as rolling
            and drag resistance.

            From Newtons 2nd law with x_d as the derivative and x_dd as the second order derivative:
                mass * x_dd + agv_damping * x_d = acceleration * mass
        """
        accel_out = (-self.agv_damping_c/self.m) * self.speed + (self.agv_damping_c/self.m)*acceleration

        return self.speed + accel_out * Ts

    def throttle_to_acceleration(self, throttle_setpoint):
        """"
            On the AGV a change in throttle results in acceleration or deacceleration of the AGV. How much this
            acceleration is can be calculated with the equation of motion. The motor delivers a forward force and the
            resistance forces considered are the rolling resistance and the internal resistances in the AGV.

            NOTE: for now acceleration is just provided, so no transfer between throttle and acceleration.
        """
        # Set throttle setpoint
        self.throttle_setpoint = throttle_setpoint

        # Conversion to acceleration
        acceleration = throttle_setpoint

        return acceleration

    def run_kinematic_bicyle_model(self, x_init, y_init, Ts, duration, steering_angle, acceleration, dynamic_model):
        """"
            Simulate model to obtain output of x and y coordinates of the vehicle. States of the model are
            speed, side slip angle, heading angle. The inputs are the steering angle and throttle position/acceleration.
        """
        steering_angle_rad = steering_angle * (np.pi / 180)

        x_coords, y_coords = [], []
        x_pos, y_pos = x_init, y_init
        for _ in range(duration):
            x_delta, y_delta = agv.kinematic_bicycle_model(Ts, steering_angle_rad, acceleration, dynamic_model)
            x_pos = x_pos + x_delta
            y_pos = y_pos + y_delta
            x_coords.append(x_pos)
            y_coords.append(y_pos)

        return x_coords, y_coords

    def visualize(self, x_pos, y_pos):
        fig, ax = plt.subplots()

        x_min = min(x_pos)
        x_max = max(x_pos)
        y_min = min(y_pos)
        y_max = max(y_pos)
        # Add margins to avoid position on boundary of graph.
        ax.set_xlim(x_min - 10, x_max + 10)
        ax.set_ylim(y_min - 10, y_max + 10)

        for x, y in zip(x_pos, y_pos):
            ax.plot(x, y, marker='o', linestyle='None', color='r')
            plt.pause(0.2)


if __name__ == '__main__':
    agv = AGV()

    # Create path
    x1, y1 = agv.run_kinematic_bicyle_model(0, 0, 0.1, 10, 4, 0, True)
    x2, y2 = agv.run_kinematic_bicyle_model(x1[-1], y1[-1], 0.1, 5, 0, 4, True)
    x3, y3 = agv.run_kinematic_bicyle_model(x2[-1], y2[-1], 0.1, 10, 6, 3, True)
    x4, y4 = agv.run_kinematic_bicyle_model(x3[-1], y3[-1], 0.1, 6, -7, -4, True)

    xs = np.concatenate([x1, x2, x3, x4])
    ys = np.concatenate([y1, y2, y3, y4])
    agv.visualize(xs, ys)