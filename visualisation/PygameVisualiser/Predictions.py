import numpy as np


class ConstCurvaturePredicter():
    def __init__(self):
        """
            assumes the agent will move with constant curvature
            basically an extension of the linear model
        """
        # this means we take the last 1 seconds worth of ticksteps for calculating velocity, curvature, etc..

    def make_predictions(self, past_positions):
        """ make predictions"""
        predictions = []

        for i in range(10):
            predictions[i] = {}


            position = self.predict_agent_position_at_n_secs_in_future(i, past_positions)
            predictions.append(position)
        return predictions

    def predict_agent_position_at_n_secs_in_future(self, n, past_positions):
        """ make predictions"""

        previous_points = past_positions
        circle_centre = self.circle_centre_from_three_points(previous_points[0], previous_points[1], previous_points[2])
        radius = self.distance_between_two_points(circle_centre, previous_points[0])

        if np.isfinite(radius):
            angular_velocity = self.get_angular_velocity_between_two_points(circle_centre, previous_points[2],
                                                                            previous_points[0], averaging_time_window)
            d_theta = -angular_velocity * n
            rel_x = agent.position[0] - circle_centre[0]
            rel_y = agent.position[1] - circle_centre[1]

            agent_circle_rotated_pos = [rel_x * np.cos(d_theta) + rel_y * np.sin(d_theta),
                                        -rel_x * np.sin(d_theta) + rel_y * np.cos(d_theta)]
            post_rotated_pos = [agent_circle_rotated_pos[0] + circle_centre[0],
                                agent_circle_rotated_pos[1] + circle_centre[1]]

            x_addition = post_rotated_pos[0] - agent.position[0]
            y_addition = post_rotated_pos[1] - agent.position[1]

        else:
            # longitudinal components
            longitudinal_velocity = self.get_longitudinal_velocity_between_two_points(previous_points[2],
                                                                                      previous_points[0],
                                                                                      averaging_time_window)

            x_addition = dt * longitudinal_velocity[0]
            y_addition = dt * longitudinal_velocity[1]

        predicted_x = agent.position[0] + x_addition
        predicted_y = agent.position[1] + y_addition
        return [predicted_x, predicted_y]

    def get_longitudinal_velocity_between_two_points(self, point1, point2, time_window):
        dist = self.distance_between_two_points(point1, point2)
        if dist == 0:
            return [0, 0]

        normal_direction = [point2[0] - point1[0], point2[1] - point1[1]]
        normalising_factor = dist / (time_window * np.linalg.norm(normal_direction))
        normalised_vel = [normal_direction[0] * normalising_factor, normal_direction[1] * normalising_factor]
        return normalised_vel

    def get_angular_velocity_between_two_points(self, circle_centre, point1, point2, time_window):
        """ get the angle subtended by two lines from the circle centre, then divide by the time taken"""
        angle_subtended = self.signed_angle_between_in_radians(circle_centre, point1, point2)
        omega = angle_subtended / time_window  # time taken is number of ticksteps, * tick/timestep conversion
        return omega

    def signed_angle_between_in_radians(self, centre, point1, point2):
        p1 = [point1[0] - centre[0], point1[1] - centre[1]]
        p2 = [point2[0] - centre[0], point2[1] - centre[1]]
        angle = np.arctan2(p1[0] * p2[1] - p1[1] * p2[0], p1[0] * p2[0] + p1[1] * p2[1]);
        return angle

    # def get_previous_positions(self, agent):
    #     keys = sorted(agent.past_trajectory.keys())
    #     if (len(keys) < self.length_of_curvature_moving_average_window_steps):
    #         return [agent.position, agent.position, agent.position]
    #
    #     pos1 = agent.position
    #     pos2 = agent.past_trajectory[keys[-int(self.length_of_curvature_moving_average_window_steps / 2)]].position
    #     pos3 = agent.past_trajectory[keys[-self.length_of_curvature_moving_average_window_steps]].position
    #
    #     return [pos1, pos2, pos3]

    def get_signed_curvature_from_three_points(self, a, b, c):
        area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

        d_ab = self.distance_between_two_points(a, b)
        d_bc = self.distance_between_two_points(b, c)
        d_ac = self.distance_between_two_points(a, c)
        distances = d_ab * d_ac * d_bc
        if not np.isfinite(distances):
            return np.inf

        return 4 * area / distances

    def distance_between_two_points(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return np.sqrt(dx ** 2 + dy ** 2)

    def circle_centre_from_three_points(self, p1, p2, p3):
        c = (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2
        a = (p2[0] - p3[0]) ** 2 + (p2[1] - p3[1]) ** 2
        b = (p3[0] - p1[0]) ** 2 + (p3[1] - p1[1]) ** 2

        s = 2 * (a * b + b * c + c * a) - (a * a + b * b + c * c)
        if a == 0 or b == 0 or c == 0 or s == 0:
            return [np.inf, np.inf]

        px = (a * (b + c - a) * p1[0] + b * (c + a - b) * p2[0] + c * (a + b - c) * p3[0]) / s
        py = (a * (b + c - a) * p1[1] + b * (c + a - b) * p2[1] + c * (a + b - c) * p3[1]) / s

        return [px, py]

