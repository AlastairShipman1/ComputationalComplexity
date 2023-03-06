import numpy as np


class ConstCurvaturePredicter():
    def __init__(self, vehicle, length_of_prediction_set):
        """
        this takes the previous positions [[x,y,dt], [1,2,3], [1,2,4], [3,4,5]]
        and extrapolates out
        """
        self.vehicle = vehicle
        self.number_of_predictions = 10
        self.prediction_time_jump = 0.15 #in seconds
        self.length_of_prediction_set = length_of_prediction_set

    def make_predictions(self, past_positions):
        """ make predictions"""

        if(len(past_positions)%2!=1):
            raise Exception("Past positions should have an odd number of elements")
        past_positions=past_positions[-self.length_of_prediction_set:]
        mid_index=int(len(past_positions) / 2)+1

        self.previous_points = [past_positions[-1], past_positions[mid_index], past_positions[0]]
        self.circle_centre = self.circle_centre_from_three_points(self.previous_points[0], self.previous_points[1],self.previous_points[2])
        self.radius = self.distance_between_two_points(self.circle_centre, self.previous_points[0])

        self.averaging_time_window = 0
        for p in past_positions:
            self.averaging_time_window += p[-1]
        self.averaging_time_window/= 1000  # in milliseconds

        predictions = []
        for i in range(self.number_of_predictions):
            position = self.predict_agent_position_at_n_secs_in_future(i)
            predictions.append(position)
        return predictions

    def predict_agent_position_at_n_secs_in_future(self, i):
        """ make predictions"""

        if np.isfinite(self.radius):
            angular_velocity = self.get_angular_velocity_between_two_points(self.circle_centre, self.previous_points[2][:-1],
                                                                            self.previous_points[0][:-1], self.averaging_time_window)
            d_theta = -angular_velocity * i * self.prediction_time_jump
            rel_x = self.vehicle.world_x - self.circle_centre[0]
            rel_y = self.vehicle.world_y - self.circle_centre[1]

            agent_circle_rotated_pos = [rel_x * np.cos(d_theta) + rel_y * np.sin(d_theta),
                                        -rel_x * np.sin(d_theta) + rel_y * np.cos(d_theta)]
            post_rotated_pos = [agent_circle_rotated_pos[0] + self.circle_centre[0],
                                agent_circle_rotated_pos[1] + self.circle_centre[1]]

            x_addition = post_rotated_pos[0] - self.vehicle.world_x
            y_addition = post_rotated_pos[1] - self.vehicle.world_y

        else:
            # longitudinal components
            longitudinal_velocity = self.vehicle.v_long
            #.get_longitudinal_velocity_between_two_points(self.previous_points[2],self.previous_points[0],self.averaging_time_window)

            x_addition = self.prediction_time_jump *i * longitudinal_velocity * np.cos(np.deg2rad(self.vehicle.direction))
            y_addition = - self.prediction_time_jump *i* longitudinal_velocity * np.sin(np.deg2rad(self.vehicle.direction))

        predicted_x = self.vehicle.world_x + x_addition
        predicted_y = self.vehicle.world_y + y_addition
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
        angle = np.arctan2(p1[0] * p2[1] - p1[1] * p2[0], p1[0] * p2[0] + p1[1] * p2[1])
        return angle

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
        temp = p2[0] * p2[0] + p2[1] * p2[1]
        bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
        cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
        det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

        if abs(det) < 1.0e-6:
            return [np.inf, np.inf]

        # Center of circle
        cx = (bc * (p2[1] - p3[1]) - cd * (p1[1] - p2[1])) / det
        cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

        return [cx, cy]

    def circle_centre_old(self, p1, p2, p3):
        #this is an unstable piece of code- it doesn't handle floating point errors that well.
        c = (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2
        a = (p2[0] - p3[0]) ** 2 + (p2[1] - p3[1]) ** 2
        b = (p3[0] - p1[0]) ** 2 + (p3[1] - p1[1]) ** 2

        s = 2 * (a * b + b * c + c * a) - (a * a + b * b + c * c)
        if a == 0 or b == 0 or c == 0 or s == 0:
            return [np.inf, np.inf]

        px = (a * (b + c - a) * p1[0] + b * (c + a - b) * p2[0] + c * (a + b - c) * p3[0]) / s
        py = (a * (b + c - a) * p1[1] + b * (c + a - b) * p2[1] + c * (a + b - c) * p3[1]) / s

        return [px, py]
