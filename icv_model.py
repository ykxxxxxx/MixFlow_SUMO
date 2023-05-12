import numpy as np

class ICVModel:
    def __init__(self, s_0, v_0, a, b, time_gap, delay):
        self.s_0 = s_0
        self.v_0 = v_0
        self.a = a
        self.b = b
        self.time_gap = time_gap
        self.delay = delay

    def control(self, ego_vehicle, front_vehicle, leader_velocity):
        delta_x = ego_vehicle.position[0] - front_vehicle.position[0]
        delta_v = ego_vehicle.velocity - front_vehicle.velocity
        desired_velocity = leader_velocity + self.time_gap * delta_v

        # CACC control law
        s_star = self.s_0 + max(0, ego_vehicle.velocity * self.delay +
                                (ego_vehicle.velocity * delta_v) /
                                (2 * np.sqrt(self.a * self.b)))
        s_star = max(s_star, ego_vehicle.length)
        s_star = min(s_star, self.time_gap * desired_velocity)

        acc = self.a * (
            1 - np.power((ego_vehicle.velocity / desired_velocity), 4) -
            np.power((s_star / delta_x), 2))
        return acc
