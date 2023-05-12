class CHVModel:
    def __init__(self, v_max, a, b, delta, s_0):
        self.v_max = v_max
        self.a = a
        self.b = b
        self.delta = delta
        self.s_0 = s_0

    def control(self, ego_vehicle, front_vehicle):
        delta_x = ego_vehicle.position[0] - front_vehicle.position[0]
        delta_v = ego_vehicle.velocity - front_vehicle.velocity
        s_star = self.s_0 + max(0, ego_vehicle.velocity * self.delta +
                                (ego_vehicle.velocity * delta_v) /
                                (2 * np.sqrt(self.a * self.b)))
        acc = self.a * (1 - np.power((ego_vehicle.velocity / self.v_max), 4)
                        - np.power((s_star / delta_x), 2))
        return acc
