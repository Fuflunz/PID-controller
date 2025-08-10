class PID:
    dt = 0.
    kp = 0.
    kd = 0.
    ki = 0.
    err = 0.
    dT = 0.
    count = 0.
    Int = 0.
    range = 0.

    def __init__(self, dt, kp, kd, ki, dT, range):
        self.dt = dt  # dt = Zeit in s
        self.kp = kp  # kp = Proportionalkonstante
        self.kf = kd  # kd = Differentialkonstante
        self.ki = ki  # ki = Integralkonstante
        self.dT = dT  # dT = Integrationszeit in s
        self.range = range  # range, ab wann Differentialteil eingreifen soll
        # range ist in der Unit des zu korrigierenden Wertes

    def correct(self, set, act):
        e = set - act

        P = self.kp * e

        if (self.err < self.range):
            if (self.count * self.dt == self.dT):
                I = self.Int
                self.count = 0
                self.Int = 0.
                self.Int += self.ki * e * self.dt
                self.count += 1
                # print("Integrate")
                # print(self.count)
            else:
                I = 0.
                self.Int += self.ki * e * self.dt
                self.count += 1
                # print(self.count)

        if (self.err < self.range):
            D = -self.kd * ((e - self.err) / self.dt)
        else:
            D = 0.

        out = P + I + D

        self.err = e
        return out 