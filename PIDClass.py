class PID:
    dt = 0.
    kp = 0.
    kd = 0.
    ki = 0.
    err = 0.
    dT = 0.
    count = 0.
    Int = 0.
    
    def __init__(self, dt, kp, kd, ki, dT, Offset):   
        self.dt = dt                                 # dt = Zeit in s
        self.kp = kp                                 # kp = Proportionalkonstante
        self.kf = kd                                 # kd = Differentialkonstante
        self.ki = ki                                 # ki = Integralkonstante
        self.dT = dT                                 # dT = Integrationszeit in s
        self.Offset = Offset
        self.err = 0
        
    def correct(self, set, act, range1, range2, range3, range4, Out_max, Out_min):
        e = set - act
        
        P = self.kp * e
        
        if(range2 < act) and (act < range1):
            print("in Range (Int)")
            self.Int += self.err
            I = self.ki * self.dt * self.Int
        else:
            self.Int = 0
            I = 0.
            
        if(range4 < act < range3):
            print("in Range (Diff)")
            D = -self.kd * ((e-self.err)/(self.dt))
        else:
            D = 0.
            
        out = P+I+D+self.Offset
        if(out > Out_max):
            o = Out_max
        elif(out<Out_min):
            o = Out_min
        else:
            o = out
        
        print("P")
        print(P)
        print("I")
        print(I)
        print("D")
        print(D)
        
        self.err = e
        return o
