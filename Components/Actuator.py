import numpy as np


class Actuator:
    def __init__(self,acutNoise):
        self.acutNoise  = acutNoise
        self.uInput     = 0.0
        # print("Position Actuator Initialized with: ", self.acutNoise, " m 1-sigma gaussian noise")

    def Actuate(self,uCmd):
        self.uInput = uCmd + np.random.normal(0,self.acutNoise)*uCmd
        return self.uInput