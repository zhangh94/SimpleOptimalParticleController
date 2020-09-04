import numpy as np


class Sensor:
    def __init__(self,measNoise):
        self.measNoise  = measNoise
        self.xMeas      = np.array([[0.0]])
        # print("Position Sensor Initialized with: ", self.measNoise, " m 1-sigma gaussian noise")

    def Measure(self,x):
        self.xMeas = x +  np.random.normal(0,self.measNoise)
        return self.xMeas
