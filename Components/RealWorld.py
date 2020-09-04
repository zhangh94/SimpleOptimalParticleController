import numpy as np


class RealWorld:
    def __init__(self,x0Dev):
        self.x0 = np.random.normal(0,x0Dev,1) # generate initial position [m]
        self.x  = np.array([self.x0]);
        # print("Initial Position: ", self.x0)

    def Integrate(self,uInput,dt):

        # First Order/Euler Method Integration
        self.x  = self.x + uInput * dt
        return self.x
