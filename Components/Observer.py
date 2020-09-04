import numpy as np


class Observer:
    def __init__(self,Kkf):
        # Parameters for dynamic gain Kf (not used/not needed)
        # self.R = measNoise**2           # measurement noise variance (for LQE/Kf) [m^2]
        # self.Q = acutNoise**2           # process noise variance (for LQE/Kf) [m^2]
        # self.P = x0Dev**2               # initial position state (co)variance [m^2]
        # self.H = np.array([[1.0]])      # observation matrix

        # Kalman Filter static gain
        self.xHat = np.array([[0.0]])   # state vector
        self.K  = Kkf

        # Kalman Filter State Space Model
        self.A  = np.array([[0-self.K]])
        self.B  = np.array([[1, self.K]])

    def Estimate(self,xMeas,uCmd,dt):

        # Kf - Dynamic Gains (Not used since static gain good enough here)
        # # Predict
        # phi         = self.A * dt
        # xHat        = phi * self.xHat + self.B * uCmd * dt
        # P           = phi * self.P * phi.T + self.Q
        #
        # # Update
        # K           = P * self.H.T / (self.H * P * self.H.T + self.R*dt)
        # y           = xMeas - self.H * xHat
        # self.xHat   = xHat + K * y
        # self.P      = (1 - K * self.H) * P

        # State Space form KF with Constant Gain
        x       = np.matmul(self.B, np.concatenate((uCmd,xMeas),axis=0)) + self.A * self.xHat
        self.xHat = self.xHat + x * dt

        return self.xHat
