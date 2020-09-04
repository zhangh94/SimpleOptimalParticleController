import matplotlib.pyplot as plt

from Components.RealWorld import *
from Components.Actuator import *
from Components.Sensor import *
from Components.Observer import *
from Components.Controller import *


class Sim:
    def __init__(self, params, traj,Klqr,Kkf,label):
        self.simTime    = params['simTime']
        self.dt         = params['dt']
        self.params     = params
        self.traj       = traj
        self.Klqr       = Klqr
        self.Kkf        = Kkf
        self.label      = label

        # [time     xTruth  xMeas   xEst    xCmd    uInput  uCmd]
        self.log        = np.zeros((0, 7))

    def Run(self):
        nSteps = self.simTime/self.dt

        # Initialize Simulation Classes
        rw  = RealWorld(self.params['x0Dev'])
        a   = Actuator(self.params['acutNoise'])
        s   = Sensor(self.params['measNoise'])
        kf  = Observer(self.Kkf)
        lqr = Controller(self.Klqr)

        # Loop Through Simulation Time Steps
        cmdStart    = 0
        uCmd        = np.array([[0.0]])
        cmd         = 0.0

        for i1 in range(int(nSteps)):
            t   = i1*self.dt

            # Dont control if traj hasnt started
            if cmdStart == 0:
                cmdStart = t >= self.traj[0,0]

            # Measure and Estimate Position
            xMeas   = s.Measure(rw.x)
            xHat    = kf.Estimate(xMeas,uCmd,self.dt)

            if cmdStart:
                # Implement Control Law
                cmd     = np.interp(t,self.traj[0,:],self.traj[1,:])
                uCmd    = lqr.Control(xHat,cmd)
                uInput  = a.Actuate(uCmd)

                # Integrate Acutator Action
                rw.Integrate(uInput,self.dt)

            # log simulation data
            # [time     xTruth  xMeas   xEst    xCmd    uInput  uCmd]
            log = np.array([[t, rw.x, s.xMeas, kf.xHat, cmd, a.uInput, lqr.uCmd]])
            self.log = np.concatenate((self.log,log),axis=0)

    def Plot(self):
        # print("Plot Simulation Data")
        # [time     xTruth  xMeas   xEst    xCmd    uInput  uCmd]
        l = self.log
        t       = l[:,0]
        xTruth  = l[:,1]
        xMeas   = l[:,2]
        xEst    = l[:,3]
        xCmd    = l[:,4]
        uInput  = l[:,5]
        uCmd    = l[:,6]

        # Controller Performance
        ax11 = plt.subplot(321)
        fig     = plt.gcf()
        fig.suptitle(self.label,fontsize=8)
        plt.plot(t, xCmd, label='Pos Cmd')
        plt.plot(t, xTruth, label='Pos (Truth)')
        plt.plot(t, xEst, label='Pos (Estimated)')
        ax11.set(xlabel='Time [s]', ylabel='Position [m]')
        ax11.set_title('Ctrlr Performance (States)', fontsize = 8)
        ax11.legend()
        ax11.grid()

        ax12 = plt.subplot(322)
        plt.plot(t, xCmd-xTruth, label='Ctrl Err (vs. Truth)')
        plt.plot(t, xCmd-xEst, label='Ctrl Err (vs. Est)')
        ax12.set(xlabel='Time [s]', ylabel='Position [m]')
        ax12.set_title('Ctrlr Performance (Errs)', fontsize = 8)
        ax12.legend()
        ax12.grid()

        ax21 = plt.subplot(323)
        plt.plot(t, xMeas, label='Pos (Measured)')
        plt.plot(t, xTruth, label='Pos (Truth)')
        plt.plot(t, xEst, label='Pos (Estimated)')
        ax21.set(xlabel='Time [s]', ylabel='Position [m]')
        ax21.set_title('Observer Performance (States)', fontsize = 8)
        ax21.grid()
        ax21.legend()

        ax22 = plt.subplot(324)
        plt.plot(t, xTruth-xMeas, label='Pos Err (Sensor)')
        plt.plot(t, xTruth-xEst, label='Pos Err (LQE/Kf)')
        ax22.set(xlabel='Time [s]', ylabel='Position [m]')
        ax22.set_title('Observer Performance (Errs)', fontsize = 8)
        ax22.grid()
        ax22.legend()

        ax31 = plt.subplot(325)
        plt.plot(t, uCmd, label='Input (LQR Cmd)')
        plt.plot(t, uInput, label='Input (Acutator)')
        ax31.set(xlabel='Time [s]', ylabel='Input [m]')
        ax31.set_title('Actuator Performance (States)', fontsize = 8)
        ax31.grid()
        ax31.legend()

        ax32 = plt.subplot(326)
        plt.plot(t, uCmd-uInput)
        ax32.set(xlabel='Time [s]', ylabel='Input [m]')
        ax32.set_title('Actuator Performance (Err)', fontsize = 8)
        ax32.grid()
        ax32.legend()
        plt.tight_layout()

        plt.show()
