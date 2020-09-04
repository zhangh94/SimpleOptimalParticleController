class Controller:
    def __init__(self,Klqr):
        self.K  = Klqr
        self.uCmd = 0.0
        # print("LQR Initialized with K = ", self.K)

    def Control(self,xHat, cmd):

        ctrlErr     = cmd - xHat
        self.uCmd   = self.K * ctrlErr
        return self.uCmd
