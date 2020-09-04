from Sim import *


# Simulation Parameters
params = {
    'simTime':  10.0,   # simulation time [s]
    'dt':       0.25,   # simulation time step [s]
    'traj':     np.array([[1,3,4,7],[1,3,2,5]]), # time vs. pos
    'measNoise':0.3,    # position measurement noise [m 1-sigma]
    'acutNoise':0.2,    # acutator noise [m 1-sigma]
    'x0Dev':    0.5     # devation in starting pos [m 1-sigma]
}

# Case 1 - Set high cost on position error
print('Running Case 1')
OptimizeErrs = Sim(params,params['traj'],5.07,0.006644,'Optimize Pos Err')
OptimizeErrs.Run()
OptimizeErrs.Plot()

# Case 2 - Set high cost on controller effort
print('Running Case 2')
OptimizeCtlr = Sim(params,params['traj'],0.8868,0.006644,'Optimize Ctrlr Effort')
OptimizeCtlr.Run()
OptimizeCtlr.Plot()

# Case 3 - Set balanced cost between position error and controller effort
print('Running Case 3 (last)')
Balanced = Sim(params,params['traj'],2.0112,0.006644,'Balance Err/Effort')
Balanced.Run()
Balanced.Plot()
