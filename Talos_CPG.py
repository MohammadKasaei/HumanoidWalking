import numpy as np
import scipy.sparse as sparse
from Walking import GenrateWalkingTrajectories as WG
from Agents import ComanAgent, TalosAgent


if __name__ == '__main__':
   
    robot = TalosAgent.Talos()
    
    robot.reset()
     
    robot.StepX = 0.
    robot.StepY = 0.
    robot.StepTheta = 0.0
    robot.StepTime = 0.5

    robot.update_time_param()
    robot.update_move_param()

    # for i in range(10000):
    #     robot.motorTest(3,-i*0.0001)
    #     robot.motorTest(6,-i*0.001)
        
    t  = 0
    t0 = 0
    Lposes, Rposes = robot.generate_cpg(t)
    robot.step(np.zeros(20))
    
    tsim = [t]
    for sim_idx in range(80000):
        
        if (sim_idx<2000):
            robot.NewStepX_raw = 0.1
            robot.NewStepY_raw = 0.05
            robot.NewStepTheta_raw = -20 
            robot.NewStepTime = 1
        elif (sim_idx<4000):
            robot.NewStepX_raw = 0.1
            robot.NewStepY_raw = 0.05
            robot.NewStepTheta_raw = 20 
            robot.NewStepTime = 1
        elif (sim_idx<6000):
            robot.NewStepX_raw = 0.
            robot.NewStepY_raw = 0.1
            robot.NewStepTheta_raw = 0 
            robot.NewStepTime = 1
        elif (sim_idx<8000):
            robot.NewStepX_raw = 0.
            robot.NewStepY_raw = -0.1
            robot.NewStepTheta_raw = 0 
            robot.NewStepTime = 1
        elif (sim_idx<10000):
            robot.NewStepX_raw = 0.
            robot.NewStepY_raw = -0.
            robot.NewStepTheta_raw = 20 
            robot.NewStepTime = 1
        elif (sim_idx<14000):
            robot.NewStepX_raw = 0.2
            robot.NewStepY_raw = -0.
            robot.NewStepTheta_raw = 0 
            robot.NewStepTime = 0.8
        elif (sim_idx<16000):
            robot.NewStepX_raw = 0.25
            robot.NewStepY_raw = -0.
            robot.NewStepTheta_raw = 0 
            robot.NewStepTime = 1.2
        

        t += robot.SamplingTime
        robot.step(np.zeros(20))
        

