import numpy as np
import scipy.sparse as sparse
# from Walking import GenrateWalkingTrajectories as WG
from Agents import G1Agent


if __name__ == '__main__':
   
    robot = G1Agent.G1()
    
    robot.reset()
        

    robot.StepX = 0.0
    robot.StepY = 0.
    robot.SwingStepZ = 0.01
    robot.StepTheta = 0.0
    robot.StepTime = 0.4
    
    robot.update_time_param()
    robot.update_move_param()
    

    t  = 0
    
    tsim = [t]
    for sim_idx in range(80000):
        robot.NewStepX_raw = 0.
        robot.NewStepY_raw = 0.0
        
        if sim_idx<500:
            robot.SwingStepZ = 0.0
        else:
            robot.SwingStepZ = min(0.045, sim_idx*0.00001)
            if robot.SwingStepZ>0.043:
                t += 1
                if t<2000:
                    robot.NewStepX_raw = 0.1
                    robot.NewStepY_raw = -0.0
                    robot.NewStepTheta_raw = -0 
                elif t<4000:
                    robot.NewStepX_raw = 0.0
                    robot.NewStepY_raw = -0.0
                    robot.NewStepTheta_raw = -0
                elif t<6000:
                    robot.NewStepX_raw = -0.1
                    robot.NewStepY_raw = 0.0
                    robot.NewStepTheta_raw = -0
                elif t<8000:
                    robot.NewStepX_raw = 0.0
                    robot.NewStepY_raw = 0.0
                    robot.NewStepTheta_raw = -0
                elif t<11000:
                    robot.NewStepX_raw = 0.05
                    robot.NewStepY_raw = 0.025
                    robot.NewStepTheta_raw = 15
                elif t<12500:
                    robot.NewStepX_raw = 0.0
                    robot.NewStepY_raw = 0.0
                    robot.NewStepTheta_raw = 0
                elif t<16000:
                    robot.NewStepX_raw = 0.05
                    robot.NewStepY_raw = -0.025
                    robot.NewStepTheta_raw = -15
                elif t<1700:
                    robot.NewStepX_raw = 0.0
                    robot.NewStepY_raw = 0.0
                    robot.NewStepTheta_raw = 0
                elif t<19000:
                    robot.NewStepX_raw = 0.1
                    robot.NewStepY_raw = 0.0
                    robot.NewStepTheta_raw = 0
                
                else:
                    robot.NewStepX_raw = 0.0
                    robot.NewStepY_raw = 0.0
                    robot.NewStepTheta_raw = 0
                    
                    

        # t += robot.SamplingTime
        robot.step(np.zeros(20))
        

