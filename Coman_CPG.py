import numpy as np
import scipy.sparse as sparse
from Walking import GenrateWalkingTrajectories as WG
from Agents import ComanAgent, TalosAgent


if __name__ == '__main__':
    robot = ComanAgent.Coman()
    
    robot.reset()
     
    robot.StepX = 0.05
    robot.StepY = 0.0
    robot.StepTheta = 0.0

    robot.StepTime = 0.6

    robot.update_time_param()
    robot.update_move_param()


    t  = 0
    t0 = 0
    Lposes, Rposes = robot.generate_cpg(t)
    robot.step(np.zeros(20))
   
    
    tsim = [t]
    for sim_idx in range(80000):
        t += robot.SamplingTime
        robot.step(np.zeros(20))
        

