
import matplotlib.pyplot as plt

class FootStepPlanner:
    SupportPositionsX  = []
    SupportPositionsY  = []
    RFootx =  []
    RFooty =  []
    
    LFootx =  []
    LFooty =  []
    

    def __init__(self,FR0X=0,FR0Y=0,FL0X=0,FL0Y=0.1,DistanceBetweenFeet=0.2):
        self.FR0X = FR0X
        self.FR0Y = FR0Y
        self.FL0X = FL0X
        self.FL0Y = FL0Y
        self.DistanceBetweenFeet = DistanceBetweenFeet
        

    def PlanSteps(self,NumberOfStep,StepX,StepY,FirstStepIsRight):
        SupportPositionsX = []
        SupportPositionsY = []

        RFootx =  [self.FR0X]
        RFooty =  [self.FR0Y]

        LFootx =  [self.FL0X]
        LFooty =  [self.FL0Y]

        if (FirstStepIsRight):
            RightIsSupport = 0       
            LeftIsSupport = 1        
        else:
            RightIsSupport = 1       
            LeftIsSupport = 0        
    
        for i in range(1,NumberOfStep+1):
            if (RightIsSupport == 1):
                    SupportPositionsX.append(RFootx[len(RFootx)-1])
                    SupportPositionsY.append(RFooty[-1])
                    LFootx.append(LFootx[len(LFootx)-1]+StepX)
                    LFooty.append(LFooty[len(LFooty)-1]+StepY)    
                    
                    RightIsSupport = 0
                    LeftIsSupport = 1

            elif (LeftIsSupport == 1):
                    #print LFootx[len(LFootx)-1]
                    SupportPositionsX.append(LFootx[len(LFootx)-1])
                    SupportPositionsY.append(LFooty[len(LFootx)-1])
                    RFootx.append(RFootx[len(RFootx)-1]+StepX)
                    RFooty.append(RFooty[len(RFooty)-1]+StepY)        

                    RightIsSupport = 1
                    LeftIsSupport = 0


        if (RightIsSupport == 1):
                SupportPositionsX.append(RFootx[len(RFootx)-1])
                SupportPositionsY.append(RFooty[len(RFooty)-1])    

                LFootx.append(LFootx[len(LFootx)-1]+StepX/2)
                LFooty.append(LFooty[len(LFooty)-1]+StepY/2)    

                SupportPositionsX.append(LFootx[len(LFootx)-1])
                SupportPositionsY.append(LFooty[len(LFooty)-1])
                        
                RightIsSupport = 1
                LeftIsSupport = 1

        elif (LeftIsSupport == 1):
                SupportPositionsX.append(LFootx[len(LFootx)-1])
                SupportPositionsY.append(LFooty[len(LFooty)-1])
                RFootx.append(RFootx[len(RFootx)-1]+StepX/2)
                RFooty.append(RFooty[len(RFooty)-1]+StepY/2)        
                
                SupportPositionsX.append( RFootx[len(RFootx)-1])
                SupportPositionsY.append(RFooty[len(RFooty)-1])   

                RightIsSupport = 1
                LeftIsSupport = 1
        self.SupportPositionsX = SupportPositionsX
        self.SupportPositionsY = SupportPositionsY
        self.RFootx =  RFootx
        self.RFooty =  RFooty    
        self.LFootx =  LFootx
        self.LFooty =  LFooty


#
#DistanceBetweenFeet=0.2
#NumberOfStep=5
#StepX=0.1
#StepY=0
#FirstStepIsRight=1
#
#FR0X=-StepX/2
#FR0Y=DistanceBetweenFeet/2
#FL0X=0
#FL0Y=-DistanceBetweenFeet/2
#
#
#FP = FootStepPlanner(FR0X,FR0Y,FL0X,FL0Y,DistanceBetweenFeet)
#FP.PlanSteps(NumberOfStep,StepX,StepY,FirstStepIsRight)
##print "X:",FP.SupportPositionsX,"\nY:",FP.SupportPositionsY
#
#
#fig = plt.figure()
#plt.scatter(FP.SupportPositionsX,FP.SupportPositionsY,label='FootSteps',marker = '*',color = 'darkgreen',linewidth=3)
#plt.legend()
#plt.hold()
#plt.plot(FP.SupportPositionsX,FP.SupportPositionsY,label='FootSteps',color = 'darkred',linewidth=1)

#
#ax = fig.add_subplot(111)
#ax.plot([1, 2, 3, 4], [10, 20, 25, 30], color='lightblue', linewidth=3)
#ax.scatter([0.3, 3.8, 1.2, 2.5], [11, 25, 9, 26], color='darkgreen', marker='^')
#ax.set_xlim(0.5, 4.5)
#plt.show()
