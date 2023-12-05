import matplotlib.pyplot as plt
import numpy as np


def divRoundClosest(n,d):
    if (n<0 and d>0) or (n>0 and d<0):
        return (n-d//2)//d
    else:
        return (n+d//2)//d


a = 2000
vconst = 8000
stageTimeStep = 0.02
ticksPerSecond = 2000000


accelerationSteps = np.array(range(16001))
constantSteps = np.array(range(1,20001))
decelerationSteps = np.array(range(1,16001))
steps = np.concatenate((accelerationSteps, accelerationSteps[-1]+constantSteps, accelerationSteps[-1]+constantSteps[-1]+decelerationSteps))
taccel = np.sqrt(2*accelerationSteps/a)
tconstant = taccel[-1]+constantSteps/vconst
tdecel = tconstant[-1] + 1/a*(vconst-np.sqrt(vconst**2-a*decelerationSteps*2))
t = np.concatenate((taccel, tconstant, tdecel))

ticksPerStep = np.zeros(52000)


initialTimes = t[0:10]
ticksPerStep[0:9] = np.around(np.diff((initialTimes))*ticksPerSecond, 0)
finalTimes = t[-100:]
ticksPerStep[-99:] = np.around(np.diff((finalTimes))*ticksPerSecond, 0)
print(ticksPerStep[-99:])
queueEndTime = initialTimes[-1]
currentState = 1
remainingSteps = 52000
performedAccelSteps = len(initialTimes)-1
i=0
while remainingSteps>len(finalTimes)-1 :    
    if currentState == 1:
        if queueEndTime+stageTimeStep <= taccel[-1]:
            nextQueueEndTime = queueEndTime+stageTimeStep
        else:
            nextQueueEndTime = taccel[-1]
            performedSteps = len(accelerationSteps)-1
            currentState = 2
        planningSteps = int(a*nextQueueEndTime*nextQueueEndTime//2-performedAccelSteps)
        print(planningSteps)
        meanSpeed = a*(queueEndTime+nextQueueEndTime)//2
        ticksPerStep[performedAccelSteps : performedAccelSteps + planningSteps]  = divRoundClosest(ticksPerSecond,meanSpeed)
        performedAccelSteps = performedAccelSteps + planningSteps
        #print(performedAccelSteps)
        remainingSteps = remainingSteps - planningSteps
        queueEndTime = nextQueueEndTime
        i=i+1
    elif currentState == 2:
        if remainingSteps > len(decelerationSteps):
            pass
        else:
            currentState = 3
            queueEndTime = 0
            performedDecelSteps = 0
        planningSteps = int( min(vconst*stageTimeStep, remainingSteps - len(decelerationSteps)))
        ticksPerStep[performedSteps : performedSteps + planningSteps] = divRoundClosest(ticksPerSecond,vconst)
        performedSteps = performedSteps + planningSteps
        remainingSteps = remainingSteps - planningSteps
        


    elif currentState == 3:
        nextQueueEndTime = queueEndTime+stageTimeStep
        planningSteps = int(vconst*nextQueueEndTime - a*nextQueueEndTime*nextQueueEndTime//2)-performedDecelSteps
        planningSteps = min(planningSteps, remainingSteps-(len(finalTimes)-1))
        print(planningSteps)
        meanSpeed = vconst - a*(queueEndTime+nextQueueEndTime)//2
        ticksPerStep[performedSteps : performedSteps + planningSteps]  = divRoundClosest(ticksPerSecond,meanSpeed)
        performedSteps = performedSteps + planningSteps
        performedDecelSteps = performedDecelSteps + planningSteps
        #print(performedAccelSteps)
        remainingSteps = remainingSteps - planningSteps
        queueEndTime = nextQueueEndTime

print(remainingSteps)
ticksPerStep[-99:] = np.around(np.diff((finalTimes))*ticksPerSecond, 0)     
print(i)
print(len(ticksPerStep))
print(len(steps))
print(max(ticksPerStep))
print(min(ticksPerStep[0:16000]))
plt.scatter(t, steps, facecolors='none', edgecolors='b')
plt.scatter(np.cumsum(ticksPerStep)/ticksPerSecond,steps[1:], facecolors='none', edgecolors='r')
plt.figure()
plt.plot(np.cumsum(ticksPerStep)/ticksPerSecond, ticksPerSecond/ticksPerStep)
print(np.where(ticksPerStep == 0))
plt.show()