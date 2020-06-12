import sim     # access CoppeliaSim elements
import numpy as np
import math
import time


def setJointPosition(thetas):
    # order of joint changes is currently optimized for not slapping the ball off the table
    #sim.simxPauseCommunication(clientID, True)      # pauses communication so we can send multiple pieces of data at once
    sim.simxSetJointTargetPosition(clientID, jointHandle[0], thetas[0], sim.simx_opmode_oneshot)
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[1], thetas[1], sim.simx_opmode_oneshot)
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[3], thetas[3], sim.simx_opmode_oneshot)
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[5], thetas[5], sim.simx_opmode_oneshot)
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[4], thetas[4], sim.simx_opmode_oneshot)
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[2], thetas[2], sim.simx_opmode_oneshot)
    #sim.simxPauseCommunication(clientID, False)  # sends data again
    time.sleep(.5)  # wait for robot to move

def moveAndLiftBall():
    # MOVE ARM TO GRAB BALL --------------------------
    liftBallPosition = [math.radians(-90), math.radians(-35), math.radians(-80), math.radians(35), math.radians(90), math.radians(90)]
    setJointPosition(liftBallPosition)

    # GRAB BALL --------------------------------------
    error_code = sim.simxSetJointTargetVelocity(clientID, gripMotorHandle, 0, sim.simx_opmode_streaming)
    time.sleep(.5)
    error_code = sim.simxSetJointTargetVelocity(clientID, gripMotorHandle, 0, sim.simx_opmode_streaming)
    time.sleep(.5)
    sim.simxSetObjectParent(clientID, ballHandle, gripAttachPoint, True, sim.simx_opmode_oneshot)  # makes the gripper the parent object to the ball (fakes picking it up)

    # RETURN TO UPRIGHT POSITION --------------------
    throwPosition = [math.radians(-90), math.radians(0), math.radians(45), math.radians(45), math.radians(90), math.radians(90)]
    setJointPosition(throwPosition)

# takes angle in degrees and velocity in m/s
# we might also have to change the angle that the wrist moves forward to so that the ball has a better arc
def throwBall(baseAngle, ballVelocity):
    sim.simxSetJointTargetPosition(clientID, jointHandle[0], math.radians(baseAngle), sim.simx_opmode_oneshot)  # turn base
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[3], math.radians(80), sim.simx_opmode_oneshot)  # wrist back
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[3], math.radians(-5), sim.simx_opmode_oneshot)  # wrist forward
    time.sleep(.5)
    error_code, pos = sim.simxGetObjectPosition(clientID, ballHandle, -1, sim.simx_opmode_blocking)  # gets position of ball in world frame just before assigning velocity

    sim.simxPauseCommunication(clientID, True)  # pauses communication so we can send multiple pieces of data at once
    sim.simxSetObjectParent(clientID, ballHandle, -1, True, sim.simx_opmode_oneshot)  # disconnect ball from parent
    sim.simxSetObjectPosition(clientID, ballHandle, -1, pos, sim.simx_opmode_oneshot)  # "resets" ping pong to given location in the world frame (-1)
    sim.simxSetObjectFloatParameter(clientID, ballHandle, 3000, ballVelocity, sim.simx_opmode_oneshot)  # throws ball with given velocity in the x direction
    sim.simxPauseCommunication(clientID, False)  # sends data again
    time.sleep(1)

def readCupSensor(handle):
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, handle, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cups's sensor")
        stopSim()
    return detectionState

def resetBallPos():
    sim.simxPauseCommunication(clientID, True)
    sim.simxSetObjectPosition(clientID, ballHandle, -1, [.46, .062, .72], sim.simx_opmode_oneshot)  # "resets" ping pong to start location in the world frame (-1)
    sim.simxSetObjectFloatParameter(clientID, ballHandle, 3000, 0, sim.simx_opmode_oneshot)  # sets ball x velocity to 0
    sim.simxSetObjectFloatParameter(clientID, ballHandle, 3001, 0, sim.simx_opmode_oneshot)  # sets ball y velocity to 0
    sim.simxSetObjectFloatParameter(clientID, ballHandle, 3002, 0, sim.simx_opmode_oneshot)  # sets ball z velocity to 0
    sim.simxSetObjectQuaternion(clientID, ballHandle, -1, [0, 0, 0, 0], sim.simx_opmode_oneshot)        # "resets" ping pong orientation
    sim.simxPauseCommunication(clientID, False)

def stopSim():
    time.sleep(2)
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxGetPingTime(clientID)  # guarantees that last command sent out has time to arrive
    sim.simxFinish(clientID)  # Close the connection to V-REP
    print("Simulation ended")


# SET UP CLIENT ----------------------------------
sim.simxFinish(-1)     # closes all opened connections
clientID=sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)    # starts a connection
if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")

# LABEL HANDLES FOR CONTROL -----------------------
# Get handle for ping pong ball
error_code, ballHandle = sim.simxGetObjectHandle(clientID, 'Ping_Pong', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for ping pong')
    stopSim()

jointHandle = np.zeros((6, 1))      # creates array of zeros, 1 row, 6 columns
error_code, baseHandle = sim.simxGetObjectHandle(clientID, 'UR3_link1_visible', sim.simx_opmode_blocking)   # Get handle for base of robot
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for base frame')
    stopSim()

# Get handle for all joints of robot
error_code, jointHandle[0] = sim.simxGetObjectHandle(clientID, 'UR3_joint1', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for first joint')
    stopSim()
error_code, jointHandle[1] = sim.simxGetObjectHandle(clientID, 'UR3_joint2', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for second joint')
    stopSim()
error_code, jointHandle[2] = sim.simxGetObjectHandle(clientID, 'UR3_joint3', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for third joint')
    stopSim()
error_code, jointHandle[3] = sim.simxGetObjectHandle(clientID, 'UR3_joint4', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for fourth joint')
    stopSim()
error_code, jointHandle[4] = sim.simxGetObjectHandle(clientID, 'UR3_joint5', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for fifth joint')
    stopSim()
error_code, jointHandle[5] = sim.simxGetObjectHandle(clientID, 'UR3_joint6', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for sixth joint')
    stopSim()

# Get handle for end-effector of robot
error_code, endHandle = sim.simxGetObjectHandle(clientID, 'UR3_link7_visible', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception('could not get object handle for end effector')
    stopSim()

# Get gripper handles
error_code, gripAttachPoint = sim.simxGetObjectHandle(clientID, 'RG2_attachPoint', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception("could not get object handle for gripper's attach point")
    stopSim()
error_code, gripMotorHandle = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception("could not get object handle for gripper's motor")
    stopSim()

# Initialize gripper's sensor
error_code, gripProxSensor = sim.simxGetObjectHandle(clientID, 'RG2_attachProxSensor', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception("could not get object handle for gripper's proximity sensor")
    stopSim()

# ACTIONS ---------------------------------------
moveAndLiftBall()
throwBall(-66.3, -5.35)     # find values so that the ball lands in the cup

# Initialize cup's sensor, doing this earlier causes a notable decrease in accuracy for some reason
error_code, cupSensor = sim.simxGetObjectHandle(clientID, 'Proximity_sensor0', sim.simx_opmode_blocking)
if error_code != sim.simx_return_ok:
    raise Exception("could not get object handle for cups's sensor")
error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, cupSensor, sim.simx_opmode_blocking)
time.sleep(1)
if error_code != sim.simx_return_ok:
    raise Exception("could not read cups's sensor")

isItInYet = readCupSensor(cupSensor)        # returns detection state of prox sensor
print("Is it in yet? ", isItInYet)

# STOP SIMULATION -------------------------------
stopSim()
