import sim     # access CoppeliaSim elements
import numpy as np
import math
import time

def setJointPosition(clientID, thetas, jointHandle):
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

def readCupSensor(clientID, handle):
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, handle, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cups's sensor")
        stopSim(clientID)
    return detectionState

def stopSim(clientID):
    time.sleep(2)
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxGetPingTime(clientID)  # guarantees that last command sent out has time to arrive
    sim.simxFinish(clientID)  # Close the connection to V-REP
    print("Simulation ended")

def shoot(clientID, baseAngle, ballVelocity, cups_left):
    # LABEL HANDLES FOR CONTROL -----------------------
    # Get handle for ping pong ball
    error_code, ballHandle = sim.simxGetObjectHandle(clientID, 'Ping_Pong', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for ping pong')
        stopSim(clientID)

    jointHandle = np.zeros((6, 1))  # creates array of zeros, 1 row, 6 columns
    error_code, baseHandle = sim.simxGetObjectHandle(clientID, 'UR3_link1_visible',
                                                     sim.simx_opmode_blocking)  # Get handle for base of robot
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for base frame')
        stopSim(clientID)

    # Get handle for all joints of robot
    error_code, jointHandle[0] = sim.simxGetObjectHandle(clientID, 'UR3_joint1', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for first joint')
        stopSim(clientID)
    error_code, jointHandle[1] = sim.simxGetObjectHandle(clientID, 'UR3_joint2', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for second joint')
        stopSim(clientID)
    error_code, jointHandle[2] = sim.simxGetObjectHandle(clientID, 'UR3_joint3', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for third joint')
        stopSim(clientID)
    error_code, jointHandle[3] = sim.simxGetObjectHandle(clientID, 'UR3_joint4', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for fourth joint')
        stopSim(clientID)
    error_code, jointHandle[4] = sim.simxGetObjectHandle(clientID, 'UR3_joint5', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for fifth joint')
        stopSim(clientID)
    error_code, jointHandle[5] = sim.simxGetObjectHandle(clientID, 'UR3_joint6', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for sixth joint')
        stopSim(clientID)

    # Get handle for end-effector of robot
    error_code, endHandle = sim.simxGetObjectHandle(clientID, 'UR3_link7_visible', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception('could not get object handle for end effector')
        stopSim(clientID)

    # Get gripper handles
    error_code, gripAttachPoint = sim.simxGetObjectHandle(clientID, 'RG2_attachPoint', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for gripper's attach point")
        stopSim(clientID)

    # REMOVE CUPS MADE ------------------------------
    for i in range(0, 10):
        if cups_left[i] == 0:
            if i == 0: handle = "Cup11"
            elif i == 1: handle = "Cup21"
            elif i == 2: handle = "Cup22"
            elif i == 3: handle = "Cup31"
            elif i == 4: handle = "Cup32"
            elif i == 5: handle = "Cup33"
            elif i == 6: handle = "Cup41"
            elif i == 7: handle = "Cup42"
            elif i == 8: handle = "Cup43"
            elif i == 9: handle = "Cup44"
            else: handle = ""
            error_code, cup = sim.simxGetObjectHandle(clientID, str(handle), sim.simx_opmode_blocking)
            if error_code != sim.simx_return_ok:
                raise Exception('could not get object handle for the cup to remove')
                stopSim(clientID)
            error_code, cupPos = sim.simxGetObjectPosition(clientID, cup, -1, sim.simx_opmode_oneshot)
            sim.simxSetObjectPosition(clientID, cup, -1, [cupPos[0], cupPos[1], 0.075], sim.simx_opmode_oneshot)

    # MOVE ARM TO GRAB BALL --------------------------
    liftBallPosition = [math.radians(-90), math.radians(-35), math.radians(-80), math.radians(35), math.radians(90),
                        math.radians(90)]
    error_code = sim.simxSetJointTargetVelocity(clientID, gripAttachPoint, 0, sim.simx_opmode_streaming)
    time.sleep(.5)
    error_code = sim.simxSetJointTargetVelocity(clientID, gripAttachPoint, 0, sim.simx_opmode_streaming)
    time.sleep(.5)
    setJointPosition(clientID, liftBallPosition, jointHandle)

    # GRAB BALL --------------------------------------
    # sim.simxSetObjectParent(clientID, ballHandle, gripAttachPoint, True, sim.simx_opmode_oneshot)  # makes the gripper the parent object to the ball (fakes picking it up)
    error_code = sim.simxSetJointTargetVelocity(clientID, gripAttachPoint, 0, sim.simx_opmode_streaming)
    time.sleep(.5)
    error_code = sim.simxSetJointTargetVelocity(clientID, gripAttachPoint, 0, sim.simx_opmode_streaming)
    time.sleep(.5)
    sim.simxSetObjectParent(clientID, ballHandle, gripAttachPoint, False, sim.simx_opmode_oneshot)

    # RETURN TO UPRIGHT POSITION --------------------
    throwPosition = [math.radians(-90), math.radians(0), math.radians(45), math.radians(45), math.radians(90),
                     math.radians(90)]
    setJointPosition(clientID, throwPosition, jointHandle)
    # THROW BALL -----------------------------------
    sim.simxSetJointTargetPosition(clientID, jointHandle[0], math.radians(baseAngle),
                                   sim.simx_opmode_oneshot)  # turn base
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[3], math.radians(80), sim.simx_opmode_oneshot)  # wrist back
    time.sleep(.5)
    sim.simxSetJointTargetPosition(clientID, jointHandle[3], math.radians(-5), sim.simx_opmode_oneshot)  # wrist forward
    time.sleep(.5)
    error_code, pos = sim.simxGetObjectPosition(clientID, ballHandle, -1,
                                                sim.simx_opmode_blocking)  # gets position of ball in world frame just before assigning velocity

    sim.simxPauseCommunication(clientID, True)  # pauses communication so we can send multiple pieces of data at once
    sim.simxSetObjectParent(clientID, ballHandle, -1, True, sim.simx_opmode_oneshot)  # disconnect ball from parent
    sim.simxSetObjectPosition(clientID, ballHandle, -1, pos,
                              sim.simx_opmode_oneshot)  # "resets" ping pong to given location in the world frame (-1)
    sim.simxSetObjectFloatParameter(clientID, ballHandle, 3000, ballVelocity,
                                    sim.simx_opmode_oneshot)  # throws ball with given velocity in the x direction
    sim.simxPauseCommunication(clientID, False)  # sends data again
    time.sleep(1)

    # INITIALIZE AND READ CUP SENSORS -------------
    # Initialize cup's sensor, doing this earlier causes a notable decrease in accuracy for some reason
    error_code, cupSensor11 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor11', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup11 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor11, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup11 sensor")
    landedIn11 = readCupSensor(clientID, cupSensor11)  # returns detection state of prox sensor11

    # Initialize cup21's sensor
    error_code, cupSensor21 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor21', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup21 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor21, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup21 sensor")
    landedIn21 = readCupSensor(clientID, cupSensor21)  # returns detection state of prox sensor

    # Initialize cup22's sensor
    error_code, cupSensor22 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor22', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup22 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor22, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup22 sensor")
    landedIn22 = readCupSensor(clientID, cupSensor22)  # returns detection state of prox sensor

    # Initialize cup31's sensor
    error_code, cupSensor31 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor31', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup31 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor31, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup31 sensor")
    landedIn31 = readCupSensor(clientID, cupSensor31)  # returns detection state of prox sensor

    # Initialize cup32's sensor
    error_code, cupSensor32 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor32', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup32 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor32, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup32 sensor")
    landedIn32 = readCupSensor(clientID, cupSensor32)  # returns detection state of prox sensor

    # Initialize cup33's sensor
    error_code, cupSensor33 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor33', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup33 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor33, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup33 sensor")
    landedIn33 = readCupSensor(clientID, cupSensor33)  # returns detection state of prox sensor

    # Initialize cup41's sensor
    error_code, cupSensor41 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor41', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup41 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor41, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup41 sensor")
    landedIn41 = readCupSensor(clientID, cupSensor41)  # returns detection state of prox sensor

    # Initialize cup42's sensor
    error_code, cupSensor42 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor42', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup42 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor42, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup42 sensor")
    landedIn42 = readCupSensor(clientID, cupSensor42)  # returns detection state of prox sensor

    # Initialize cup43's sensor
    error_code, cupSensor43 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor43', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup43 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor43, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup43 sensor")
    landedIn43 = readCupSensor(clientID, cupSensor43)  # returns detection state of prox sensor

    # Initialize cup44's sensor
    error_code, cupSensor44 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor44', sim.simx_opmode_blocking)
    if error_code != sim.simx_return_ok:
        raise Exception("could not get object handle for cup44 sensor")
    error_code, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
        clientID, cupSensor44, sim.simx_opmode_blocking)
    time.sleep(1)
    if error_code != sim.simx_return_ok:
        raise Exception("could not read cup44 sensor")
    landedIn44 = readCupSensor(clientID, cupSensor44)  # returns detection state of prox sensor

    isItInYet = [landedIn11, landedIn21, landedIn22, landedIn31, landedIn32, landedIn33, landedIn41, landedIn42, landedIn43, landedIn44]

    stopSim(clientID)
    return isItInYet
