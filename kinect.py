
import os
import time

from primesense import openni2
from primesense import _openni2 as c_api
import numpy as np
import threading
import logging
from rpyc.utils.server import ThreadedServer

import config
import rpcReceive
import rpcSend

from scipy.signal import convolve2d

DEPTH_PORT = 20002

KINECT_MIN_DISTANCE = 800
KINECT_MAX_DISTANCE = 4500
KINECT_VERTICAL_RANGE_DEG = 47  # kinect vertical view angle
KINECT_HORIZONTAL_RANGE_DEG = 57
KINECT_MOUNT_HEIGHT = 290       # distance above table
KINECT_TO_HEAD = 550                  # obstacles above this height can be ignored
KINECT_VERTICAL_MOUNT_ANGLE_DEG = -3.5
DEPTH_ROWS = 480
DEPTH_COLS = 640
ROBOT_WIDTH = 600                   # mm
OFFSET_KINECT_FROM_CART_CENTER = 200    # Kinect from is not at cart rotation center, offset in mm

NUM_FLOOR_ROWS = 150
NUM_HEAD_ROWS = 150
FLOOR_RANGE = 70     # range in mm above/below calculated floor to be considered as floor

# Each row in the depth data represents a vertical angle range
topAngle = KINECT_VERTICAL_RANGE_DEG/2 + KINECT_VERTICAL_MOUNT_ANGLE_DEG
bottomAngle = KINECT_VERTICAL_RANGE_DEG/2 - KINECT_VERTICAL_MOUNT_ANGLE_DEG
rowDeg = KINECT_VERTICAL_RANGE_DEG / DEPTH_ROWS
rowRad = np.radians(rowDeg)
colDeg = KINECT_HORIZONTAL_RANGE_DEG / DEPTH_COLS

navManager = None


def openKinectStream():
    """
    it takes around 1.5s to open the depth stream so it's better to reuse an open stream
    :return:
    """

    if not config.openKinectFailureLogged:
        config.log(f"try to load openni2 driver", publish=False)
    try:
        openni2.initialize("C:/Program Files (x86)/OpenNI2/Tools/")
        config.dev = openni2.Device.open_any()

        #if standAlone:
        #    print(dev.get_sensor_info(c_api.OniSensorType.ONI_SENSOR_COLOR))

        config.depth_stream = config.dev.create_depth_stream()
        if config.depth_stream is None:
            if not config.openKinectFailureLogged:
                config.log(f"could not acquire depth_stream", publish=False)
            return False

        config.depth_stream.start()
        config.depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM, resolutionX=640, resolutionY=480, fps=30))
        config.openKinectFailureLogged = False
        config.log(f"depth stream startet")
        return True

    except Exception as e:
        if not config.openKinectFailureLogged:
            config.log(f"openKinectStream failed, {e}", publish=not config.openKinectFailureLogged)
            config.log(f"in case 12 V is available check for 'Kinect for Windows' in DeviceManager, should show 4 subentries", publish = not config.openKinectFailureLogged)
            config.openKinectFailureLogged = True
            closeKinectStream()
        return False


def kinectInit():

    #arch = int(platform.architecture()[0].lower().replace("bit", ""))
    #print(arch)
    if not config.openKinectFailureLogged:
        config.log(f"kinectInit, try to connect", publish=False)
    # try to capture the depth data
    if openKinectStream():

        config.log(f"kinect stream opened, try to read frame, depth_stream: {config.depth_stream}", publish=False)
        frame = config.depth_stream.read_frame()
        if frame is None:
            config.log(f"could not read a depth frame", publish=False)
            os._exit(2)
        else:
            config.log(f"depth data successfully captured", publish=False)
            return True
    else:
        return False


def closeKinectStream():
    if config.depth_stream is not None:
        config.depth_stream.stop()
    openni2.unload()
    config.openKinectFailureLogged = False
    config.dev = None
    config.kinectReady = False
    config.log(f"kinect depth stream stopped and openni unloaded")



def removeUpperRange(depth):
    '''
    based on distance from kinect to head remove obstacle points above Marvin's head
    calculate for each row of the depth array the "ceiling distance"
    remove points exceeding this distance (irrelevant for movements)
    '''
    # for each row (angle) the distance to the ceiling
    distanceToCeiling = np.zeros(NUM_HEAD_ROWS)

    # for upper kinect rows the distance to the ceiling, distance values greater than this can be removed
    angle = topAngle
    for i in range(NUM_HEAD_ROWS):
        distanceToCeiling[i] = KINECT_TO_HEAD / np.tan(np.radians(angle))
        angle -= rowDeg

    # remove points above Marvins head
    showResults = False
    if showResults:
        for row in range(NUM_HEAD_ROWS):
            numCleared = 0
            for col in range(0, DEPTH_COLS):
                if not np.isnan(depth[row, col]):
                    #print(f"row: {row}, col: {col}, depth[row,col]: {depth[row,col]}, distanceToFloor[row]: {distanceToFloor[row]}")
                    if depth[row, col] >= distanceToCeiling[row]:
                        if showResults:
                            numCleared += 1
                        else:
                            depth[row, col] = np.NaN
            if showResults:
                config.log(f"row: {row}, above head points: {numCleared}, ceiling: {distanceToCeiling[row]}")
    else:
        for row in range(NUM_HEAD_ROWS):
            depth[row, depth[row] > distanceToCeiling[row]] = np.NaN



def removeFloor(depth, kinectHeight = 1180):
    '''
    based on mount height and mount angle of kinect provide the floor distance for each row of the depth map
    filter out points in this distance range (abyss will become an obstacle)
    depth value is z value in point cloud
    '''
    zUpper = np.zeros(NUM_FLOOR_ROWS)      # initialize
    zLower = np.zeros(NUM_FLOOR_ROWS)  # initialize

    # for each column the cosine
    colCos = np.zeros(DEPTH_COLS)
    angle = KINECT_HORIZONTAL_RANGE_DEG / 2
    for i in range(len(colCos)):
        colCos[i] = np.cos(np.radians(angle))
        angle -= colDeg

    # for each row in the depth data the distance to the floor
    angle = bottomAngle
    y1 = kinectHeight - FLOOR_RANGE
    y2 = kinectHeight + FLOOR_RANGE
    for i in range(NUM_FLOOR_ROWS):
        tanAngle = np.tan(np.radians(angle))
        zUpper[i] = y1 / tanAngle       # adj = opp/tan
        zLower[i] = y2 / tanAngle
        angle -= rowDeg

    # remove floor
    showResults = False
    if showResults:
        for i in range(NUM_FLOOR_ROWS):

            subFloorDistanceCount = 0
            floorPoints = 0
            numNans = 0
            numObs = 0

            row = DEPTH_ROWS -1 - i       # bottom row has index 479
            #for col in range(0, DEPTH_COLS):
            for col in range(1, DEPTH_COLS-1):

                newDepth = depth[row, col]

                if np.isnan(depth[row, col]):
                    numNans += 1

                if depth[row, col] > zLower[i]:
                    newDepth = zUpper[i] # limit distance to floor to upper floor for points below floor
                    subFloorDistanceCount += 1

                else:
                    # remove floor distance value if in floor range
                    if depth[row, col] > zUpper[i] and depth[row, col] < zLower[i]:
                        newDepth = np.NaN
                        floorPoints += 1

                    if depth[row, col] < zUpper[i]:
                        numObs += 1

                depth[row, col] = newDepth

            print(f"row: {row}, floor: {floorPoints}, sub-floor: {subFloorDistanceCount}, nan: {numNans}, obstacle: {numObs}, min: {np.nanmin(depth[row]):.0f}, max: {np.nanmax(depth[row]):.0f}, floor: {zUpper[i]:.0f}-{zLower[i]:.0f}")

    else:
        for i in range(NUM_FLOOR_ROWS):
            row = DEPTH_ROWS - 1 - i
            depth[row, depth[row] > zLower[i]] = zUpper[i]
            depth[row, ((depth[row] > zUpper[i]) * (depth[row] < zLower[i]))] = np.NaN        # * has function of "and" here


def obstacleMap(orientation, tableHeight=920):
    '''
    take a depth picture from the kinect
    floor and ceiling are not considered obstacles
    for each column take the closest point to create a top view of obstacles
    return for each column (640) the closest distance
    '''

    config.log(f"obstacleMap, capture depth frame", publish=False)
    numRetries = 0
    while True:
        try:    # capture a new image, do not use a buffered one
            _ = config.depth_stream.read_frame()
            time.sleep(0.05)
            depthFrame = config.depth_stream.read_frame()
            config.log(f"depth data read done, frame captured: {depthFrame is not None}")
            break
        except Exception as e:
            config.log(f"exception reading frame, {e}")
            depthFrame = None
            numRetries += 1
            if numRetries < 2:
                closeKinectStream()
                if not kinectInit():
                    os._exit(0)

    config.log(f"eval closest point per column", publish=False)
    frame_data = depthFrame.get_buffer_as_uint16()
    arr1d = np.frombuffer(frame_data, dtype=np.int16)
    depth = arr1d.astype(float)
    depth.shape = (480, 640)

    # this might create NaN rows, ignore the warning output
    np.warnings.filterwarnings('ignore')
    depth[depth < KINECT_MIN_DISTANCE] = np.NaN
    depth[depth > KINECT_MAX_DISTANCE] = np.NaN

    logMinColValues = False
    if logMinColValues:
        depthColMinMax = np.zeros((3, DEPTH_COLS, 2))
        for i in range(DEPTH_COLS):
            try:
                depthColMinMax[0,i, 0] = np.nanmin(depth[:,i])
                depthColMinMax[0, i, 1] = np.nanargmin(depth[:,i])
            except:
                pass

    config.log(f"remove upper range", publish=False)
    removeUpperRange(depth)
    if logMinColValues:
        for i in range(DEPTH_COLS):
            try:
                depthColMinMax[1,i, 0] = np.nanmin(depth[:,i])
                depthColMinMax[1, i, 1] = np.nanargmin(depth[:,i])
            except:
                pass

    config.log(f"remove floor", publish=False)
    removeFloor(depth, tableHeight + KINECT_MOUNT_HEIGHT)
    if logMinColValues:
        for i in range(DEPTH_COLS):
            try:
                depthColMinMax[2,i, 0] = np.nanmin(depth[:,i])
                depthColMinMax[2, i, 1] = np.nanargmin(depth[:,i])
            except:
                pass

        for i in range(DEPTH_COLS):
            print(f"col {i} raw: {depthColMinMax[0,i,0]}, {depthColMinMax[0,i,1]}, removeUpperCeiling: {depthColMinMax[1,i,0]}, {depthColMinMax[1,i,1]}, removeFloor: {depthColMinMax[2,i,0]}, {depthColMinMax[2,i,1]}")

    # for each column the closest point
    obstacles = np.nanmin(depth, axis=0)
    config.log(f"return obstacle array (closest point in each col", publish=False)
    return obstacles   # for each screen column the closest obstacle between floor and ceiling



def cartObstacleDetected():
    '''
    take a depth picture from the kinect and check for free move of 1 m in the range of the forward moving cart
    return blocked/free
    '''
    #config.log(f"try to get depth frame, depth_stream: {config.depth_stream}")

    try:
        frame = config.depth_stream.read_frame()
    except Exception as e:
        config.log(f"could not get depth frame, {e}")
        closeKinectStream()
        return False, 0, 0, 0

    frame_data = frame.get_buffer_as_uint16()
    arr1d = np.frombuffer(frame_data, dtype=np.int16)

    depth = arr1d.astype(float)
    depth.shape = (480, 640)

    # this might create NaN rows, ignore the warning output
    np.warnings.filterwarnings('ignore')
    depth[depth < KINECT_MIN_DISTANCE] = np.NaN
    depth[depth > KINECT_MAX_DISTANCE] = np.NaN

    # smooth the values with moving average
    window = np.ones((5,5))
    window /= window.sum()
    smoothedDepth = convolve2d(depth, window, mode='same', boundary='symm')

    # for each column the closest point in the raw data
    minForEachCol = np.nanmin(depth, axis=0)
    minForEachRow = np.nanmin(depth, axis=1)
    closestPoint = np.nanmin(minForEachCol)

    # for each column the closest point in the smooted data
    minForEachCol = np.nanmin(smoothedDepth, axis=0)
    minForEachRow = np.nanmin(smoothedDepth, axis=1)
    closestPointSmoothed = np.nanmin(minForEachCol)

    #print(f"closestPointRaw: {closestPoint}, closestPointSmoothed: {closestPointSmoothed}")

    # eval horizontal angle to closest obstacle point
    col = np.nanargmin(minForEachCol)       # column of min value in array, col 0 is to the right
    row = np.nanargmin(minForEachRow)

    colAngle = KINECT_HORIZONTAL_RANGE_DEG/DEPTH_COLS     # angle of a single column
    xAngle = (col - (DEPTH_COLS/2))* colAngle

    rowAngle = KINECT_VERTICAL_RANGE_DEG/DEPTH_ROWS       # angle of a single row
    yAngle = (row - (DEPTH_ROWS/2)) * rowAngle

    config.log(f"closest point: {closestPointSmoothed:.0f}, col: {col}, xAngle: {xAngle:.2f}, row: {row}, yAngle: {yAngle:.2f}", publish=False)

    if closestPointSmoothed > KINECT_MIN_DISTANCE:

        # check for obstacle in path of robot (50 cm corridor)
        # we might later add constraints to robot pose and use narrower corridor.
        # robot body would need to rotate sideways to minimize shape
        if abs(np.tan(np.radians(xAngle)) * closestPointSmoothed) < (ROBOT_WIDTH/2):
            return True, closestPointSmoothed, xAngle, yAngle

    return False, 0, 0, 0



# runs in own thread monitorThread
def monitorObstacles():

    # try to get an image first (needs 12 V power)
    numRetries = 0
    while True:
        if not config.kinectReady:
            while True:
                if kinectInit():
                    config.log(f"kinect init successful")
                    break
                else:
                    if numRetries == 0:
                        config.log(f"waiting for 12V power")

                    if numRetries > 60:
                        config.log(f"waited 2 mins for power, going down now")
                        time.sleep(10)
                        os._exit(0)

                    numRetries += 1
                    time.sleep(2)

            config.kinectReady = True
            config.log(f"test image successfully taken, kinect ready")
            rpcSend.publishServerReady()

        while True:

            # if kinect is not ready stop monitoring
            if not config.kinectReady:
                break

            # check for monitoring active
            if config.obstacleCheckActive:

                obstacle, distance, xAngle, yAngle = cartObstacleDetected()
                if obstacle and distance < 1200:
                    rpcSend.publishObstacleDetected(distance, xAngle, yAngle)

                    # detecting an obstacle stops monitoring.
                    # the client needs to retrigger monitoring
                    config.obstacleCheckActive = False
            else:
                time.sleep(0.5)


if __name__ == '__main__':

    ##########################################################
    # initialization
    # Logging, renaming old logs for reviewing ...
    logAppend = True
    baseName = "log/kinect"
    fileMode = "a" if logAppend else "w"

    # if a new log is created with each start save the file history
    if not logAppend:

        oldName = f"{baseName}9.log"
        if os.path.isfile(oldName):
            os.remove(oldName)
        for i in reversed(range(9)):
            oldName = f"{baseName}{i}.log"
            newName = f"{baseName}{i+1}.log"
            if os.path.isfile(oldName):
                os.rename(oldName, newName)

        oldName = f"{baseName}.log"
        newName = f"{baseName}0.log"
        if os.path.isfile(oldName):
            try:
                os.rename(oldName, newName)
            except Exception as e:
                config.log(f"can not rename {oldName} to {newName}")

    logging.basicConfig(
        filename=f"{baseName}.log",
        level=logging.INFO,
        format='%(message)s',
        filemode=fileMode)

    config.log("kinect started")


    if config.standAlone:

        rpcSend.requestKinectPower(True)

        # try to grab an image
        kinectInit()
        tableHeight = 920    # 920 table height
        obstacles = obstacleMap(0,tableHeight)

        obstacle, distance, xAngle, yAngle = cartObstacleDetected()
        config.log(f"distance: {distance:.0f}, xAngle: {xAngle:.0f}, yAngle: {yAngle:.0f}")

        os._exit(5)


    # add thread to monitor distance during forward move of cart
    monitorThread = threading.Thread(target=monitorObstacles, args={})
    monitorThread.setName('monitorThread')
    monitorThread.start()

    # start rpc listener, does not return
    print(f"start listening on port {config.MY_RPC_PORT}")
    myConfig = {"allow_all_attrs": True, "allow_pickle": True}
    server = ThreadedServer(rpcReceive.rpcListener(), port=config.MY_RPC_PORT, protocol_config=myConfig)
    server.start()

    # code here never reached


