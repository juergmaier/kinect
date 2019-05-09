
import datetime
import logging

import rpcSend

dev = None
depth_stream = None
clientList = []
obstacleCheckActive = False
openKinectFailureLogged = False
kinectReady = False

######################################################
# this task can run as a slave (controlled by rpc commands) or standAlone
######################################################
standAlone = False

MY_IP = "192.168.0.17"
MY_RPC_PORT = 20003      # defined in taskOrchestrator
MY_NAME = 'kinect'

def log(msg, publish=True):
    logtime = str(datetime.datetime.now())[11:]
    logging.info(f"{logtime} - {msg}")
    print(f"{logtime} - {msg}")

    if publish:
        rpcSend.publishLog("kinect - " + msg)


