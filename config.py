
import datetime

import rpcSend

######################################################
# this task can run as a slave (controlled by rpc commands) or standAlone
######################################################
standAlone = False

MY_IP = "192.168.0.17"
MY_XMLRPC_PORT = 30003      # defined in taskOrchestrator

def log(msg):
    logtime = str(datetime.datetime.now())[11:]
    print(f"{logtime} - {msg}")

    rpcSend.publishLog("kinect - " + msg)


