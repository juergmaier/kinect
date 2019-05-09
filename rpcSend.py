
import config
import rpyc

def publishLog(msg):

    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:
            if len(c['messageList']) == 0 or 'log' in c['messageList']:
                try:
                    c['replyConn'].root.exposed_log(msg)

                except Exception as e:
                    print(f"exception in publishLog with {c['clientId']}: {e}")
                    c['replyConn'] = None


def publishLifeSignal():

    #config.log(f"publishing life signal")
    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:
            if len(c['messageList']) == 0 or 'lifeSignalUpdate' in c['messageList']:
                try:
                    c['replyConn'].root.exposed_lifeSignalUpdate(config.MY_NAME)
                except Exception as e:
                    c['replyConn'] = None
                    config.log(f"exception in publishLifeSignal with {c['clientId']}: {e}")


def publishObstacleDetected(distance, xAngle, yAngle):

    config.log(f"publishing obstacle detected {distance}, xAngle: {xAngle}, yAngle: {yAngle}")

    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:
            if len(c['messageList']) == 0 or 'obstacleUpdate' in c['messageList']:
                try:
                    c['replyConn'].root.exposed_obstacleUpdate(f"{distance},{xAngle},{yAngle}")
                except Exception as e:
                    c['replyConn'] = None
                    config.log(f"exception in publishObstacleDetected with {c['clientId']}: {e}")


def publishServerReady():

    config.log(f"publishing ready={config.kinectReady} message to clients")

    for i, c in enumerate(config.clientList):

        if c['replyConn'] is not None:
            if len(c['messageList']) == 0 or 'serverReady' in c['messageList']:
                try:
                    c['replyConn'].root.exposed_serverReady(f"kinect", config.kinectReady)
                except Exception as e:
                    c['replyConn'] = None
                    config.log(f"exception in publishServerReady with {c['clientId']}: {e}")


def requestKinectPower(newState):
    try:
        cart = rpyc.connect("127.0.0.1", 20001)
        cart.root.exposed_powerKinect(newState)
    except Exception as e:
        config.log(f"exception trying to get power from cart: {e}")