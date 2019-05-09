import os
import time
import rpyc

import config
import kinect
import rpcSend

watchInterval = 5


class rpcListener(rpyc.Service):

    ############################## common routines for servers
    def on_connect(self, conn):
        config.log(f"on_connect in server seen {conn}")
        callerName = conn._channel.stream.sock.getpeername()
        config.log(f"caller: {callerName}")


    def on_disconnect(self, conn):
        config.log(f"on_disconnect was triggered {conn}")


    def exposed_requestForReplyConnection(self, ip, port, messages=[], interval=5):

        messageList = list(messages)
        config.log(f"request for reply connection received from {ip}:{port}, messageList: {messageList}")
        myConfig = {"allow_all_attrs": True, "allow_pickle": True}
        try:
            replyConn = rpyc.connect(ip, port)
            config.log(f"reply connection established")
        except Exception as e:
            config.log(f"failed to open a reply connection, {e}")
            return

        clientId = (ip, port)
        connectionUpdated = False
        for c in config.clientList:
            if c['clientId'] == clientId:
                config.log(f"update client connection")
                c['replyConn'] = replyConn
                connectionUpdated = True

        if not connectionUpdated:
            config.log(f"append client connection {clientId}")
            config.clientList.append({'clientId': clientId,
                                      'replyConn': replyConn,
                                      'lastMessageReceivedTime': time.time(),
                                      'messageList': messageList,
                                      'interval': interval})

        # if kinect is already running send a ready message
        if config.kinectReady:
            rpcSend.publishServerReady()
        else:
            rpcSend.publishLifeSignal()


    def exposed_requestLifeSignal(self, ip, port):

        #config.log(f"life signal request received", publish=False)
        for c in config.clientList:
            if c['clientId'] == (ip, port):
                # print(f"life signal received from  {ip}, {port}, {time.time()}")
                c['lastMessageReceivedTime'] = time.time()
        rpcSend.publishLifeSignal()


    def exposed_terminate(self):
        print(f"{config.MY_NAME} task - terminate request received")
        os._exit(0)
        return True
    ############################## common routines for servers


    def exposed_getDepth(self, orientation, tableHeight = 920):

        config.log(f"getDepth request received, orientation: {orientation}, tableHeight: {tableHeight}")
        obstacles = kinect.obstacleMap(orientation, tableHeight)
        config.log(f"return obstacles, None: {obstacles is None}")
        return obstacles


    def exposed_startMonitoring(self):
        """
        for forward move return flag for cart obstacle detected
        :return:
        """
        config.log(f"start monitoring received")
        config.obstacleCheckActive = True


    def exposed_stopMonitoring(self):
        config.log(f"stop monitoring received")
        config.obstacleCheckActive = False


    def exposed_kinectDeactivate(self):
        print(f"kinect - kinectDeactivate request received, no action yet")




