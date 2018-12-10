import os
import time
import rpyc

import kinect

clientList = []
watchInterval = 5

server = 'kinect'


class rpcListener(rpyc.Service):

    ############################## common routines for servers
    def on_connect(self, conn):
        print(f"on_connect in server seen {conn}")
        callerName = conn._channel.stream.sock.getpeername()
        print(f"caller: {callerName}")

    def on_disconnect(self, conn):
        callerName = conn._channel.stream.sock.getpeername()
        print(f"{server} - on_disconnect triggered, conn: {callerName}")

    def exposed_requestForReplyConnection(self, ip, port, interval=5):

        print(f"request for reply connection {ip}:{port}")
        replyConn = rpyc.connect(ip, port)

        clientId = (ip, port)
        connectionUpdated = False
        for c in clientList:
            if c['clientId'] == clientId:
                print(f"update client connection")
                c['replyConn'] = replyConn
                connectionUpdated = True

        if not connectionUpdated:
            print(f"append client connection {clientId}")
            clientList.append({'clientId': clientId, 'replyConn': replyConn, 'lastMessageReceivedTime': time.time(), 'interval': interval})


    def exposed_getLifeSignal(self, ip, port):

        for c in clientList:
            if c['clientId'] == (ip, port):
                #print(f"life signal received from  {ip}, {port}, {time.time()}")
                c['lastMessageReceivedTime'] = time.time()

        return True

    def exposed_terminate(self):
        print(f"{server} task - terminate request received")
        os._exit(0)
        return True
    ############################## common routines for servers


    def exposed_getDepth(self, orientation):
        print("kinect - getDepth request received")
        return kinect.obstacleMap(orientation)


    def exposed_kinectDeactivate(self):
        print("kinect - kinectDeactivate request received")
        kinect.kinectDeactivate()




