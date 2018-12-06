import os
import time
import rpyc

import kinect

clientList = []
watchInterval = 5

server = 'kinect'


def addClient(c, i):
    global clientList, watchInterval

    clientList.append(c.copy())
    watchInterval = i
    print(f"client added, {c}")


def updateMessageTime(pid):
    global clientList

    for i, c in enumerate(clientList):
        if c['pid'] == pid:
            # print(f"{time.time():.0f} update message time, pid {pid}, clientIndex: {i}")
            clientList[i]['lastMessageReceivedTime'] = time.time()
            # config.log(f"getLifeSignal time update, pid: {pid}, clientIndex: {i}, time: {time.time()}")


def removeClient(i):
    global clientList

    print(f"remove client from clientList, index: {i}, client: {clientList[i]}")
    del clientList[i]


class kinectListener(rpyc.Service):
    ############################## common routines for clients
    watchInterval = 5

    def on_connect(self, conn):

        print(f"{server} - on_connect triggered")
        callerName = conn._channel.stream.sock.getpeername()
        # self.persistConn = conn
        clientPid, clientInterval = conn.root.exposed_getPid()

        if clientInterval < self.watchInterval:  # use shortest client interval in watchdog loop
            self.watchInterval = clientInterval

        clientIndex = [i for i in clientList if i['conn'] == conn]
        if len(clientIndex) == 0:
            client = {'conn': conn,
                      'callerName': callerName,
                      'pid': clientPid,
                      'interval': clientInterval,
                      'lastMessageReceivedTime': time.time()}
            addClient(client, self.watchInterval)
        # config.log(f"on_connect in '{server}' with {client}")

    def on_disconnect(self, conn):
        callerName = conn._channel.stream.sock.getpeername()
        print(f"{server} - on_disconnect triggered, conn: {callerName}")

    def exposed_getLifeSignal(self, pid):

        updateMessageTime(pid)
        return True

    def exposed_terminate(self):
        print(f"{server} task - terminate request received")
        os._exit(0)
        return True

    ############################## common routines for clients

    def exposed_getDepth(self, orientation):
        print("kinect - getDepth request received")
        return kinect.obstacleMap(orientation)


    def exposed_kinectDeactivate(self):
        print("kinect - kinectDeactivate request received")
        kinect.kinectDeactivate()




