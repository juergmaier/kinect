
import os

from xmlrpc.server import SimpleXMLRPCServer

import config
import rpcSend
import kinect

# xmlrpc
xmlrpcServer  = None

def xmlrpcListener():

    global xmlrpcServer

    xmlrpcServer = SimpleXMLRPCServer((config.MY_IP, config.MY_XMLRPC_PORT))

    xmlrpcServer.register_function(exposed_register, "exposed_register")
    xmlrpcServer.register_function(exposed_getLifeSignal, "exposed_getLifeSignal")
    xmlrpcServer.register_function(exposed_getDepth, "exposed_getDepth")
    xmlrpcServer.register_function(exposed_kinectDeactivate, "exposed_kinectDeactivate")
    xmlrpcServer.register_function(exposed_terminate, "exposed_terminate")

    xmlrpcServer.serve_forever()


def exposed_register(ip, port):
    print("exposed_register request received from {ip}:{port}")
    rpcSend.addXmlrpcClient(ip, port)
    return True


def exposed_getLifeSignal():
    return True


def exposed_getDepth(orientation):
    return kinect.obstacleMap(orientation)


def exposed_kinectDeactivate():
    kinect.kinectDeactivate()


def exposed_terminate():
    print(f"xmlrpc kinect - terminate command received")
    os._exit(0)
    return True


