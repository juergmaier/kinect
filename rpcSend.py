


import xmlrpc.client
import config

xmlrpcClientList = []

def addXmlrpcClient(ip, port):
    """
    each process has to register to get updates from servos
    :param ip:
    :param port:
    :return:
    """

    global xmlrpcClientList

    clientAddr = f"http://{ip}:{port}"
    print(f"clientAddr: {clientAddr}")
    try:
        proxy = xmlrpc.client.ServerProxy(clientAddr)
        print(f"registered clients: {xmlrpcClientList}")
        print(f"new client: {proxy}")
        if str(proxy) not in str(xmlrpcClientList):
            xmlrpcClientList.append(proxy)
            print(f"new client added")
        else:
            config.log(f"client {proxy} already registered")

    except Exception as e:
        print(f"failure to add client {ip}:{port} - {e}")

    try:
        config.log("xmlrpc log testmessage")
    except Exception as e:
        print(f"log to xmlrpcClientList failed: {e}")



def publishLog(msg):

    for i, c in enumerate(xmlrpcClientList):
        #print(f"log to {xmlrpcClientList[i]}")
        try:
            c.exposed_log(msg)
        except Exception as e:
            xmlrpcClientList.remove(i)
            config.log(f"exception in publishLog: {str(e)}")

