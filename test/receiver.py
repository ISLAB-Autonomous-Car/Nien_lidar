import socket
import time
localIP     = "192.168.1.2"
localPort   = 20001
bufferSize  = 1024
msgFromServer       = "Hello UDP Client"
bytesToSend         = str.encode(msgFromServer)
# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")
# Listen for incoming datagrams
# def get_ip():
#     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     try:
#         # doesn't even have to be reachable
#         s.connect(('10.255.255.255', 1))
#         IP = s.getsockname()[0]
#     except Exception:
#         IP = '127.0.0.1'
#     finally:
#         s.close()
#     return IP
# hostname = socket.gethostname()
# IPAddr = get_ip()
# print("Your Computer Name is: " + hostname)
# print("Your Computer IP Address is: " + IPAddr)
while(True):
    t=time.time()
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]
    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address) 
    print("FPS:{}".format(1/(time.time()-t)))
    print(clientMsg)
    print(clientIP)
    # Sending a reply to client
    UDPServerSocket.sendto(bytesToSend, address)
