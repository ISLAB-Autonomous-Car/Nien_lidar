import socket
def send_sig(msgFromClient,IPaddress):
    msgFromClient       = "Hello UDP Server"
    bytesToSend         = str.encode(msgFromClient)
    serverAddressPort   = (IPaddress, 20001)
    bufferSize          = 1024
    # Create a UDP socket at client side
    UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # Send to server using created UDP socket
    while True:
        UDPClientSocket.sendto(bytesToSend, serverAddressPort)
        msgFromServer = UDPClientSocket.recvfrom(bufferSize) 
        msg = "Message from Server {}".format(msgFromServer[0])
