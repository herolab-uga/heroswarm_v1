import socket

def Main():

    HOST='127.0.0.1'
    PORT=12345
    CLIENT_SOCKET1=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    CLIENT_SOCKET1.connect((HOST,PORT))
    print("1 Connected!!")
    CLIENT_SOCKET2=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    CLIENT_SOCKET2.connect((HOST,PORT))
    print("2 Connected!!")
    message1="I am the Client-1,1"
    message2="I am the Client-2,1"
    CLIENT_SOCKET1.send(message1.encode('ascii'))
    CLIENT_SOCKET2.send(message2.encode('ascii'))
    message1="I am the Client-1,2"
    message2="I am the Client-2,2"
    CLIENT_SOCKET1.send(message1.encode('ascii'))
    CLIENT_SOCKET2.send(message2.encode('ascii'))

if __name__ == '__main__':
    Main()