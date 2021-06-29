import threading
import getch
import socket, pickle


def Main():
    HOST='192.168.1.78'
    PORT=12345
    CLIENT_SOCKET=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    CLIENT_SOCKET.connect((HOST,PORT))
    print("1 Connected!!")
    CLIENT_SOCKET.send(b'9')
    while True:
        query=input("Enter Query:")
        CLIENT_SOCKET.send(query.encode('ascii'))
        clientDataP=CLIENT_SOCKET.recv(4096)
        clientData=pickle.loads(clientDataP)
        print((clientData))








if __name__ == '__main__':
    Main()

