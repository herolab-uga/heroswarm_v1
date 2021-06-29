import socket
import math

def main():
    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 65432        # Port to listen on (non-privileged ports are > 1023)
    
    s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen()
    while True:
        conn, addr = s.accept()
        data = conn.recv(1024)
        setMotion(data)
        conn.shutdown(socket.SHUT_RDWR)
        conn.close()
                

        
def setMotion(data):
    dta=data.decode("utf-8").split()
    stpFlag=False
    if not len(dta)==0:
        x=int(float(dta[0]))
        y=int(float(dta[1]))
        theta=int(float(dta[2]))
        eStatus= dta[3]=='True'
        ex=int(float(dta[4]))
        ey=int(float(dta[5]))
        trgDist=(math.sqrt((x-ex)**2)+(y-ey)**2)
        print(trgDist)

        if(trgDist<1000):
            stpFlag=True
        if theta>10 and theta<180:
            print('Turn Left')
        elif theta<350 and theta >=180 :
            print('Turn Right')
        elif stpFlag or not eStatus:
            print('Stop')
        else:
            print('Go Straight')




if __name__ == '__main__':
    main()

