import time, math, socket, struct, sys
import numpy  as np


#init params
wheelRadius = 0.035
numberOfRuns = 0
lastTime = time.time()
distanceIncrementAnterior = 9999
invWheelDistance = 5.88
numberOfCars = 1
targetIdOdometry = 0 #position to update
"""it is necessary to write initial pose of all the cars"""
x = [10,30,60,90,120] #initial position of every car X
y = [0,0,0,0,0] #initial position of every car Y
theta = [0,0,0,0,0]
disntaceBetweenVehicles = [0]*numberOfCars
lastTime = [time.time()]*numberOfCars

# socket for unity
UDP_IP_Sim = "localhost"
UDP_PORT_Sim = 5005
MESSAGE_Sim = "Starting!\n"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
  
#socket for car
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.SOL_UDP)
s.bind(('192.168.173.10',6790))
s.settimeout(20)

def send_order(car, message):
        address = '192.168.173.'+str(car)
        port = 6789
        s.sendto(message.encode(), (address, port))

def write_file(file_name, text_to_write):
        f = open(file_name,'w')
        f.write(str(text_to_write))
        f.close()

def read_file(file_name):
        f = open(file_name, 'r')
        list_split = f.read().strip().split(';')
        f.close()
        return list_split

def write_vars_names(vl,file):
        f = open(file,'w') # overwrites file
        for i in vl:
            f.write(str(i)+'\n')
        f.close()

def write_vars(vl,file):
        f = open(file,'a') # overwrites file
        for i in vl:
            f.write(str(i)+';')
        f.write('\n')
        f.close()

def create_list_vars(ret):
        return str(ret)

variables_list = []
value_to_write = []
  

def generateCheckSum(msg):
    try:
        BCC = msg[0]
        BCC ^= msg[1]
        BCC ^= msg[2]
        for i in range(3,msg[1]-3):
            BCC ^= msg[i]
        BCC ^= msg[msg[1]-3]
        return BCC
    except:
        return -1
 
vel = 0      
while 1:    
    package, address = s.recvfrom(1024)
    
    """
    package[0] is the mode 'E','I'
    package[1] is the message size
    package[2:end] is the message data
    """
    #print(package[0],",",package[1],",",package[2],",",package[package[1]-2])#,",",package[4],",",package[5],",",package[6],",",package[7],",",package[8],",",package[9],",",package[10],",",package[11],",",package[12],",",package[13])    

    # checking message correct
    IdData = 999
    if((package[0]) == ord('%')):        
        if(generateCheckSum(package) == package[package[1]-2]):
            if(package[2] == 1):                 
                wL = struct.unpack('f', package[7:11])[0]                
                wR = struct.unpack('f', package[3:7])[0]
                
                vel = ((wL+wR)*0.5*0.037)*0.2 + vel*0.8
                print(vel)
                
                IdData = package[package[1]-3]  
                
                if(IdData == targetIdOdometry):                    
                    sampleTime = time.time()-lastTime[IdData]
                    lastTime[IdData] = time.time()
                
                    distanceIncrement = (wR+wL)*wheelRadius*0.5*sampleTime                    
                    disntaceBetweenVehicles[IdData] = distanceIncrementAnterior - distanceIncrement    
                    distanceBytes = bytearray(struct.pack("f", disntaceBetweenVehicles[IdData]))
#                    
#                    msg = [ord('%'),6,1,0]
#                    BCC = generateCheckSum(msg)
#                    send_order(address[0][-3:], (chr(msg[0])+chr(msg[1])+chr(msg[2])+chr(msg[3])+chr(BCC)+'#'))                    
#                    
                    angleIncrement = (wR-wL)*wheelRadius*invWheelDistance*sampleTime
                    
                    x[IdData] += distanceIncrement*math.cos(theta[IdData])
                    y[IdData] += distanceIncrement*math.sin(theta[IdData])
                    theta[IdData] += angleIncrement        
                    
                    MESSAGE_Sim = "%"+str(x[IdData])+":"+str(y[IdData])+":"+str(theta[IdData])+":"+str(IdData)+"#"
                    sock.sendto(bytes(MESSAGE_Sim, "utf-8"), (UDP_IP_Sim, UDP_PORT_Sim)) #send message to unity simulator
                    
                    distanceIncrementAnterior = distanceIncrement
                    
                    targetIdOdometry = (targetIdOdometry+1)%numberOfCars
                    if(targetIdOdometry == 0):
                        distanceIncrementAnterior = 9999                             
                
#                numberOfRuns += 1                
#                if(numberOfRuns == 5):
#                    numberOfRuns = 0
    
                file_name = address[0][-3:]+'-46.txt'
                for i in range(int((package[1]-6)/4)-2):
                    value_to_write.append(struct.unpack('f', package[4*i+11:4*i+15])[0])
                write_vars(value_to_write, file_name)
                value_to_write = []
                
            elif(package[2] == 0):
                for i in range(int((package[1]-6)/5)): 
                    variables_list.append(str(package[5*i+3:5*i+8]))
                write_vars_names(variables_list, 'channels-vars.txt')
                variables_list = []
                
                msg = [ord('%'),6,0,0]
                BCC = generateCheckSum(msg)
                send_order(address[0][-3:], (chr(msg[0])+chr(msg[1])+chr(msg[2])+chr(msg[3])+chr(BCC)+'#'))
            else:
                print("instruction does not exist")                    
        else:
            pass
            #corrupted message          
    
    l = read_file('send.txt')    
    if l != ['0', '0']:
        if int(l[0]) == 1:
            msg = [ord('%'),6,11,0]
            BCC = generateCheckSum(msg)
            send_order(100, ('%'+chr(6)+chr(11)+chr(0)+chr(BCC)+'#'))      #start
        elif int(l[0]) == 2:
            msg = [ord('%'),6,12,0]
            BCC = generateCheckSum(msg)
            send_order(100, ('%'+chr(6)+chr(12)+chr(0)+chr(BCC)+'#'))     #stop
        elif int(l[0]) == 3:
            msg = [ord('%'),6,13,0]
            BCC = generateCheckSum(msg)
            send_order(100, ('%'+chr(6)+chr(13)+chr(0)+chr(BCC)+'#'))     # change lane
        elif int(l[0]) == 4:
            msg = [ord('%'),6,14,0]
            BCC = generateCheckSum(msg)
            send_order(100, ('%'+chr(6)+chr(14)+chr(0)+chr(BCC)+'#'))     #modify speed
            
        if int(l[1]) == 1:
            msg = [ord('%'),6,11,1]
            BCC = generateCheckSum(msg)
            send_order(101, ('%'+chr(6)+chr(11)+chr(1)+chr(BCC)+'#'))      #start
        elif int(l[1]) == 2:
            msg = [ord('%'),6,12,1]
            BCC = generateCheckSum(msg)
            send_order(101, ('%'+chr(6)+chr(12)+chr(1)+chr(BCC)+'#'))     #stop
        elif int(l[1]) == 3:
            msg = [ord('%'),6,13,1]
            BCC = generateCheckSum(msg)
            send_order(101, ('%'+chr(6)+chr(13)+chr(1)+chr(BCC)+'#'))     # change lane
        elif int(l[1]) == 4:
            msg = [ord('%'),6,14,1]
            BCC = generateCheckSum(msg)
            send_order(101, ('%'+chr(6)+chr(14)+chr(1)+chr(BCC)+'#'))     #modify speed
        write_file('send.txt', '0;0')