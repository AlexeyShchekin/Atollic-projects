'''
Created on Feb 1, 2019

@author: a.shchekin
'''
import numpy as np
import serial
import serial.tools.list_ports
import time
import h5py
import struct

Dir = "64CH_1"

ports = list(serial.tools.list_ports.comports())
print(ports)
devices = []
for p in ports:
    try:
        com = serial.Serial(p[0],baudrate=115200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,timeout=10,xonxoff=0)
        com.write(("AI-DEVICE\r\n").encode('utf-8'))
        print("Port "+p[0]+" is opened and being checked for AGP")
        t0 = time.time()
        while (com.inWaiting()==0 and time.time()-t0<2.0):
            continue
        if (com.inWaiting()>0):
            resp = com.readline()
            if (chr(resp[0])=='O' and chr(resp[1])=='K'):
                devices.append(p)
            else:
                print(resp)
        else:
            print(str("Port "+p[0]+" is not responding as AI device..."))
        com.close()
    except:
        print(str("Port "+p[0]+" is not available for connection or blocked..."))
print("List of found AI devices: "+str([p[0] for p in devices]))

if len(devices)>0:
    com = serial.Serial(devices[0][0],baudrate=115200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,timeout=10,xonxoff=0)
    
    with h5py.File("../data/EMB_Test_Set_"+Dir+".h5", 'r') as hf:
        X_test = hf["EMB_Test_X"][:]
        Y_test = hf["EMB_Test_Y"][:]
    
    Description = np.load("../data/MedicalDescription_"+Dir+".npy")
    
    N = X_test.shape[0]
    print("Total requests: "+str(N))
    #N = 10
    
    for n in range(0,N):
        com.write(("START-TEST\r\n").encode('utf-8'))
        t0 = time.time()
        while (com.inWaiting()==0 and time.time()-t0<5.0):
            continue
        if (com.inWaiting()>0):
            resp = com.readline()
            if (chr(resp[0])=='O' and chr(resp[1])=='K'):
                for i in range(0,8):
                    for j in range(0,8):
                        com.write(struct.pack('<f',np.float32(X_test[n,i,j])))
                        #com.write(struct.pack('>f',np.float32(0.1)))
                t0 = time.time()
                while (com.inWaiting()<256 and time.time()-t0<10.0):
                    continue
                if (com.inWaiting()<256):
                    print("Some data lost...")
                    print(com.inWaiting())
                else:
                    X1 = np.zeros([8,8])
                    for i in range(0,8):
                        for j in range(0,8):
                            X1[i][j] = struct.unpack('<f', com.read(4))[0]
                    t0 = time.time()
                    while (com.inWaiting()<36 and time.time()-t0<10.0):
                        continue
                    if (com.inWaiting()<36):
                        print("Response lost...")
                        print(com.inWaiting())
                    else:
                        R = np.zeros([9])
                        for i in range(0,9):
                            R[i] = struct.unpack('<f', com.read(4))[0]
                        '''for i in range(0,8):
                            for j in range(0,8):
                                print(str(X_test[n][i][j])+", "+str(X1[i][j]))'''
                        index_R = np.argmax(R)
                        index_ref = np.argmax(Y_test[n,:])
                        print("N= "+str(n)+" Reference: "+str(Description[index_ref])+" Predicted: "+str(Description[index_R]))
                #Send data
            else:
                print(resp)
        else:
            print(str("Port "+p[0]+" is not responding as AI device..."))
    
    com.close()
