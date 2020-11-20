import numpy as np
from lib import command 
from struct import *
import time,sys
from xbee import XBee
import serial
import glob
from math import ceil,floor,copysign,sqrt,atan2,asin,cos,sin
from callbackFunc import xbee_received
import shared
import datetime
import cpluspluscode_Wrap4Python_ext

#Set the robot destination address
DEST_ADDR = 0x3002
#Convert this to a sequence of bytes, for transmission
DEST_ADDR = pack('>H',DEST_ADDR)
#Definition PI
PI = 3.14159265359
#Definition Dt
DELTA_T= 200
#LQR controller
K =[31.6228,19.059] # K =[10.0, 5.4772]
#Kalman's Filter
K_F =[2.3700,1.4869]# K_F =[1.8809,1.1167]

# Point for path following
Px_0=[ 300, 432, 488, 432, 293, 0, -293, -432, -488, -432, -300, 300, 432, 488, 432, 293, 0, -293, -432, -488, -432, -300]
Py_0=[-147, 2, 280, 511, 655, 758, 655, 511, 280, 2, -147, -354, -502, -780, -1011, -1155, -1258, -1155, -1011, -780, -502, -354]
Px_1=[432, 488, 432, 293, 0, -293, -432, -488, -432, -300, 300, 432, 488, 432, 293, 0, -293, -432, -488, -432, -300, 300]
Py_1=[ 2, 280, 511, 655, 758, 655, 511, 280, 2, -147, -354, -502, -780, -1011, -1155, -1258,-1155, -1011, -780, -502, -354, -147] 
Px_mid=[ 366.0, 460.0, 460.0, 362.5, 146.5, -146.5, -362.5, -460.0, -460.0, -366.0, 0,  366.0, 460.0, 460.0, 362.5, 146.5, -146.5, -362.5, -460.0, -460.0, -366.0, 0]
Py_mid=[ -72.5, 141.0, 395.5, 583.0, 706.5, 706.5, 583.0, 395.5, 141.0, -072.5, -250.5, -428.0, -641.0, -8955, -1083.0, -1206.5, -1206.5, -1083.0, -895.5, -641.0, -428.0, -250.5]
v_x=[ 0.7485, 0.9803, 0.9719, 0.7195, 0.3316, -0.3316, -0.7195, -0.9719, -0.9803, -0.7485, -0.3261, -0.7463, -0.9803, -0.9719, -0.7195, -0.3316, 0.3316, 0.7195, 0.9719, 0.9803, 0.7463, 0.3261]
v_y=[ -0.6631, -0.1975, 0.2356, 0.6945, 0.9434, 0.9434, 0.6945, 0.2356, -0.1975, -0.6631, -0.9453, -0.6656, -0.1975, 0.2356, 0.6945, 0.9434, 0.9434, 0.6945, 0.2356, -0.1975, -0.6656, -0.9453]
# Point for path following


# Class OctoROACH

class OctoROACH:

   def __init__(self):
      self.px=0        # Position of the octoroach
      self.py=0        # TODO: consider full space positioning
      self.theta=0

      self.previous_line=0
      self.actual_line=0
      self.Vicon_handler=cpluspluscode_Wrap4Python_ext.cpluspluscode_Wrap4Python()

      try:
          self.ser = serial.Serial(shared.BS_COMPORT, shared.BS_BAUDRATE, timeout=3, rtscts=1)
      except serial.serialutil.SerialException:
          print "Could not open serial port:",shared.BS_COMPORT
          print "Check that this COM port exists, and edit shared.py as needed."
          sys.exit()

      self.xb = XBee(self.ser, callback = xbee_received)

      angRateDeg = 0
      angRate = round( angRateDeg / shared.count2deg)
      while not(shared.steering_rate_set):
         print "Setting steering rate..."
         self.xb_send(0, command.SET_CTRLD_TURN_RATE, pack('h',angRate))
         time.sleep(0.5)

      motorgains = [100,2,0,4,0,    100,2,0,4,0]
      while not(shared.motor_gains_set):
         print "Setting motor gains..."
         self.xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
         time.sleep(0.5)

      steeringGains = [0,0,0,0,0]
      while not (shared.steering_gains_set):
         print "Setting steering gains..."
         self.xb_send(0, command.SET_STEERING_GAINS, pack('5h',*steeringGains))
         time.sleep(0.5)

   def retrievePosition(self):
      self.Vicon_handler.retrievePositioning()
      self.px=self.Vicon_handler.return_pos_x()
      self.py=self.Vicon_handler.return_pos_y()
      self.theta=self.Vicon_handler.return_orientation_theta()
   ########## Helper functions #################
   def xb_send(self,status, type, data):
      payload = chr(status) + chr(type) + ''.join(data)
      self.xb.tx(dest_addr = DEST_ADDR, data = payload)

   def resetRobot():
      self.xb_send(0, command.SOFTWARE_RESET, pack('h',0))
   ########## END Helper functions ########################
   def setVelocities(self,wl,wr):
#      print wl
#      print wr
      moves = 1
      moveq = [moves, \
               wl, wr, 350] #400
#              wl, wr, time
      self.xb_send(0, command.SET_MOVE_QUEUE, pack('=h'+moves*'hhL', *moveq))

   def determine_Op_Line(self):
      distances=[]
      distances_c=[]
      signo_1=[]
      signo_2=[]
      cast_OUT=[]
      linea=0  
      signo=1
      for aa in range(len(Px_0)):
         vector_temp=[self.px - Px_mid[aa],self.py - Py_mid[aa], 0] 
         vector_temp2=[v_x[aa],v_y[aa],0]
         vector_temp3=[self.px - Px_0[aa],self.py - Py_0[aa], 0] 
         vector_temp4=[self.px - Px_1[aa],self.py - Py_1[aa], 0]
 
         distances.append( abs( dot( vector_temp , vector_temp2 ) ) )
         distances_c.append( norm(vector_temp) )
         if (distances_c[aa] < distances_c[linea]):
            linea=aa
         copysign(signo, dot([0,0,1], cross( vector_temp2 , vector_temp3)) )
         signo_1.append( signo )
         copysign(signo, dot([0,0,1], cross( vector_temp2 , vector_temp4)) )
         signo_2.append( signo )
         if (signo_1[aa]!=signo_2[aa]):
            cast_OUT.append(aa)

      if (len(cast_OUT)!=0):
         linea=cast_OUT[0]
         for bb in cast_OUT:
            if ( distances[bb]<distances[linea] ):
               linea= cast_OUT[bb]

      self.previous_line=self.actual_line

      if ((linea==10) or (linea==21)):
         if ((linea==10) and ( self.previous_line==20 or self.previous_line==21)):
            linea=21
         if ((linea==21) and ( self.previous_line==9 or self.previous_line==10)):
            linea=10

      self.actual_line=linea

      return linea

# Class OctoROACH

def dot(v1,v2):
   N=len(v1)
   valor=0
   for a in range(N):
      valor= valor + v1[a]*v2[a]
   return valor

def norm(v):
   N=len(v)
   valor=0
   for a in range(N):
      valor = valor + pow(v[a],2)
   return sqrt( valor )

def unit_v(v):
   N=len(v)
   magn=norm(v)
   resultado=[]
   for a in range(N):
      resultado.append(v[a]/magn)
   return resultado

def cross(v1,v2):
   v=[ ((v1[1]*v2[2])-(v1[2]*v2[1])), ((v1[2]*v2[0])-(v1[0]*v2[2])), ((v1[0]*v2[1])-(v1[1]*v2[0]))]
   return v

def findFileName():
    filenames = glob.glob("*FollowingEIGHT*.txt");
    # Explicitly remove "imudata.txt", since that can mess up the pattern
    if 'FollowingEIGHT.txt' in filenames:
        filenames.remove('FollowingEIGHT.txt')
    
    if filenames == []:
        dataFileName = "FollowingEIGHT1.txt"
    else:
        filenames.sort()
        filenum = [int(fn[14:-4]) for fn in filenames]
        filenum.sort()
        filenum = filenum[-1] + 1
        dataFileName = "FollowingEIGHT" + str(filenum) + ".txt"
    return dataFileName

def main():
    global angRateDeg, angRate, motorgains, steeringGains, runtime,  n, fthrust, moveq

    dataFileName = findFileName();   
    print "Data file:  ", dataFileName

    fileout = open(dataFileName,'w')
    fileout.write('%% \n clear,clc,close all \n')
    fileout.write('% K= ['+str(K[0])+','+str(K[1])+']' ' Kf= ['+str(K_F[0])+','+str(K_F[1])+'] \n')
    fileout.write('% Formato= [OctoR_1.px  OctoR_1.py  OctoR_1.theta  z1  z1_est  z2_est]\n')
    fileout.write('EstructuraMatlab=[\n')
    counterC=0
    counterCB=0
    counterCC=0

    OctoR_1=OctoROACH()
    
    raw_input("Press enter to start run ...")
    counterCB=0
    z1_est=0
    z2_est=0
 
    while (counterC<30000):
       OctoR_1.retrievePosition()
       Data_Operative=OctoR_1.determine_Op_Line()

       print "\nlinea de operacion "+str(Data_Operative)+ "iterador "+str(counterC)

       if counterCC==1000:
          z1_est=0
          z2_est=0
          counterCC=0

       vector_temp=[OctoR_1.px - Px_mid[Data_Operative],OctoR_1.py - Py_mid[Data_Operative], 0]
       vector_temp2=[v_x[Data_Operative],v_y[Data_Operative],0]
       d=dot( vector_temp , vector_temp2 )

       vector_temp=[cos(OctoR_1.theta), sin(OctoR_1.theta), 0]
       vector_temp2=unit_v([Px_1[Data_Operative]-Px_0[Data_Operative],Py_1[Data_Operative]-Py_0[Data_Operative],0])
       cross_producct=cross(vector_temp,vector_temp2)
       errorAngle=asin(cross_producct[2])

       z1=d
       u_w=-(K[0]*z1_est)-(K[1]*z2_est)
       temp_est=[ ((-1.8809*z1_est)+z2_est+(K_F[0]*z1))*(0.02) , ((-1.1167*z1_est)+u_w+(K_F[1]*z1))*(0.02) ]
       z1_est = z1_est + temp_est[0]
       z2_est = z2_est + temp_est[1]    

       u_w=-(K[0]*z1_est)-(K[1]*z2_est)
#       print "\n u_w"
#       print u_w

       omega=- (u_w/((100)*cos(errorAngle)))

       fileout.write('    ' + str(OctoR_1.px) + '    ' + str(OctoR_1.py) + '    ' + str(OctoR_1.theta) + '    ' + str(z1) + '    ' + str(z1_est ) + '    ' + str(z2_est) +';\n')


       if omega> 0.78:
          omega= 0.78
       if omega<-0.78:
          omega=-0.78

       w_l=round(75 -(60*omega))
       w_r=round(75 +(60*omega))

#       print "\nw_l "
#       print w_l
#       print "\nw_r "
#       print w_r

       if counterCB==70: #50
          OctoR_1.setVelocities(w_l,w_r)
          counterCB=0

       counterC+=1    
       counterCB+=1
       counterCC+=1

    filenum = int(dataFileName[14:-4])

    fileout.write('];\n')
    fileout.write('\n save DataMatlab'+str(filenum)+' EstructuraMatlab \n')
    fileout.close()

    print "Ctrl + C to exit"

    while True:
        try:
            time.sleep(1)
            #print ".",
        except KeyboardInterrupt:
            break

    OctoR_1.xb.halt()
    OctoR_1.ser.close()


    print "Done"

#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        OctoR_1.xb.halt()
        OctoR_1.ser.close()
    #except Exception as args:
    #    print "\nGeneral exception:",args
    #    print "Attemping to exit cleanly..."
    #    xb.halt()
    #    ser.close()
    except serial.serialutil.SerialException:
        OctoR_1.xb.halt()
        OctoR_1.ser.close()
