#!/usr/bin/env python
# -*-coding:Latin-1 -*
import serial
import math 
import time

import navio.ms5611
import navio.util
import navio.ublox

import spidev
import argparse 
import sys
import navio.mpu9250

from picamera import PiCamera
camera = PiCamera()

navio.util.check_apm()

baro = navio.ms5611.MS5611()
baro.initialize()

ser = serial.Serial('/dev/ttyACM0', 9600)
print("CTRL + C pour arrêter")

def gps():
	latitude = "none"
	if __name__ == "__main__":

		ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)

		ubl.configure_poll_port()
		ubl.configure_poll(navio.ublox.CLASS_CFG, navio.ublox.MSG_CFG_USB)
		#ubl.configure_poll(navio.ublox.CLASS_MON, navio.ublox.MSG_MON_HW) no touch

		ubl.configure_port(port=navio.ublox.PORT_SERIAL1, inMask=1, outMask=0)
		ubl.configure_port(port=navio.ublox.PORT_USB, inMask=1, outMask=1)
		ubl.configure_port(port=navio.ublox.PORT_SERIAL2, inMask=1, outMask=0)
		ubl.configure_poll_port()
		ubl.configure_poll_port(navio.ublox.PORT_SERIAL1)
		ubl.configure_poll_port(navio.ublox.PORT_SERIAL2)
		ubl.configure_poll_port(navio.ublox.PORT_USB)
		ubl.configure_solution_rate(rate_ms=1000)

		ubl.set_preferred_dynamic_model(None)
        	ubl.set_preferred_usePPP(None)

        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSLLH, 1)
       		ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_PVT, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_STATUS, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SOL, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELNED, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_SVINFO, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_VELECEF, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_POSECEF, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_RAW, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SFRB, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_SVSI, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_ALM, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_RXM, navio.ublox.MSG_RXM_EPH, 1)
        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_TIMEGPS, 5)
        	ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_CLOCK, 5)
        	#ubl.configure_message_rate(navio.ublox.CLASS_NAV, navio.ublox.MSG_NAV_DGPS, 5) no touch 
		x = True
		y = True
        	while x== True or y == True:
            		msg = ubl.receive_message()
            		if msg is None:
                		if opts.reopen:
                    			ubl.close()
                    			ubl = navio.ublox.UBlox("spi:0.0", baudrate=5000000, timeout=2)
                    			continue
                		print(empty)
                		break
            		#print(msg) if need
            		if msg.name() == "NAV_POSLLH": 
				longitude = str(msg).split(",")[1] 
				longitude = float(longitude[11:])/(10000000) 
				latitude = str(msg).split(",")[2] 
				latitude = float(latitude[10:])/(10000000) 
				altitude = str(msg).split(",")[3] 
				altitude = float(altitude[8:])/1000 - 49 
				x =False

            		if msg.name() == "NAV_STATUS":
             			ngps = str(msg).split(",")[1]
				ngps = int(ngps[8:])
            	 		y=False
	ubl.close()
    	return longitude, latitude, altitude, ngps

def AccGyroMag():
	imu = navio.mpu9250.MPU9250()
	if imu.testConnection():
		print "Connection established: True"
	else:
		sys.exit("Connection established: False")

	imu.initialize()
	m9a, m9g, m9m = imu.getMotion9()
	return m9a, m9g, m9m

def cam_start(i):
	camera.start_preview()
	camera.start_recording('/home/pi/backup/video' + str(i) + '.h264')

def cam_stop():
	camera.stop_recording()
	camera.stop_preview()

def dist(x1,y1,z1, x2, y2, z2):
	dx = math.fabs(x2- x1) * 81028
	dy =  math.fabs(y2-y2) *111180
	dz =  math.fabs(z2-z1)
	return dx, dy, dz

def speed(dx,dy,dz,t):
	vx = dx/t
	vy = dy/t
	vz = dz/t
	return vx, vy, vz

i =0
k=0
try:
	cam_start(i)
except:
	print "unexpected error camera
        ser.write(str("\nUnexpected error camera\n"))

while(True):
	fichier = open("/home/pi/backup/data.txt", "a")

	if k== 6:
		i+=1
		try :
			cam_stop()
			cam_start(i)
		except:
		print "unexpected error camera
		ser.write(str("\nUnexpected error camera\n"))
		k=0
	k+=1

	baro.refreshPressure()
	time.sleep(0.01) #Pressure data ready 10ms (mBar)
	baro.readPressure()

	baro.refreshTemperature()
	time.sleep(0.01) # Temperature data ready 10ms (C)
	baro.readTemperature()

	baro.calculatePressureAndTemperature()

	print "Temperature(C): %.6f" % (baro.TEMP), "Pressure(millibar): %.6f" % (baro.PRES)	
	
	ser.write(str("Temperature(C): %.6f\n" % (baro.TEMP)))
	fichier.write("\nTemperature(C): %.6f\n" % (baro.TEMP))
        ser.write(str("Pressure(millibar): %.6f\n" % (baro.PRES)))
	fichier.write("Pressure(millibar): %.6f\n" % (baro.PRES))
        time.sleep(1)
	
	ser.write(str("\n"))
	fichier.write("\n")
	longitude1, latitude1, altitude1, ngps1 = gps()	
	print "altitude : " and altitude1
	ser.write(str("longitude : %8f\n" % (longitude1)))
	fichier.write("longitude : %8f\n" % (longitude1))
	ser.write(str("latitude : %8f\n" % (latitude1)))
	fichier.write("latitude : %8f\n" % (latitude1))
	ser.write(str("altitude : %3f\n" % (altitude1)))
	fichier.write("altitude : %3f\n" % (altitude1))
	time.sleep(3)
	longitude2, latitude2, altitude2, ngps2 = gps()
	dx, dy ,dz = dist(longitude1, latitude1, altitude1,  longitude2, latitude2, altitude2)
	vx, vy, vz = speed(dx, dy ,dz ,3)
	ser.write(str("vitesse longitude : "))
        fichier.write("vitesse longitude : ")
	ser.write(str(vx))
        fichier.write(str(vx))
	ser.write(str("\nvitesse latitude : "))
        fichier.write("\nvitesse latitude : ")
        ser.write(str(vy))
        fichier.write(str(vy))
	ser.write(str("\nvitesse axe z : "))
        fichier.write("\nvitesse axe z : ")
        ser.write(str(vz))
        fichier.write(str(vz))
	time.sleep(1)
	
	ser.write(str("\n"))
	fichier.write("\n")
	m9a, m9g, m9m = AccGyroMag()
	print "Acc:", "{:+7.3f}".format(m9a[0]), "{:+7.3f}".format(m9a[1]), "{:+7.3f}".format(m9a[2]),
	print " Gyr:", "{:+8.3f}".format(m9g[0]), "{:+8.3f}".format(m9g[1]), "{:+8.3f}".format(m9g[2]),
	print " Mag:", "{:+7.3f}".format(m9m[0]), "{:+7.3f}".format(m9m[1]), "{:+7.3f}".format(m9m[2])
	ser.write(str("\nAcc : (x)"))
	ser.write(str("Acc:" and  "{:+7.3f}".format(m9a[0])))
	ser.write(str("  (y) "))
      	ser.write(str("Acc:" and  "{:+7.3f}".format(m9a[1])))
	ser.write(str("  (z) "))
        ser.write(str("Acc:" and  "{:+7.3f}".format(m9a[2])))
	fichier.write("\n Acc : ")
	fichier.write("{:+7.3f}".format(m9a[0]))
 	fichier.write("{:+7.3f}".format(m9a[1]))
	fichier.write("{:+7.3f}".format(m9a[2]))
	
	ser.write(str("\n\n"))
	fichier.write("\n\n")	
	time.sleep(2)
	
	fichier.close()
