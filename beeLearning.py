# -*- coding: utf-8 -*-
"""
Created

author: Eatai Roth
based on http://www.toptechboy.com/tutorial/python-with-arduino-lesson-11-plotting-and-graphing-live-data-from-arduino-with-matplotlib/
and requires arduino file of beeLearning_adns_9800_ER.ino
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial, struct, time, datetime, sys, os

class beeBall():

    
    
    def __init__(self, SAVEFOLDER = None, SAVEFILENAME = 'testTrial', COM_PORT = 2, SAMP_RATE = 20, DURATION = 200, BALL_DIAMETER = 55.1):
        plt.close("all")
        
        self.date = datetime.datetime.now().isoformat('-')
        
        if SAVEFOLDER is None:
            self.saveFolder = os.getcwd()
        else:
            self.saveFolder = SAVEFOLDER
       
        self.saveFilename = SAVEFILENAME
        
        self.ballDiameter = BALL_DIAMETER  # radius in mm
        self.duration = DURATION
        self.sampRate = SAMP_RATE
        self.dT = float(1.0/self.sampRate)
        self.data = []
        self.varNames = ('t', 'dX1', 'dY1', 'dX2', 'dY2', 'BdX', 'BdY', 'BdTheta')
        
        self.arduinoSerial = serial.Serial(COM_PORT, 115200,
        timeout=5,
        bytesize = serial.EIGHTBITS)
        
        self.resetArduino()
    
    def __del__(self):
        self.arduinoSerial.close()
        # sys.exit()
        
    def resetArduino(self):
        print 'Resetting Arduino...'
        self.arduinoSerial.setDTR(level=False)
        time.sleep(1)
        self.arduinoSerial.flushInput()
        self.arduinoSerial.setDTR()
        print 'Arduino reset.'
        
        # ----- Confirm ADNS-9800 chips are properly initialized -----
        print self.arduinoSerial.readline()[:-2]  # Firmware confirm, chip 1
        print self.arduinoSerial.readline()[:-2]  # Init confirm, chip 1
        print self.arduinoSerial.readline()[:-2]  # Firmware confirm, chip 2
        print self.arduinoSerial.readline()  # Init confirm, chip 2

    def readDataLine(self): 
        COS45 = np.cos(np.pi/4)
        
        inputString = self.arduinoSerial.readline()[:-2]
        try:
            data_temp = np.array(map(float, inputString.split(',')))
            t = data_temp[0]
            x1 = data_temp[1]
            y1 = data_temp[2]
            x2 = data_temp[3]
            y2 = data_temp[4]
            data_temp = data_temp.reshape(1,5)
#            self.data_temp = np.array([[t, x1, y1, x2, y2]])
        
        except IndexError:
            data_out = np.zeros([1,8])
            print "Missing data"
        except ValueError:
            data_out = np.zeros([1,8])
            print "Problem String %s" %inputString
        else:
            self.beeData = np.array([[(y1-y2)/(2*COS45), (y1+y2)/(2*COS45), (x1+x2)/(self.ballDiameter)]])
            data_out = np.hstack((data_temp, self.beeData))
        
        return data_out         
    
    def readDataLoop(self):
        numSamps = self.duration * self.sampRate
        currTime = 0
        counter = 0
        print 'Starting collection'
        while (currTime<=(self.duration)) and (counter <= numSamps):  
            data_temp = self.readDataLine()
            
            if (counter == 0):
                self.data = data_temp
            else:
                self.data = np.vstack((self.data, data_temp)) 
            counter+=1
            currTime = data_temp[0,0]
        
    def startTrial(self, RPS = None, dur = None):
        if not(RPS is None):
            self.sampRate = RPS
        if not(dur is None):
            self.duration = dur
        
        time.sleep(0.2)
        self.arduinoSerial.flushInput()

        self.arduinoSerial.write(struct.pack('>i', self.sampRate))
        self.arduinoSerial.write(struct.pack('>i', self.duration))
        
        print self.arduinoSerial.readline()[:-2]  # Sample rate confirm
        print self.arduinoSerial.readline()[:-2]  # Duration confirm

        self.readDataLoop()
    
    def saveData(self):
        trialNum = 0
        
        try:
            os.stat(self.saveFolder)
        except:
            os.mkdir(self.saveFolder)
            
        testString = "%s%s_%04d.txt" %(self.saveFolder, self.saveFilename, trialNum)
        while os.path.isfile(testString):
            trialNum += 1
            testString = "%s_%04d.txt" %(self.saveFilename, trialNum)
        print "Saving to " + testString
        
        dateString = "%s\n"  %self.date
        headingString = ''
        for varName in self.varNames:
            headingString += '{0:>15}'.format(varName)
        headingString = dateString + headingString + '\n'
        np.savetxt(testString, self.data , fmt = "%15.4f", delimiter = ",", header = headingString)

        #np.savetxt(testString, self.data, fmt = %7.4f%, delimiter = '\t')
        
    def plotData(self):
        t = self.data[:,0]
        dX = self.data[:,5]
        dY = self.data[:,6]
        dTheta = self.data[:,7]
        
        Theta = np.hstack((0, (np.cumsum(dTheta)%(2*np.pi))[:-1]))
        X = np.cumsum(dX*np.cos(Theta) - dY*np.sin(Theta))
        Y = np.cumsum(dX*np.sin(Theta) + dY*np.cos(Theta))
        plt.plot(X,Y)
        plt.axis('equal')
        plt.hold(True)
        plt.draw()
        
if __name__ == '__main__':  
    try:
        BB = beeBall('BeeBallTestData\\', "trial")
        BB.startTrial(100,20)
        BB.plotData()
        plt.show()
        BB.saveData()
    finally:
        plt.show()
        BB.arduinoSerial.close()
    # BB.plotData()
    
        
    
    
    


