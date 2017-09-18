#!/usr/bin/env python

# Generic driver for the Neato XV-11 Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#
# TODO: make sure we don't use listing[-1] without checking if the list is empty
#

"""
neato_driver.py is a generic driver for the Neato XV-11 Robotic Vacuum.
ROS Bindings can be found in the neato_node package.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import socket
import time
import select

BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second

xv11_analog_sensors = [ "WallSensorInMM",
                "BatteryVoltageInmV",
                "LeftDropInMM",
                "RightDropInMM",
                "RightMagSensor",
                "LeftMagSensor",
                "XTemp0InC",
                "XTemp1InC",
                "VacuumCurrentInmA",
                "ChargeVoltInmV",
                "NotConnected1",
                "BatteryTemp1InC",
                "NotConnected2",
                "CurrentInmA",
                "NotConnected3",
                "BatteryTemp0InC" ]

xv11_digital_sensors = [ "SNSR_DC_JACK_CONNECT",
                "SNSR_DUSTBIN_IS_IN",
                "SNSR_LEFT_WHEEL_EXTENDED",
                "SNSR_RIGHT_WHEEL_EXTENDED",
                "LSIDEBIT",
                "LFRONTBIT",
                "RSIDEBIT",
                "RFRONTBIT" ]

xv21_motor_info = [ ' Brush_RPM',
                'Brush_mA'
                'Vacuum_RPM',
                'Vacuum_mA',
                'LeftWheel_RPM',
                'LeftWheel_Load%',
                'LeftWheel_PositionInMM',
                'LeftWheel_Speed',
                'RightWheel_RPM',
                'RightWheel_Load%',
                'RightWheel_PositionInMM',
                'RightWheel_Speed',
                'Charger_mAH',
                'SideBrush_mA']

xv11_motor_info = [ "Brush_MaxPWM",
                "Brush_PWM",
                "Brush_mVolts",
                "Brush_Encoder",
                "Brush_RPM",
                "Vacuum_MaxPWM",
                "Vacuum_PWM",
                "Vacuum_CurrentInMA",
                "Vacuum_Encoder",
                "Vacuum_RPM",
                "LeftWheel_MaxPWM",
                "LeftWheel_PWM",
                "LeftWheel_mVolts",
                "LeftWheel_Encoder",
                "LeftWheel_PositionInMM",
                "LeftWheel_RPM",
                "RightWheel_MaxPWM",
                "RightWheel_PWM",
                "RightWheel_mVolts",
                "RightWheel_Encoder",
                "RightWheel_PositionInMM",
                "RightWheel_RPM",
                "Laser_MaxPWM",
                "Laser_PWM",
                "Laser_mVolts",
                "Laser_Encoder",
                "Laser_RPM",
                "Charger_MaxPWM",
                "Charger_PWM",
                "Charger_mAH" ]

xv11_charger_info = [ "FuelPercent",
                "BatteryOverTemp",
                "ChargingActive",
                "ChargingEnabled",
                "ConfidentOnFuel",
                "OnReservedFuel",
                "EmptyFuel",
                "BatteryFailure",
                "ExtPwrPresent",
                "ThermistorPresent[0]",
                "ThermistorPresent[1]",
                "BattTempCAvg[0]",
                "BattTempCAvg[1]",
                "VBattV",
                "VExtV",
                "Charger_mAH",
                "MaxPWM" ]

class xv11():

    def __init__(self,port):
        self.port = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        #self.port.setblocking(False)
        try:
            self.port.connect((port,7777))
        except socket.error, ex:
            print ex
        read_sockets, write_sockets, exceptional_sockets = select.select([self.port], [self.port], [])
        while self.port not in write_sockets:
            print "Checking for connection!", len(read_sockets), len(write_sockets)
            read_sockets, write_sockets, exceptional_sockets = select.select([self.port], [self.port], [])
            time.sleep(1)

        print "CONNECTED!"
        #self.port.settimeout(10)
        self.last_cmd = None
        # Storage for motor and sensor information
        self.state = {"LeftWheel_PositionInMM": 0, "RightWheel_PositionInMM": 0}
        self.stop_state = True
        # turn things on
        time.sleep(2)
        #print self.port.recv(16384)
        time.sleep(2)
        self.setTestMode("on")
        time.sleep(2)
        self.setLDS("on")

    def exit(self):
        self.setLDS("off")
        self.setTestMode("off")

    def send_command(self,cmd):
        print cmd.data
        if cmd.data == 'shutdown':
            self.exit()
            self.port.send("shutdown\r\n")
        else:
            self.port.send(cmd.data + "\r\n")

    def setTestMode(self, value):
        """ Turn test mode on/off. """

        self.port.send("testmode " + value + '\r\n')
        print "SETTING TEST MODE TO",value

    def setLDS(self, value):
        print "setldsrotation " + value + "\r\n"
        self.port.send("setldsrotation " + value + '\r\n')

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        self.port.send("getldsscan\r\n")

    @staticmethod
    def filter_outliers(ranges,intensities):
        # debug: turn off filtering for now
        #return (ranges,intensities)
        if len(ranges) == 0:
            return (ranges,intensities)
        # filter out lone detections
        for i in range(len(ranges)):
            previous = (i-1)%len(ranges)
            next = (i+1)%len(ranges)
            if (ranges[previous] == 0 and ranges[next] == 0) or intensities[i] < 10:
                ranges[i] = 0.0
                intensities[i] = 0.0
        # filter out ranges that are too long or too short
        for i in range(len(ranges)):
            if ranges[i] > 5.0:
                pass
	       	    # assume no detection is free space
                #ranges[i] = 2.0
                #intensities[i] = 50
            if ranges[i] < .2 or ranges[i] > 5.0:
                ranges[i] = 0.0
                intensities[i] = 0.0
        return (ranges,intensities)

    def getScanRanges(self):
        """ Read values of a scan -- call requestScan first! """
        ranges = list()
        intensities = list()

        try:
            remainder = ""
            found_start_token = False
            while not(found_start_token):
                while True:
                    try:
                        line = self.port.recv(1024)
                        break
                    except:
                        pass
                if line.find('Unknown Cmd') != -1:
                    # something weird happened bail
                    pass
                    #print "GOT AN UNKONWN COMMAND ", line
                    #return ([],[])
                line = remainder + line
                remainder = ""
                listing = [s.strip() for s in line.splitlines()]
                if not(line.endswith('\n')) and len(listing):
                    remainder = listing[-1]
                    listing = listing[0:-1]

                for i in range(len(listing)):
                    entry = listing[i]
                    if entry.startswith('AngleInDegrees') and (len(listing)-1>i or line.endswith('\n')):
                        listing = listing[i+1:]
                        found_start_token = True
                        break
            if len(listing) and not(line.endswith('\n')):
                remainder = listing[-1]
                listing = listing[0:-1]
            else:
                remainder = ""

            while True:
                for i in range(len(listing)):
                    entry = listing[i]
                    vals = entry.split(',')
                    try:
                        a = int(vals[0])
                        r = int(vals[1])
                        intensity = int(vals[2])
                        if len(ranges) > a:
                            # got a value we thought we lost
                            ranges[a] = r/1000.0
                            intensities[a] = intensity
                        else:
                            ranges.append(r/1000.0)
                            intensities.append(intensity)
                    except:
                        ranges.append(0.0)
                        intensities.append(0.0)
                        # should not happen too much... debug if it does
                        pass
                    if len(ranges) >= 360:
                        return xv11.filter_outliers(ranges, intensities)

                listing = []
                while True:
                    try:
                        line = self.port.recv(1024)
                        break
                    except:
                        pass
                listing = [s.strip() for s in line.splitlines()]
                if len(listing) > 0:
                    listing[0] = remainder + listing[0]
                    remainder = ""

                if not(line.endswith('\n')) and len(listing):
                    remainder = listing[-1]
                    listing = listing[0:-1]
                else:
                    remainder = ""
            return xv11.filter_outliers(ranges, intensities)
        except:
            return ([],[])        
        

    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then, 
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if not(self.stop_state):
                self.port.send("setmotor 1 1 1\r\n")
                self.stop_state = True
            else:
                pass
                #self.port.send("setmotor 0 0 0\r\n")
        else:
            self.stop_state = False
            self.last_cmd = (l,r,s)
            self.port.send("setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s))+"\r\n")
        self.flush_socket()

    def flush_socket(self):
        # for now just return
        return
        self.port.setblocking(False)
        #read_sockets = select.select([self.port], [], [])
        while True:
            try:
                line = self.port.recv(1024)
                if len(line) == 0:
                    break
            except:
                break
            #read_sockets = select.select([self.port], [], [])
        self.port.setblocking(True)

    def getMotors(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """
        #self.port.flushInput()
        self.port.send("getmotors\r\n")

        while True:
            try:
                line = self.port.recv(1024)
                break
            except Exception as inst:
                print "UNEXPECTED! ERROR 0 " + str(inst)
        if line.find('Unknown Cmd') != -1:
            # something weird happened bail
            print "UNEXPECTED! ERROR 1"
            raise IOError('Get Motors Failed')
        listing = [s.strip() for s in line.splitlines()]
        if not(line.endswith('\n')) and len(listing):
            remainder = listing[-1]
            listing = listing[0:-1]
        else:
            remainder = ""
        found_start_token = False

        while len(listing) < 14 or not found_start_token:
            if not found_start_token:
                for i,l in enumerate(listing):
                    if l.startswith('Parameter,Value'):
                        found_start_token = True
                        listing = listing[i+1:]
                        break
            if len(listing) >= 14:
                break
            try:
                line = remainder + self.port.recv(1024)
#                break
            except:
                print "UNEXPECTED! ERROR 2"
                pass
            remainder = ""
            listing += [s.strip() for s in line.splitlines()]
            if not(line.endswith('\n')) and len(listing):
                remainder = listing[-1]
                listing = listing[0:-1]
            else:
                remainder = ""
        for i in range(len(listing)):
            try:
                values = listing[i].split(',')
                self.state[values[0]] = int(values[1])
            except Exception as inst:
                pass
        self.flush_socket()
        return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"],self.state["LeftWheel_Speed"],self.state["RightWheel_Speed"]]

    def getAnalogSensors(self):
        print "NOT CURRENTLY SUPPORTED"
        """ Update values for analog sensors in the self.state dictionary. """
        self.port.write("getanalogsensors\n")
        line = self.port.readline()
        while line.split(",")[0] != "SensorName":
            try:
                line = self.port.readline()
            except:
                return
        for i in range(len(xv11_analog_sensors)):
            try:
                values = self.port.readline().split(",")
                self.state[values[0]] = int(values[1])
            except:
                pass

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """
        self.port.send("getdigitalsensors\r\n")
        line = self.port.recv(1024)
        if line.find('Unknown Cmd') != -1:
            # something weird happened bail
            raise IOError('Get Digital Sensors Failed')

        listing = [s.strip() for s in line.splitlines()]
        if not(line.endswith('\n')) and len(listing):
            remainder = listing[-1]
            listing = listing[0:-1]
        else:
            remainder = ""

        while (len(listing) < 11):
                line = remainder + self.port.recv(1024)
                remainder = ""
                listing += [s.strip() for s in line.splitlines()]
                if not(line.endswith('\n')) and len(listing):
                    remainder = listing[-1]
                    listing = listing[0:-1]
                else:
                    remainder = ""
        for i in range(len(listing)-1):
            try:
                values = listing[i+1].split(',')
                self.state[values[0]] = int(values[1])
            except:
                pass
        return [self.state['LFRONTBIT'],self.state['LSIDEBIT'],self.state['RFRONTBIT'],self.state['RSIDEBIT']]

    def getCharger(self):
        """ Update values for charger/battery related info in self.state dictionary. """
        self.port.write("getcharger\n")
        line = self.port.readline()
        while line.split(",")[0] != "Label":
            line = self.port.readline()
        for i in range(len(xv11_charger_info)):
            values = self.port.readline().split(",")
            try:
                self.state[values[0]] = int(values[1])
            except:
                pass

    def setBacklight(self, value):
        if value > 0:
            self.port.write("setled backlighton")
        else:
            self.port.write("setled backlightoff")

    #SetLED - Sets the specified LED to on,off,blink, or dim. (TestMode Only)
    #BacklightOn - LCD Backlight On  (mutually exclusive of BacklightOff)
    #BacklightOff - LCD Backlight Off (mutually exclusive of BacklightOn)
    #ButtonAmber - Start Button Amber (mutually exclusive of other Button options)
    #ButtonGreen - Start Button Green (mutually exclusive of other Button options)
    #LEDRed - Start Red LED (mutually exclusive of other Button options)
    #LEDGreen - Start Green LED (mutually exclusive of other Button options)
    #ButtonAmberDim - Start Button Amber Dim (mutually exclusive of other Button options)
    #ButtonGreenDim - Start Button Green Dim (mutually exclusive of other Button options)
    #ButtonOff - Start Button Off

