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
import cPickle as pickle
import struct
from os import system


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

    def __init__(self, port):
        self.host = port
        self.port = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.port.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.port.setsockopt(socket.SOL_TCP, socket.TCP_KEEPIDLE, 30)
        self.port.setsockopt(socket.SOL_TCP, socket.TCP_KEEPINTVL, 15)

        try:
            self.port.connect((port,7777))
        except socket.error, ex:
            print ex
        read_sockets, write_sockets, exceptional_sockets = select.select([self.port], [self.port], [])
        while self.port not in write_sockets:
            print "Checking for connection!", len(read_sockets), len(write_sockets)
            read_sockets, write_sockets, exceptional_sockets = select.select([self.port], [self.port], [])
            time.sleep(1)

        UDP_IP = "0.0.0.0"
        UDP_PORT = 7777

        self.sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        self.sensor_sock.bind((UDP_IP, UDP_PORT))
        self.sensor_sock.settimeout(.02)
        print "CONNECTED!"

        #self.port.settimeout(10)
        self.last_cmd = None
        # Storage for motor and sensor information
        self.state = {"LeftWheel_PositionInMM": 0, "RightWheel_PositionInMM": 0}
        self.stop_state = True
        # turn things on
        time.sleep(4)
        self.setTestMode("on")
        time.sleep(2)
        self.setLDS("on")

    def do_udp_hole_punch(self):
        """ This should be called once the Raspberry pi is forwarding sensor packets
            in order to allow UDP traffic to be forwarded from the Olin network or
            the ethernet to the OLIN-ROBOTICS network """
        system('hping3 -c 1 -2 -s 7777 -p 7777 ' + self.host)

    def exit(self):
        self.setLDS("off")
        self.setTestMode("off")

    def setTestMode(self, value):
        """ Turn test mode on/off. """

        self.port.send("testmode " + value + '\n')
        print "SETTING TEST MODE TO",value

    def setLDS(self, value):
        print "setldsrotation " + value + '\n'
        self.port.send("setldsrotation " + value + '\n')

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        # for now we will rely on the pi to request scans, we will just fetch the sensor packet here
        #self.port.send("getldsscan\r\n")
        try:
            sensor_packet, _ = self.sensor_sock.recvfrom(65536)
            print 'got a sensor packet'
            self.sensor_dict = pickle.loads(sensor_packet)
        except socket.timeout:
            self.sensor_dict = {}
            print "no packet received... not necessarily a problem"

    def getScanRanges(self):
        """ Read values of a scan -- call requestScan first! """
        ranges = list()
        intensities = list()
        if 'ldsscanranges' not in self.sensor_dict:
            #print 'missing scan ranges'
            return ([],[])
        ranges = struct.unpack('<%sH' % self.sensor_dict['ldsscanranges'][0], self.sensor_dict['ldsscanranges'][1])
        return ([r/1000.0 for r in ranges], [10.0]*len(ranges))

    def resend_last_motor_command(self):
        if self.last_cmd:
            self.setMotors(self.last_cmd[0], self.last_cmd[1], self.last_cmd[2])

    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then, 
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if not(self.stop_state):
                self.port.send("setmotor 1 1 1\n")
                self.stop_state = True
                self.last_cmd = (0.0, 0.0, 0.0)
            else:
                pass
                #self.port.send("setmotor 0 0 0\r\n")
        else:
            self.stop_state = False
            self.last_cmd = (l,r,s)
            self.port.send("setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s))+"\n")

    def getMotors(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """
        if 'motors' in self.sensor_dict:
            self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"] = \
                    struct.unpack('<2d', self.sensor_dict['motors'])
            return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]]
        return None

    def getAccel(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """
        if 'accel' in self.sensor_dict:
            self.state["PitchInDegrees"], self.state["RollInDegrees"], self.state["XInG"], self.state["YInG"], self.state["ZInG"], self.state["SumInG"] =\
                struct.unpack('<6f', self.sensor_dict['accel'])
            return [self.state["PitchInDegrees"],
                    self.state["RollInDegrees"],
                    self.state["XInG"],
                    self.state["YInG"],
                    self.state["ZInG"],
                    self.state["SumInG"]]
        return None

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """
        #self.port.send("getdigitalsensors\r\n")
        # for now we will let the raspberry pi request the digital sensors by itself
        if 'digitalsensors' in self.sensor_dict:
            self.state['LFRONTBIT'],self.state['LSIDEBIT'],self.state['RFRONTBIT'],self.state['RSIDEBIT'] =\
                struct.unpack('<4d', self.sensor_dict['digitalsensors'])

            return [self.state['LFRONTBIT'],self.state['LSIDEBIT'],self.state['RFRONTBIT'],self.state['RSIDEBIT']]
        return None

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
