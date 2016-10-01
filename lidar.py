#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,math,os
from vehicle import Drone
from library import angle_heading_target,_angle,get_distance_metres
from library import CancelWatcher,Singleton
from config import config

con=config.get_lidar()

class Lidar(object):
    _pipeSet = {}
    __metaclass__= Singleton
    def __init__(self,vehicle):

        replyPipe="./Reply"
        requestPipe="./Request"
        self.vehicle=vehicle
       
        if self.__class__._pipeSet.has_key((replyPipe,requestPipe)) is False:
            self.__class__._pipeSet[(replyPipe,requestPipe)] = {}
            self.__class__._pipeSet[(replyPipe,requestPipe)]["Reply"] =  open(replyPipe,"r")
            self.__class__._pipeSet[(replyPipe,requestPipe)]["Request"] =  open(requestPipe,"w")
        self.request = self.__class__._pipeSet[(replyPipe,requestPipe)]["Request"]
        self.reply= self.__class__._pipeSet[(replyPipe,requestPipe)]["Reply"]
        

    def Decision(self,targetDirection):
        targetDirection = (360 - targetDirection) % 360
        self.request.write(struct.pack("HH",targetDirection,0))
        self.request.flush()
        pointFmt = "HHH"
        (quality,angle,distance) = struct.unpack(pointFmt,self.reply.read(struct.calcsize(pointFmt)))
        angle = (360 - angle) %360;
        return angle

    def Guided_Avoid(self,_type='Guided',velocity=1,checktime=1,deviation=2):
        if con[0] ==0:
            print 'Lidar is closed!!!'
            return 1       
        watcher=CancelWatcher()
        if _type is "Guided":
            target=self.vehicle.get_target()
            if target is None:
                self._log("Target is None!")
                return -1
            self._log('Guided with Avoidance to {}'.format(target))
            
        elif _type is 'RTL':
            target=self.vehicle.get_home()
            if target is None:
                self._log("Home is None!")
                return -1
            self._log('RTL ! Home is {}'.format(target))

        while not watcher.IsCancel():
            current_location =self.vehicle.get_location()
            distance=round(get_distance_metres(current_location,target),2)
            self._log("Distance to Target {}m".format(distance))
            if distance<3:
                self._log("Reached Target Waypoint!")
                vehicle.brake()
                return 1    
            angle=angle_heading_target(current_location,target,self.vehicle.get_heading())
            angle_avoid=self.Decision(angle)
            if _angle(angle_avoid)>deviation:
                self.vehicle.brake()
                print 'turn',angle_avoid
                self.vehicle.condition_yaw2(angle_avoid)               
            self.vehicle.forward(velocity)
            time.sleep(checktime)
        return 0
    def _log(self,msg):
        self.vehicle._log(msg)

pid = os.fork()
if pid == 0:
    os.execl("./ultra_simple","ultra_simple",con[1],str(con[2]),str(con[3]),"")
    exit(0)


if __name__=='__main__':
    vehicle=Drone()
    
    # while True:
    #   raw_input('Next')
    #   print lidar.Decision(0)
    lidar=Lidar(vehicle)
    vehicle.arm()
    vehicle.takeoff()
    vehicle.set_target(0,30)
    vehicle.Guided()
    # time.sleep(30)
    vehicle.RTL()
    # lidar.Guided_Avoid()
    # lidar.vehicle.close()

