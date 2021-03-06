#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time,math,os,struct
from vehicle import Drone
from library import angle_heading_target,_angle,get_distance_metres,get_bearing
from library import CancelWatcher,Singleton,_angle
from config import config

con=config.get_lidar()
way=config.get_way()
check=config.get_time()[0]
dev=config.get_angle()[2]
vel=config.get_speed()[4]

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
    def more_angle(self,anlge):
        if angle>=0 and angle<=180:
            angle+=10
        else:
            angle-=10
        return angle

    def Avoid(self,target,velocity,checktime,deviation):

        watcher=CancelWatcher()
        while not watcher.IsCancel():
            current_location =self.vehicle.get_location()
            distance=round(get_distance_metres(current_location,target),2)
            # self._log('Cur{},target{}'.format(current_location,target))
            self._log("Distance to Target {}m".format(distance))
            if distance<3:
                self._log("Reached Target Waypoint!")
                vehicle.brake(velocity)
                return 1    
            angle=angle_heading_target(current_location,target,self.vehicle.get_heading())
            angle_avoid=self.Decision(angle)
            if _angle(angle_avoid)>deviation:
                self.vehicle.brake(velocity)
                print 'Turn',angle_avoid
                anlge_avoid=self.more_angle(angle_avoid)
                self.vehicle.condition_yaw2(angle_avoid)               
            self.vehicle.forward(velocity)
            time.sleep(checktime)
        return 0

    def Avoid2(self,target,velocity,checktime):

        watcher=CancelWatcher()
        self.vehicle.condition_yaw2(angle_heading_target(self.vehicle.get_location(),target,self.vehicle.get_heading()))

        while not watcher.IsCancel():
            current_location =self.vehicle.get_location()
            distance=round(get_distance_metres(current_location,target),2)
            # self._log('Cur:{},target:{}'.format(current_location,target))
            self._log("Distance to Target {}m".format(distance))
            if distance<3:
                self._log("Reached Target Waypoint!")
                vehicle.brake()
                return 1
            angle=angle_heading_target(current_location,target,self.vehicle.get_heading())
            angle_avoid=self.Decision(angle)
            anlge_avoid=self.more_angle(angle_avoid)
            Sin=math.sin(math.radians(angle_avoid))
            Cos=math.cos(math.radians(angle_avoid))
            forward_v = round(velocity*Cos,1)
            right_v   = round(velocity*Sin,1)
            print 'Turn:{} forward:{}m/s right:{}m/s'.format(angle_avoid,forward_v,right_v)     
            self.vehicle.send_body_offset_ned_velocity(forward_v,right_v,0)
            time.sleep(checktime)
        return 0

    def Guided_Avoid(self,avoid_way=way[0],velocity=vel,checktime=check,deviation=dev):
        if con[0] ==0:
            print 'Lidar is closed!!!'
            return 1       
    
        target=self.vehicle.get_target()
        if target is None:
            self._log("Target is None!")
            return -1
            self._log('Guided with Avoidance to {}'.format(target))
            
        if avoid_way==1:
            self.Avoid(target,velocity,checktime,deviation)
        elif avoid_way==2:
            self.Avoid2(target,velocity.checktime)

    def RTL_Avoid(self,avoid_way=way[1],velocity=vel,checktime=check,deviation=dev):
        if con[0] ==0:
            print 'Lidar is closed!!!'
            return 1       
    
        target=self.vehicle.get_home()
        if target is None:
            self._log("HOME is None!")
            return -1
            self._log('RTL with Avoidance to {}'.format(target))
            
        if avoid_way==1:
            self.Avoid(target,velocity,checktime,deviation)
        elif avoid_way==2:
            self.Avoid2(target,velocity.checktime)

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
    # vehicle.arm()
    # vehicle.takeoff()
    vehicle.set_target(0,30)
    # vehicle.Guided()
    # time.sleep(30)
    # vehicle.RTL()
    lidar.Guided_Avoid()
    vehicle.close()

