#!/usr/bin/evn python
#coding:utf-8

import time,serial,traceback,sys,math
from dronekit import LocationGlobalRelative


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.

    This method is an approximation, and may not be accurate over large distances and close to the 
    earth's poles.
    """ 
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing


def get_distance_metres(aLocation1, aLocation2):  
    """
    Distance aLocation1 and aLocation2.
    """    
    dlat = aLocation2.lat - aLocation1.lat
    dlong= aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def angle_diff(angle1,angle2,sign):
    diff=(360+sign*(angle2-angle1))%360
    if diff<180:
        return True
    else:
        return False
    
def _angle(angle):
    if angle>180:
        angle=360-angle
    return angle

def angle_heading_target(origin,target,heading):
    target_north=get_bearing(origin,target)
    heading_target=(360+target_north-heading)%360
    return int(heading_target)

class CancelWatcher(object):
    Cancel=False
    count=0
    def __init__(self):
        if self.__class__.count==0 and self.__class__.Cancel==True:
            self.__class__.Cancel = False
        self.__class__.count+=1
    def IsCancel(self):
        return self.__class__.Cancel
    def __del__(self):
        self.__class__.count-=1
        if self.__class__.count==0:
            self.__class__.Cancel = False

class Singleton(type):  
    def __init__(cls, name, bases, dict):  
        super(Singleton, cls).__init__(name, bases, dict)  
        cls._instance = None  
    def __call__(cls, *args, **kw):  
        if cls._instance is None:  
            cls._instance = super(Singleton, cls).__call__(*args, **kw)  
        return cls._instance 



if __name__=='__main__':
    pass
   

