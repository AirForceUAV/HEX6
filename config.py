#!/usr/bin/evn python
#coding:utf-8

import sys
from library import Singleton,isNum

class Config(object):
    __metaclass__=Singleton
    def __init__(self):
        file_name='HEX6.xml'
        try: 
            import xml.etree.cElementTree as ET
        except ImportError: 
            import xml.etree.ElementTree as ET   
        try: 
            tree = ET.parse(file_name)
            self._root = tree.getroot()
        except Exception, e: 
            print "Error:cannot parse file:",file_name
            sys.exit(1)
        self._cloud   = self.get_config(0,5)
        self._vehicle = self.get_config(1,3)
        self._lidar   = self.get_config(2,4)
        self._distance= self.get_config(3,4,2)
        self._speed   = self.get_config(4,5,2)
        self._angle   = self.get_config(5,3)
        self._time    = self.get_config(6,1)
        self._way     = self.get_config(7,2)

    def isInt(self,x):
        try:
            return isinstance(int(x),int)
        except ValueError:
            return False
    def isFloat(self,x):
        try:
            return isinstance(float(x),float)
        except ValueError:
            return False
    def get_node(self,param1,param2,_type):
        value=self._root[param1][param2].get('value')
        if self.isInt(value) and _type==1:
            return int(value)
        elif self.isFloat(value) and _type==2:
            return float(value)
        else:
            return value
    def get_cloud(self):
        return self._cloud
    def get_vehicle(self):
        return self._vehicle
    def get_lidar(self):
        return self._lidar
    def get_distance(self):
        return self._distance
    def get_speed(self):
        return self._speed
    def get_angle(self):
        return self._angle
    def get_time(self):
        return self._time
    def get_way(self):
        return self._way
    def get_config(self,index,num,_type=1):
        result=[]
        times=1
        while times<=num:
            result.append(self.get_node(index,times,_type))
            times+=1
        return result


# Global config
config=Config()

if __name__=="__main__":
    print 'cloud',config.get_cloud()
    print "vehicle",config.get_vehicle()
    print 'lidar',config.get_lidar()    
    print 'distance',config.get_distance()
    print 'speed',config.get_speed()
    print 'angle',config.get_angle()
    print 'time',config.get_time()
    print 'avoidance way',config.get_way()

