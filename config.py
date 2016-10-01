#!/usr/bin/evn python
#coding:utf-8

import sys
from library import Singleton

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
        self._cloud   =[self.get_node(0,1),self.get_node(0,2),self.get_node(0,3)]
        self._vehicle = [self.get_node(1,1),self.get_node(1,2),self.get_node(1,3)]
        self._lidar   = [self.get_node(2,1),self.get_node(2,2),self.get_node(2,3),self.get_node(2,4)]
        

    def isInt(self,x):
        try:
            return isinstance(int(x),int)
        except ValueError:
            return False

    def get_node(self,param1,param2):
        value=self._root[param1][param2].get('value')
        if self.isInt(value) is True:
            return int(value)
        else:
            return value
    def get_cloud(self):
        return self._cloud
    def get_vehicle(self):
        return self._vehicle
    def get_lidar(self):
        return self._lidar


# Global config
config=Config()

if __name__=="__main__":
    print config.get_vehicle()
    print config.get_lidar()
    print config.get_cloud()

