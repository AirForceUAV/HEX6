#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,threadpool,math
from vehicle import Drone
from library import CancelWatcher
from config import config

mqttPool = threadpool.ThreadPool(1)
# eventPool=threadpool.ThreadPool(1)

def on_connect(client, userdata, rc):
	print "Connected mqtt with result code "+str(rc)
	# Subscribe Topic "Command"
	client.subscribe("Command",qos=1)

def eval_wrapper(command):
	eval(command)	

def on_message(client, userdata, msg):
	msg=str(msg.payload)
	print msg
	if msg.find('Cancel')==-1:
		requests = threadpool.makeRequests(eval_wrapper,(msg,))
		[mqttPool.putRequest(req) for req in requests]
	else:
		CancelWatcher.Cancel=True
		time.sleep(1)
		drone.brake()
		# requests = threadpool.makeRequests(eval_wrapper,("drone.brake()",))
		# [mqttPool.putRequest(req) for req in requests]


def init_mqtt(ip,port=1883,username="",password=""):
	import paho.mqtt.client as mqtt	
	client = mqtt.Client(client_id='companion',clean_session=True,userdata=None)
	# client.reinitialise(client_id='companion',clean_session=True, userdata=None)
	client._username=username
	client._password=password
	client.on_connect = on_connect
	client.on_message = on_message
	client.connect(ip, port)
	# client.loop_forever()   # blocking
	client.loop_start()	      # non-blocking
	return client

if __name__=="__main__":
	cloud=config.get_cloud()
	drone=Drone()
	if config.get_lidar()[0]==1:
		from lidar import Lidar
		print "Connecting to Lidar ..."
		lidar=Lidar(drone)
	
	if cloud[0]==1:
		print 'Connecting to Cloud ...'
		client=init_mqtt(cloud[1],cloud[2],cloud[3],cloud[4])
		
		while True:
			client.publish('FlightLog',drone.FlightLog())	
			time.sleep(1)

	vehicle.close()

	print("Completed")
	
 
			