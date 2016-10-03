#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect,VehicleMode,LocationGlobalRelative,LocationGlobal
import time,math,json,threadpool
from pymavlink import mavutil
from my_vehicle import MyVehicle
from config import config
from library import angle_diff,get_distance_metres
from library import Singleton,CancelWatcher


# mutex=threading.Lock()
        
class Drone(object):
    __metaclass__=Singleton
    def __init__(self):
        self.mqtt=None
        self.home_location = None
        self.sitl=None
        self.target=None
        self.vehicle = self.connection(config.get_vehicle()[1])
        self.channels_mid=None
            
    def connection(self,args):
        if args =='sitl':
            from dronekit_sitl import SITL
            sitl = SITL()
            self.sitl=sitl
            sitl.download('copter', '3.3', verbose=True)
            sitl_args = ['-I0', '--model', 'quad', '--home=39.757880,116.357050,584,0']
            sitl.launch(sitl_args, await_ready=True, restart=True)
            connection_string='tcp:127.0.0.1:5760'
        else:
            connection_string=args
        baudrate=config.get_vehicle()[2]
        self._log('Connecting to vehicle on:{} Baudrate:{}'.format(connection_string,baudrate))
        vehicle = connect(connection_string,wait_ready=True,baud=baudrate,vehicle_class=MyVehicle)
        # Register observers
        # vehicle.add_attribute_listener('location',self.location_callback)
        # vehicle.add_attribute_listener('battery',self.battery_callback)
        # vehicle.add_attribute_listener('heading',self.heading_callback)
        return vehicle

    def location_callback(self, vehicle, name, location):
        if location.global_relative_frame.alt is not None:
            self.altitude = location.global_relative_frame.alt
        self.current_location = location.global_relative_frame

    # def battery_callback(self,vehicle, name,battery):
    #   if battery.level<20:
    #       self._log('Low Battery! Remaining Power: {0} '.format(battery.level))

    # def heading_callback(self,vehicle,name,heading):
    #   self._log('Current heading: {0}'.format(heading))

    def get_alt(self,relative=True):
        if relative==True:
            return self.vehicle.location.global_relative_frame.alt
        else:
            return self.vehicle.location.global_frame.alt
    
    def get_target(self):       
        return self.target

    def set_airspeed(self,speed):
        self.vehicle.airspeed=speed

    def set_groundspeed(self,speed):
        self.vehicle.groundspeed=speed

    def arm(self):
        watcher=CancelWatcher()

        self.home_location = self.get_location()
        home=self.get_home()
        home_str="{0},{1},{2}".format(home.lat,home.lon,home.alt)
        self._log('Home is:%s'%home_str)
        if self.mqtt is not None:
            self.mqtt.publish("Home",home_str)

        # Set GUIDED Mode
        self._log("Set Vehicle.mode = GUIDED") 
        self.vehicle.mode = VehicleMode("GUIDED")
        self._log("Waiting for mode change ...")
        while not self.vehicle.mode.name=='GUIDED' and not watcher.IsCancel():  #Wait until mode has changed            
            time.sleep(.1)
        self._log('Current mode :{0}'.format(self.vehicle.mode.name))
        # Check is_armable
        self._log("Waiting for ability to arm...")
        while not self.vehicle.is_armable and not watcher.IsCancel():
            time.sleep(.1)

        #Armed
        self.vehicle.armed = True
        self._log("Waiting for arming...")
        while not self.vehicle.armed and not watcher.IsCancel():                
            time.sleep(.1)
    
    def disarm(self):
        self._log("DisArmed")
        self.vehicle.armed=False
        
    def takeoff(self,alt=4):
        watcher=CancelWatcher()
        if self.vehicle.armed==False:
            self._log("Please arm!")
        else:
            watcher=CancelWatcher()
            self._log("Taking off to {0}m".format(alt))
            self.vehicle.simple_takeoff(alt)
            # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command after Vehicle.simple_takeoff will execute immediately).
            while not watcher.IsCancel():
                if self.vehicle.location.global_relative_frame.alt>=alt*0.95: #Trigger just below target alt.               
                    self._log("Reached target altitude!!!")
                    break
                time.sleep(.5)
            self.brake()

    def Guided(self,speed=3):
        watcher=CancelWatcher()
        targetLocation=self.get_target()
        self._log('Guided to {}'.format(targetLocation))
        if targetLocation is None:
            print 'Target is None!'
            return 1
        

        while not watcher.IsCancel() and self.Distance_to_target()>=3:
            self.vehicle.simple_goto(targetLocation,groundspeed=speed)
            time.sleep(1)
        self._log('Reached Target Location!!!')

    
    def get_home(self):
        return self.home_location
        
    def get_location(self):
        return self.vehicle.location.global_relative_frame
    def get_location_metres(self,original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles."""
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        targetlocation=[newlat,newlon]
                
        return targetlocation

    def condition_yaw(self,heading,is_relative=1):
        '''After taking off, yaw commands are ignored until the first “movement” command has been received. 
        If you need to yaw immediately following takeoff then send a command to “move” to your current position'''
        if heading<0 or heading>=360 or not str(heading).isdigit():
            print "0<=heading<360"
            return 0
        if is_relative > 0:
            if heading == 0:
                return 0            
            if heading<=180 and heading>0:
                is_cw=1
                print 'Turn Right ',heading 
            elif heading>180 and heading<360:
                is_cw=-1
                heading=360-heading
                print 'Turn Left ',heading
        else:
            is_relative=0
            
        # create the CONDITION_YAW command using command_long_encode()

        msg =self. vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        is_cw,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def condition_yaw2(self,heading):
        if heading<0 or heading>=360 or not str(heading).isdigit():
            print "0<=heading<360"
            return 0
        if heading == 0:
            return 0
        watcher=CancelWatcher()
        
        if heading<=180 and heading>0:
            is_cw=1
            print 'Turn Right ',heading
            target_angle=(self.get_heading()+heading)%360   
        elif heading>180 and heading<360:
            is_cw=-1
            heading=360-heading
            target_angle=(360+self.get_heading()-heading)%360
            print 'Turn Left ',heading
            
        # create the CONDITION_YAW command using command_long_encode()

        msg =self. vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        is_cw,          # param 3, direction -1 ccw, 1 cw
        1, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        while not watcher.IsCancel() and angle_diff(self.get_heading(),target_angle)>1:
            # print self.get_heading()
            time.sleep(.1)
        self._log('Reached angle {}'.format(self.get_heading()))

    def send_body_offset_ned_velocity(self,forward=0,right=0,down=0):
        msg=self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,0,0,
        forward,right,down,    #vx,vy,vz
        0,0,0,   #afx,afy,afz
        0,0)    #yaw yaw_rate

        self.vehicle.send_mavlink(msg)

    def send_body_offset_ned_position(self,forward=0,right=0,down=0):
        msg=self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111111000,
        forward,right,down,
        0,0,0,    #vx,vy,vz
        0,0,0,   #afx,afy,afz
        0,0)    #yaw yaw_rate
        self.vehicle.send_mavlink(msg)

    def forward(self,velocity=1.0):
        self.send_body_offset_ned_velocity(velocity,0,0)
    def backward(self,velocity=1.0):
        self.send_body_offset_ned_velocity(-velocity,0,0)
    def yaw_left(self,angle=3):
        self.condition_yaw(360-angle)
    def yaw_right(self,angle=3):
        self.condition_yaw(angle)
    def roll_left(self,velocity=1.0):
        self.send_body_offset_ned_velocity(0,-velocity,0)
    def roll_right(self,velocity=1.0):
        self.send_body_offset_ned_velocity(0,velocity,0)
    def up(self,velocity=1.0):
        self.send_body_offset_ned_velocity(0,0,-velocity)
    def down(self,velocity=1.0):
        self.send_body_offset_ned_velocity(0,0,velocity)

    #control movemnet by velocity
    def forward_brake(self,distance=1.0,velocity=1.0):
        self._log("Forward to {0}m,velocity is {1}m/s".format(distance,velocity))
        duration=int(distance/velocity)
        watcher=CancelWatcher()
        times=0
        while times<duration and not watcher.IsCancel():
            times+=1
            self.forward(velocity)
            time.sleep(1)
        self.brake()

    def backward_brake(self,distance=1.0,velocity=1.0):
        self._log("Backward to {0}m,velocity is {1}m/s".format(distance,velocity))
        duration=int(distance/velocity)
        watcher=CancelWatcher()
        times=0
        while times<duration and not watcher.IsCancel():
            times+=1
            self.backward(velocity)
            time.sleep(1)
        self.brake()
        

    def roll_left_brake(self,distance=1.0,velocity=1.0):
        self._log("Left to {0}m,velocity is {1}m/s".format(distance,velocity))
        duration=int(distance/velocity)
        watcher=CancelWatcher()
        times=0
        while times<duration and not watcher.IsCancel():
            times+=1
            self.roll_left(velocity)
            time.sleep(1)
        self.brake()

    def roll_right_brake(self,distance=1.0,velocity=1.0):
        self._log("Right to {0}m,velocity is {1}m/s".format(distance,velocity))
        duration=int(distance/velocity)
        watcher=CancelWatcher()
        times=0
        while times<duration and not watcher.IsCancel():
            times+=1
            self.roll_right(velocity)
            time.sleep(1)
        self.brake()
    

    def up_brake(self,distance=1.0,velocity=0.5):
        self._log("Up to {0}m,velocity is {1}m/s".format(distance,velocity))
        duration=int(distance/velocity)
        watcher=CancelWatcher()
        times=0
        while times<duration and not watcher.IsCancel():
            times+=1
            self.up(velocity)
            time.sleep(1)
        self.brake()

    def down_brake(self,distance=1.0,velocity=0.5):
        self._log("Down to {0}m,velocity is {1}m/s".format(distance,velocity))
        duration=int(distance/velocity)
        watcher=CancelWatcher()
        times=0
        while times<duration and not watcher.IsCancel():
            times+=1
            self.down(velocity)
            time.sleep(1)
        self.brake()

    def brake(self):
        self._log('Brake')
        # self.send_body_offset_ned_position(0,0,0)
        self.send_body_offset_ned_velocity(0,0,0)

    def get_heading(self):
        return self.vehicle.heading
    
    def set_target_metres(self,dNorth,dEast,alt=None):
        location=self.get_location_metres(self.get_location(),dNorth,dEast)
        if not str(dNorth).isdigit() or not str(dEast).isdigit():
            return -1
        if not str(alt).isdigit():
            alt=self.get_alt()
        self.target=LocationGlobalRelative(location[0],location[1],alt)
        self._log('Target:{}'.format(self.target))

    def set_target(self,lat,lon,alt=None):
        if not str(lat).isdigit() or not str(lon).isdigit():
            return -1
        if not str(alt).isdigit():
            alt=self.get_alt()
        self.target=LocationGlobalRelative(lat,lon,alt)

    def RTL(self):
        self.retract()
        self._log("Return To Home")
        self.vehicle.mode=VehicleMode("RTL")
    def Land(self):
        self.retract()
        self._log("Landing...")
        self.vehicle.mode=VehicleMode("LAND")
    # def Stab(self):
    #     self._log("STABILIZE")
    #     self.vehicle.mode=VehicleMode("STABILIZE")
    # def Loiter(self):
    #     self._log("Loiter")
    #     self.vehicle.mode=VehicleMode("LOITER")
    # def Althold(self):
    #     self._log('Althold')
    #     self.vehicle.mode=VehicleMode("ALT_HOLD")

    def deploy(self): 
        '''LRG_SERVO_DEPLOY=1750''' 
        if self.vehicle.channels['8']<1750:
            self._log('Deploy Landing Gear')
            self.vehicle.channels.overrides['8']=1900
            self._log('Waiting for deploying LandingGear!')
            time.sleep(3)
        else:
            self._log('Yet deployed')
    def retract(self):   
        '''LRG_SERVO_RTRACT=1250'''  
        if self.vehicle.channels['8']>1250:
            self._log('Retract Landing Gear')
            self.vehicle.channels.overrides['8']=1100
            self._log('Waiting for retracting LandingGear')
            time.sleep(3)
        else:
            self._log('Yet retracted')
    def get_mode(self):
        return self.vehicle.mode.name

    def get_channels(self):
        return self.vehicle.channels

    def LocationGlobal_info(self):
        location=self.get_location()
        return "{},{},{}".format(location.lat,location.lon,location.alt)
    def Attitude_info(self):
        attitude=self.vehicle.attitude
        return "{},{},{}".format(attitude.pitch,attitude.yaw,attitude.roll)
    def Velocity_info(self):
        v=self.vehicle.velocity
        return "{},{},{}".format(v[0],v[1],v[2])
    def GPS_info(self):
        return str(self.vehicle.gps_0.satellites_visible)
    
    def Distance_from_home(self):
        if self.get_home() is not None:
            distance=get_distance_metres(self.get_home(),self.get_location())
            return round(distance,2)
        else:
            return -1
    def Distance_to_target(self):
        if self.target is not None:
            distance=get_distance_metres(self.get_location(),self.get_target())
            return round(distance,2)
        else:
            return -1
    def LocationLocal(self):
        local=self.vehicle.location.local_frame
        return "{},{},{}".format(local.north,local.east,local.down)
    

    def FlightLog(self):
        status={}
        gps=self.vehicle.gps_0
        gimbal=self.vehicle.gimbal
        battery=self.vehicle.battery
        status["id"]=time.time()
        status["Battery"]="{0},{1},{2}".format(battery.voltage,battery.current,battery.level)
        status["GPS"]="{0},{1},{2},{3}".format(gps.eph,gps.epv,gps.fix_type,gps.satellites_visible)
        status["Attitude"]=self.Attitude_info()
        status["LocationGlobal"]=self.LocationGlobal_info() 
        status["LocationLocal"]=self.LocationLocal()
        status["LastHeart"]=str(self.vehicle.last_heartbeat)
        status["Heading"]=str(self.vehicle.heading)
        status["Velocity"]=self.Velocity_info()     
        status["Gimbal"]="{0},{1},{2}".format(gimbal.pitch,gimbal.yaw,gimbal.roll)
        status["EKF"]="{0}".format(self.vehicle.ekf_ok)
        status["Rangefinder"]="{0},{1}".format(self.vehicle.rangefinder.distance,self.vehicle.rangefinder.voltage)
        status["Groundspeed"]=str(self.vehicle.groundspeed)
        status["Airspeed"]=str(self.vehicle.airspeed)
        status["SystemStatus"]=self.vehicle.system_status.state
        status["Mode"]=self.vehicle.mode.name
        status["DistanceFromHome"]=self.Distance_from_home()
        status["DistanceToTarget"]=self.Distance_to_target()
        status["IMU"]=self.vehicle.raw_imu.display()
        status["ServoOutput"]=self.vehicle.servo_output.display()
        status['RPM']=2000
        status["TimeStamp"]=int(time.time())
        channels=[]
        for i in range(1,9):
            channels.append(str(self.vehicle.channels[str(i)]))
        status["Channels"]= ",".join(channels)
        # print(status)
        return json.dumps(status)


    def close(self):
        self._log("Close vehicle object")
        self.vehicle.close()
        # Shut down simulator if it was started.
        if self.sitl is not None:
            self._log('close SITL')
            self.sitl.stop()

    # def get_parameters(self):
    #     self._log("Print all parameters (iterate `vehicle.parameters`):")
    #     params={}
    #     for key, value in self.vehicle.parameters.iteritems():
    #         params[key]=value
    #     return json.dumps(params)

    # def get_parameter(self,key):
    #     return self.vehicle.parameters[key]

    # def set_parameter(self,key,value):
    #     self.vehicle.parameters[key]=value

    def _log(self, message):            
        print "[DEBUG]:"+message
        pass

if __name__=="__main__":

    drone=Drone()
    # print drone.FlightLog()
    drone.arm()
    drone.takeoff()
    drone.set_target_metres(50,0)
    drone.set_groundspeed(3)
    # print drone.get_target()
    drone.Guided()
    # drone.condition_yaw2(230)
    time.sleep(5)
    drone.RTL()
    drone.close()
    print("Completed")
    
    