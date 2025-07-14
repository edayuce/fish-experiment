#!/usr/bin/env python3
# license removed for brevity

import gi
import time
import roslaunch
import rospy
from std_msgs.msg import String
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk as gtk
import numpy as np
import os
import smbus

import Jetson.GPIO as GPIO

import PySimpleGUI as sg
from screeninfo import get_monitors
import socket

    

class MyApp (object):

    def __init__(self):
        self.defaultFreq = 0.20
        self.defaultMode = 0
        # Nvidia Jetson Nano i2c Bus 8
        self.arduino_bus = smbus.SMBus(8)

        # This is the address we setup in the Arduino Program
        self.arduino_address = 0x40

        self.pubMotorFreq = rospy.Publisher('set_motor_freq', String, queue_size=100)
        rospy.init_node('interface', anonymous=True)


        GPIO.setwarnings(False)

        self.interrupt_pin = 7
        self.reset_pin = 29
        self.state = GPIO.LOW

        GPIO.setmode(GPIO.BOARD) 
        GPIO.setup(self.interrupt_pin, GPIO.OUT, initial=self.state)
        GPIO.setup(self.reset_pin, GPIO.OUT, initial=GPIO.LOW)

        self.username = os.environ.get('USER')
        self.builder = gtk.Builder()
        self.builder.add_from_file("/home/"+self.username+"/Desktop/fish_experiment_ws/glade_docs/myapp.glade")

        self.fish_obj = "fish"
        self.window = self.builder.get_object("FishExperiment")

        self.window.set_icon_from_file("/home/"+self.username+"/Desktop/fish_experiment_ws/glade_docs/NeuroLab_icon_white.png")

        #self.window.set_position(gtk.WindowPosition.CENTER_ALWAYS)
        self.window.connect("delete-event", gtk.main_quit)
        self.window.move(get_monitors()[0].width - 310,0)
        self.window.show()

        start_cam = self.builder.get_object("start_cam")
        start_cam.connect("clicked",self.show_webcam)

        start_webcam = self.builder.get_object("start_webcam")
        start_webcam.connect("clicked",self.webcamOnly)

        kill_cam = self.builder.get_object("kill_cam")
        kill_cam.connect("clicked",self.kill_cam)

        left_radio_button = self.builder.get_object("left_radio_button")
        left_radio_button.connect("toggled",self.set_fish_direction)


        self.mode_config = self.builder.get_object("csv_selector")
        self.mode_config.connect('changed', self.mode_set)
        

        self.falseEventFlag = False
        self.previousMode = self.defaultMode

        rospy.set_param('csv_folder', "electro")
        rospy.set_param('fish_direction', 'left')
        
        gtk.main()
    

    def set_fish_direction(self,widget):

        if(widget.get_active()):
            rospy.set_param('fish_direction', 'left')
        else:
            rospy.set_param('fish_direction', 'right')

        self.pubMotorFreq.publish('0.1')
        print(widget.get_active())

    def get_resource_path(self,rel_path):
        dir_of_py_file = os.path.dirname(__file__)
        rel_path_to_resource = os.path.join(dir_of_py_file, rel_path)
        abs_path_to_resource = os.path.abspath(rel_path_to_resource)
        return abs_path_to_resource

    def mode_set(self, combobox):

        
        model = combobox.get_model()
        index = combobox.get_active()
        myIter = combobox.get_active_iter()
        name = model[myIter][0]

        print(name)
        print(index)
        rospy.set_param('csv_folder', name)
        self.pubMotorFreq.publish('0.1')

    
    def track_fish_func(self, widget):
        self.fish_obj= "fish"

    def track_object_func(self, widget):
        self.fish_obj= "object"


    def StringToBytes(self, val):
        retVal = []
        for c in val:
                retVal.append(ord(c))
        return retVal

    def writeData(self, value, blockPopup = False):

        try:
            byteValue = self.StringToBytes(value)    
            self.arduino_bus.write_i2c_block_data(self.arduino_address,0x00,byteValue) #first byte is 0=command byte.. just is.
            return True
        except:
            if not blockPopup:
                sg.Popup('Opps!', 'Arduino is not connected!','Please turn on the Red Button! Then, try again!')
            return False

        
    
    def set_motor_frequency(self):

        my_freq = "0.1"

        self.pubMotorFreq.publish(my_freq)
           

    def enableMotor(self):
        buffer = "on"

        self.writeData(buffer)

    def goHomeMotor(self):
        buffer = "home"

        self.writeData(buffer,blockPopup = True)


    def disableMotor(self):
        buffer = "off"

        self.writeData(buffer,blockPopup = True)


    def show_webcam(self,widget):
        try:
            self.goHomeMotor()
        except:
            pass
        self.uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid1)
        self.launch_cam= roslaunch.parent.ROSLaunchParent(self.uuid1, ["/home/"+self.username+"/Desktop/fish_experiment_ws/src/retrical/launch/launch_pub.launch"])
        self.launch_cam.start()
        rospy.loginfo("Nodes are running")
    
    def webcamOnly(self,widget):
        self.uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid1)
        self.launch_cam= roslaunch.parent.ROSLaunchParent(self.uuid1, ["/home/"+self.username+"/Desktop/fish_experiment_ws/src/retrical/launch/launch_webcam.launch"])
        self.launch_cam.start()
        rospy.loginfo("Nodes are running")


    def kill_cam(self,widget):
        try:
            self.disableMotor()
        except:
            pass
        GPIO.output(self.interrupt_pin, GPIO.LOW)

        GPIO.output(self.reset_pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.reset_pin,GPIO.LOW)

        self.launch_cam.shutdown()
        

        os.system("clear")
 
   

if __name__ == "__main__":
    
    main=MyApp()


