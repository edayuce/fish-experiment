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

# import Jetson.GPIO as GPIO

import PySimpleGUI as sg

    

class MyApp (object):

    def __init__(self):
        self.defaultFreq = 0.20
        self.defaultMode = 0
        # Nvidia Jetson Nano i2c Bus 8
        #self.arduino_bus = smbus.SMBus(8)

        # This is the address we setup in the Arduino Program
        self.arduino_address = 0x40

        self.pubMotorFreq = rospy.Publisher('set_motor_freq', String, queue_size=100)
        self.pubState = rospy.Publisher('set_state', String, queue_size=100)
        rospy.init_node('interface', anonymous=True)

        # GPIO.setwarnings(False)

        self.interrupt_pin = 7
        self.reset_pin = 29
        # self.state = GPIO.LOW

        # GPIO.setmode(GPIO.BOARD) 
        # GPIO.setup(self.interrupt_pin, GPIO.OUT, initial=self.state)
        # GPIO.setup(self.reset_pin, GPIO.OUT, initial=GPIO.LOW)

        self.username = os.environ.get('USER')
        self.builder = gtk.Builder()
        self.builder.add_from_file("/home/"+self.username+"/Desktop/Experiment_Materials/glade_docs/myapp.glade")

        self.fish_obj = "fish"
        window = self.builder.get_object("FishExperiment")
        window.set_position(gtk.WindowPosition.CENTER_ALWAYS)
        window.connect("delete-event", gtk.main_quit)
        window.show()
                

        start_cam = self.builder.get_object("start_cam")
        start_cam.connect("clicked",self.show_webcam)

        start_webcam = self.builder.get_object("start_webcam")
        start_webcam.connect("clicked",self.webcamOnly)

        self.freq_config = self.builder.get_object("freq_config")

        self.gain_config = self.builder.get_object("gain_config")

        set_freq = self.builder.get_object("set_freq")
        set_freq.connect("clicked",self.set_motor_frequency)

        self.set_sos = self.builder.get_object("sum_of_sines")
        self.set_sos.connect("toggled",self.set_sos_frequency)

        self.set_gain = self.builder.get_object("gain_check")
        self.set_gain.connect("toggled",self.set_reafferent_gain)

        self.set_openloop = self.builder.get_object("open_loop")
        self.set_openloop.connect("toggled",self.openloop)

        self.set_closedloop = self.builder.get_object("closed_loop")
        self.set_closedloop.connect("toggled",self.closedloop)

        self.set_olg = self.builder.get_object("openloop_gain")
        self.set_olg.connect("toggled",self.openloop_gain)


        #self.set_sos = self.builder.get_object("close_loop")
        #self.set_sos.connect("toggled",self.closeloop)

        kill_cam = self.builder.get_object("kill_cam")
        kill_cam.connect("clicked",self.kill_cam)
        

        self.mode_config = self.builder.get_object("mode_config")
        self.mode_config.connect('changed', self.mode_set)
        
        """ self.trash_text = self.get_ent.get_text()
        self.path = None """

        self.falseEventFlag = False
        self.previousMode = self.defaultMode
        rospy.set_param('/UserSet', str(self.previousMode +1))
        
        gtk.main()

    def mode_set(self, combobox):

        
        model = combobox.get_model()
        index = combobox.get_active()

        if self.previousMode == index:
            return 0

        else:
            self.previousMode = index

        rospy.set_param('/UserSet', str(index +1))        
        print(self.previousMode)



    def StringToBytes(self, val):
        retVal = []
        for c in val:
                retVal.append(ord(c))
        return retVal

    
    def writeData(self, value, blockPopup = False):

        try:
            byteValue = self.StringToBytes(value)    
            #self.arduino_bus.write_i2c_block_data(self.arduino_address,0x00,byteValue) #first byte is 0=command byte.. just is.
            return True
        except:
            if not blockPopup:
                sg.Popup('Opps!', 'Arduino is not connected!','Please turn on the Red Button! Then, try again!')
            return False

        
    
    def set_motor_frequency(self,widget):
        if((not self.set_sos.get_active()) and (not self.set_openloop.get_active()) and (not self.set_closedloop.get_active())):
            text = self.freq_config.get_text().strip()
            my_freq = text.replace(',', '.')

            self.pubMotorFreq.publish(my_freq)
        else:
            self.pubMotorFreq.publish("sum")

    
    def set_sos_frequency(self,button):
        if button.get_active():
            self.pubMotorFreq.publish("sum")

    
    def set_reafferent_gain(self,button):
        if button.get_active():
            text = self.gain_config.get_text().strip()
            my_gain = text.replace(',', '.')
            self.pubMotorFreq.publish("gain:"+my_gain)
            print("gain:"+my_gain)

    
    def openloop(self,button):
        if button.get_active():
            self.pubMotorFreq.publish("openloop")
        else:
            text = self.freq_config.get_text().strip()
            my_freq = text.replace(',', '.')
            self.pubMotorFreq.publish(my_freq)
    
    def closedloop(self,button):
        if button.get_active():
            self.pubMotorFreq.publish("closedloop")
        else:
            text = self.freq_config.get_text().strip()
            my_freq = text.replace(',', '.')
            self.pubMotorFreq.publish(my_freq)



    def openloop_gain(self,button):
         if (button.get_active()):
             text = self.gain_config.get_text().strip()
             my_gain = text.replace(',', '.')
             self.pubMotorFreq.publish("ol_gain:"+my_gain)
             print("gain:"+my_gain)




    def enableMotor(self):
        buffer = "on"

        self.writeData(buffer)

    def goHomeMotor(self):
        buffer = "home"

        self.writeData(buffer,blockPopup = True)


    def disableMotor(self):
        buffer = "off"

        self.writeData(buffer,blockPopup = True)
    
    def get_ent_func(self,widget):
        f = open("/home/"+self.username+"/catkin_ws/src/track_deneme/launch/launch_track.launch", "w")
        self.trash_text = self.get_ent.get_text()
        #print(self.trash_text)
        #f.write('<launch><node pkg = "track_deneme" name ="tracker_node" type= TM_BS_Cuda_deneme" args="'+  self.path + ' ' + self.csvPath + '.csv' + ' ' + str(self.trash_text) + '"/></launch>')
        f.write('<launch><node pkg = "track_deneme" name ="tracker_node" type= TM_BS_Cuda_deneme" args="'+  self.path + ' ' + self.csvPath + '.csv' + ' ' + self.fish_obj + '"/></launch>')
        f.close()


    def show_webcam(self,widget):
        try:
            self.goHomeMotor()
        except:
            pass
        self.uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid1)
        self.launch_cam= roslaunch.parent.ROSLaunchParent(self.uuid1, ["/home/"+self.username+"/catkin_ws/src/rectrial/launch/launch_pub.launch"])
        self.launch_cam.start()
        time.sleep(2)
        ## Set Motor Freq by Default
        if(not self.set_sos.get_active() and not self.set_closedloop.get_active()):
            text = self.freq_config.get_text().strip()
            my_freq = text.replace(',', '.')

            self.pubMotorFreq.publish(my_freq)
        elif not self.set_sos.get_active():
            self.pubMotorFreq.publish("closedloop")
        else:
            self.pubMotorFreq.publish("sum")
        rospy.loginfo("Nodes are running")
    
    def webcamOnly(self,widget):
        self.uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid1)
        self.launch_cam= roslaunch.parent.ROSLaunchParent(self.uuid1, ["/home/"+self.username+"/catkin_ws/src/rectrial/launch/launch_webcam.launch"])
        self.launch_cam.start()
        rospy.loginfo("Nodes are running")


    def kill_cam(self,widget):
        try:
            self.disableMotor()
        except:
            pass    


        time.sleep(0.5)
        # GPIO.output(self.reset_pin,GPIO.LOW)
        self.pubState.publish("shutdown")
        time.sleep(2)

        self.launch_cam.shutdown()
        


if __name__ == "__main__":
    
    main=MyApp()


