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
from screeninfo import get_monitors


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

        self.username = os.environ.get('USER')
        self.builder = gtk.Builder()
        self.builder.add_from_file("/home/"+self.username+"/fish-experiment/glade_docs/myapp.glade")

        self.fish_obj = "fish"
        self.window = self.builder.get_object("FishExperiment")

        self.window.set_icon_from_file("/home/"+self.username+"/fish-experiment/glade_docs/NeuroLab_icon_white.png")
        self.window.connect("delete-event", gtk.main_quit)
        self.window.move(get_monitors()[0].width - 310,0)
        self.window.show()

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

        try:
            select_button = self.builder.get_object("filechoose")
            select_button.connect("file-set",self.dic_set)
        except Exception as e:
            print("Could not find filechoose button: ", e)

        try:
            track_fish = self.builder.get_object("track_fish_button")
            track_fish.connect("toggled",self.track_fish_func)
        except Exception as e:
            print("Could not find track_fish_button: ", e)

        try:
            track_object = self.builder.get_object("track_object_button")
            track_object.connect("toggled",self.track_object_func)
        except Exception as e:
            print("Could not find track_object_button: ", e)

        try:
            start_cam = self.builder.get_object("start_cam")
            start_cam.connect("clicked",self.show_webcam)
        except Exception as e:
            print("Could not find start_cam button: ", e)

        try:
            start_webcam = self.builder.get_object("start_webcam")
            start_webcam.connect("clicked",self.webcamOnly)
        except Exception as e:
            print("Could not find start_webcam button: ", e)

        try:
            self.freq_config = self.builder.get_object("freq_config")
        except Exception as e:
            print("Could not find freq_config entry: ", e)

        try:
            set_freq = self.builder.get_object("set_freq")
            set_freq.connect("clicked",self.set_motor_frequency)
        except Exception as e:
            print("Could not find set_freq button: ", e)

        try:
            self.set_sos = self.builder.get_object("sum_of_sines")
            self.set_sos.connect("toggled",self.set_sos_frequency)
        except Exception as e:
            print("Could not find sum_of_sines button: ", e)

        try:
            kill_cam = self.builder.get_object("kill_cam")
            kill_cam.connect("clicked",self.kill_cam)
        except Exception as e:
            print("Could not find kill_cam button: ", e)
        
        try:
            start_but = self.builder.get_object("start_track")
            start_but.connect("clicked",self.start_track)
        except Exception as e:
            print("Could not find start_track button: ", e)

        try:
            kill_but = self.builder.get_object("kill_track")
            kill_but.connect("clicked",self.kill_track)
        except Exception as e:
            print("Could not find kill_track button: ", e)

        try:
            self.mode_config = self.builder.get_object("mode_config")
            self.mode_config.connect('changed', self.mode_set)
        except:
            try:
                self.mode_config = self.builder.get_object("csv_selector")
                self.mode_config.connect('changed', self.mode_set)
            except Exception as e:
                print("Could not find mode_config/csv_selector combobox: ", e)

        try:
            left_radio_button = self.builder.get_object("left_radio_button")
            left_radio_button.connect("toggled",self.set_fish_direction)
        except Exception as e:
            print("Could not find left_radio_button: ", e)

        self.path = None
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

    def set_fish_direction(self,widget):
        if(widget.get_active()):
            rospy.set_param('fish_direction', 'left')
        else:
            rospy.set_param('fish_direction', 'right')
        self.pubMotorFreq.publish('0.1')
        print(widget.get_active())

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
            #self.arduino_bus.write_i2c_block_data(self.arduino_address,0x00,byteValue)
            return True
        except:
            if not blockPopup:
                dialog = gtk.MessageDialog(
                    transient_for=self.window,
                    flags=0,
                    message_type=gtk.MessageType.ERROR,
                    buttons=gtk.ButtonsType.CANCEL,
                    text="Opps! Arduino is not connected!",
                )
                dialog.format_secondary_text(
                    "Please turn on the Red Button! Then, try again!"
                )
                dialog.run()
                dialog.destroy()
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

    def enableMotor(self):
        buffer = "on"
        self.writeData(buffer)

    def goHomeMotor(self):
        buffer = "home"
        self.writeData(buffer,blockPopup = True)

    def disableMotor(self):
        buffer = "off"
        self.writeData(buffer,blockPopup = True)
    
    def dic_set (self,widget):
        f = open("/home/"+self.username+"/fish-experiment/src/rectrial/launch/launch_track.launch", "w")
        self.path = str(widget.get_filename())
        st_index = int(self.path.rindex(".avi"))
        self.csvPath = self.path[0:st_index]
        f.write('<launch><node pkg = "rectrial" name ="tracker_node" type= "TM_BS_Cuda_deneme" args="'+  self.path + ' ' + self.csvPath + '.csv' + ' ' + self.fish_obj +'"/></launch>')
        f.close()
    
    def start_track(self, widget):
        try:
            self.launch_cam.shutdown()
        except:
            pass
        
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/"+self.username+"/fish-experiment/src/rectrial/launch/launch_track.launch"])
        self.launch.start()

    def kill_track(self, widget):
        try:
            self.launch.shutdown()
            os.system("clear")
        except:
            pass

    def show_webcam(self,widget):
        try:
            self.goHomeMotor()
        except:
            pass
        self.uuid1 = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid1)
        self.launch_cam= roslaunch.parent.ROSLaunchParent(self.uuid1, ["/home/"+self.username+"/fish-experiment/src/rectrial/launch/launch_pub.launch"])
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
        self.launch_cam= roslaunch.parent.ROSLaunchParent(self.uuid1, ["/home/"+self.username+"/fish-experiment/src/rectrial/launch/launch_webcam.launch"])
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