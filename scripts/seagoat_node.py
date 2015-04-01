#! /usr/bin/env python


import os.path
from threading import Thread

import rospy

from CapraVision.server.core.manager import VisionManager
from CapraVision.client.gtk.main import WinFilterChain
from gi.repository import Gtk, GObject
from seagoatvision_ros.srv import *
import seagoat_util

# This node is the integration of Seagoat in ROS. See README for documentation

def send(cmd):

    status, reply = comm.communication.instance.send_command(cmd)

    if not status:
        reply = "communication error"

    if reply is None:
        reply = ""
    else:
        rospy.logdebug("handle_controlpanel_set: sending response to client: '" + reply + "'")

    return reply

def handle_show_gui(req):
    global node
    node.show_gui()
    return True

class SeaGoatNode:

    def __init__(self):

        global node
        node = self

        rospy.init_node('seagoat_node')

        visible = rospy.get_param('~gui', False)
        filterchain = rospy.get_param('~filterchain')

        if not os.path.exists(filterchain):
            rospy.logerr("Filterchain not found: '" + filterchain + "'")

        rospy.Service('~show_gui', ShowGui, handle_show_gui)

        calib_file = rospy.get_param('~calibration_file')
        if calib_file != "":
            #copy calibration filter
            seagoat_util.replace_filter(calib_file, filterchain, "PerspectiveCalibration")

        # Directly connected to the vision server
        c = VisionManager()

        if not c.is_connected():
            print("Vision server is not accessible.")
            return

        GObject.threads_init()

        self.w = WinFilterChain(c)
        self.w.load_chain(filterchain)
        #self.w.load_image_source("/home/yohan/Pictures/earth.jpg")
        self.w.load_rosimage_source()

        if visible is True:
            self.show_gui()

        t = Thread(target=Gtk.main)
        t.start()

        rospy.spin()

        # Close connection.
        rospy.loginfo("Closing down seagoat")
        self.w.schedule_quit()
        c.close_server()
        exit()

    def show_gui(self):
        rospy.loginfo("Opening seagoat GUI")
        self.w.schedule_show_gui()

if __name__ == "__main__":
    try:
        s = SeaGoatNode()
    except rospy.ROSInterruptException:
        pass
