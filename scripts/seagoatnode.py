#! /usr/bin/env python

import rospy
import os
from threading import Thread
from time import sleep

import rospy

from CapraVision.server.core.manager import VisionManager
from CapraVision.client.gtk.main import WinFilterChain
from gi.repository import Gtk, GObject
from seagoatvision_ros.srv import *

def handle_image(req):
    print "got image"

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

        s_show_gui = rospy.Service('~show_gui', ShowGui, handle_show_gui)

        # Directly connected to the vision server
        c = VisionManager()

        if not c.is_connected():
            print("Vision server is not accessible.")
            return

        GObject.threads_init()

        self.w = WinFilterChain(c)
        self.w.load_chain("/home/yohan/Ibex/src/seagoatvision_ros/filterchain/test2.filterchain")
        #self.w.load_image_source("/home/yohan/Pictures/earth.jpg")
        self.w.load_rosimage_source()

        if visible is True:
            self.show_gui()

        t = Thread(target=Gtk.main)
        t.start()

        rospy.spin()

        # Close connection.
        rospy.loginfo( "Closing down seagoat")
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
