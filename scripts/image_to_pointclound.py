#! /usr/bin/env python


import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
import cv2.cv as cv

bridge = CvBridge()

# w = 1292
# h = 734
w = 120
h = 73
k = 10.0

x = np.ravel(np.array([[i/k for i in xrange(0, w)] for i in xrange(0,h)], dtype=np.float))
y = np.ravel(np.array([[j/k for i in xrange(0, w)] for j in xrange(0,h)], dtype=np.float))
z = np.ravel(np.array([[0 for i in xrange(0, w)] for i in xrange(0,h)], dtype=np.float))
points_xyz = np.column_stack((x,y,z))

def handle_image(req):

    start = rospy.get_time()

    # Set up point cloud
    fields = [pc2.PointField() for _ in range(3)]

    fields[0].name = "x"
    fields[0].offset = 0
    fields[0].datatype = pc2.PointField.FLOAT32
    fields[0].count = 1

    fields[1].name = "y"
    fields[1].offset = 4
    fields[1].datatype = pc2.PointField.FLOAT32
    fields[1].count = 1

    fields[2].name = "z"
    fields[2].offset = 8
    fields[2].datatype = pc2.PointField.FLOAT32
    fields[2].count = 1

    field_id = 3
    fields.append(pc2.PointField())
    fields[field_id].name = "intensity"
    fields[field_id].datatype = pc2.PointField.FLOAT32
    fields[field_id].offset = 12
    fields[field_id].count = 1
    idx_intensity = field_id

    # add image
    img = bridge.imgmsg_to_cv2(req, desired_encoding="passthrough")
    img = cv2.cvtColor(img, cv.CV_BGR2GRAY)
    img = cv2.resize(img,(120,73))

    points = np.column_stack((points_xyz, np.ravel(img)))

    header = req.header
    header.frame_id = "laser"

    cloud_out = pc2.create_cloud(header, fields, points)
    global pub
    pub.publish(cloud_out)
    #print rospy.get_time() - start

class ImageToPointcloud:

    def __init__(self):

        rospy.init_node('image_to_pointcloud')

        topic_in = rospy.get_param('~in', "/seagoat_node/image_filtered")
        topic_out = rospy.get_param('~out', "~")

        rospy.Subscriber(topic_in, Image, handle_image)
        global pub
        pub = rospy.Publisher(topic_out, PointCloud2, queue_size=2)

        rospy.spin()


if __name__ == "__main__":
    try:
        s = ImageToPointcloud()
    except rospy.ROSInterruptException:
        pass
