#! /usr/bin/env python
import numpy as np
from scipy.ndimage import filters
import cv2,math  
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_msgs.msg import  Header
class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        self.cmsg2=None
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("frames",Image,self.callback)

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        
        self.cmsg = CompressedImage()
        self.cmsg.header.stamp = rospy.Time.now()
        self.cmsg.format = "jpeg"
        self.cmsg.data = np.array(cv2.imencode('.jpg', self.image)[1]).tostring()
        self.cmsg2=self.br.compressed_imgmsg_to_cv2(self.cmsg)
        #self.image = self.br.imgmsg_to_cv2(msg)
        #self.dtype, self.n_channels = self.br.encoding_to_cvtype2('8UC3')
    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():

            rospy.loginfo('publishing image')
            br = CvBridge()

            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.cmsg2,'bgr8'))
                #self.pub.publish(self.image,"bgr8")
                

            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("imagetimer111", anonymous=True)
    my_node = Nodo()
    my_node.start()