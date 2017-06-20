#!/usr/bin/env python
"""
Copyright (c) 2017 Yuxiang Gao

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import signal
import time
import numpy as np
import rospy
import camera_info_manager
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import gi
import sys
sys.path.insert(1, '/usr/local/lib/python2.7/site-packages')
import cv2
print cv2.__version__
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

"""
This script is for parsing and publishing images captured from thr raspberry pi camera.
      
"""

class pi_cam:
    def __init__(self, cam_num, camera_info_url, width, height):
        self.cam_name = 'pi_cam_' + str(cam_num)
        self.camera_info_url = camera_info_url
        self.frame_id = 'pi_cam'
        self.width = width
        self.height = height
        
        self.cname = camera_info_manager.genCameraName(self.cam_name)
        self.cinfo = camera_info_manager.CameraInfoManager(cname=self.cam_name, url=self.camera_info_url)
        self.cinfo.loadCameraInfo()
        self.infoPub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=1)

        self.bridge = CvBridge()
        self.imgPub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        Gst.init(None)
        self.pipe = Gst.parse_launch("""nvcamerasrc sensor-id=""" + str(cam_num) + """ fpsRange="30 30" ! video/x-raw(memory:NVMM), width=(int)""" + str(width) + """, height=(int)""" + str(height) + """, format=(string)I420, framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink sync=false max-buffers=2 drop=true name=sink emit-signals=true""")
        self.sink = self.pipe.get_by_name('sink')
        self.pipe.set_state(Gst.State.PLAYING)

    def cam_pub(self):
        sample = self.sink.emit('pull-sample')    
        img = self.gst_to_opencv(sample.get_buffer())
        try:
            self.imgPub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            # print "pub"
            # cv2.waitKey(10)
            # cv2.imshow('Video', img)
        except CvBridgeError as e:
            self.stop()
            print(e)

    def info_pub(self):
        infoMsg = self.cinfo.getCameraInfo()
        infoMsg.header.stamp = rospy.Time.now()
        infoMsg.header.frame_id = self.frame_id
        infoMsg.width = self.width
        infoMsg.height = self.height
        self.infoPub.publish(infoMsg)

    def gst_to_opencv(self, gst_buffer):
        return np.ndarray((self.height, self.width, 3),
                          buffer=gst_buffer.extract_dup(0, gst_buffer.get_size()),
                          dtype=np.uint8)

    def set_pause(pause):
        if pause:
            self.pipe.set_state(Gst.State.PAUSE)
        else:
            self.pipe.set_state(Gst.State.PLAYING)

    def stop(self):
        self.pipe.set_state(Gst.State.NULL)

def main():
    rospy.init_node('GstRasPiCam',anonymous=False)
    cam_num = rospy.get_param('~cam_num', 1)
    cam_info_url = rospy.get_param('~cam_info_url', '')
    width = rospy.get_param('width', 820)
    height = rospy.get_param('height', 616)

    rospy.loginfo('camnum is %d', cam_num)
    piCam = pi_cam(cam_num, cam_info_url, width, height)

    print 'gstRasPiCam ' + str(cam_num) + ' start'
   
    while not rospy.is_shutdown():
        piCam.cam_pub()
        piCam.info_pub()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
