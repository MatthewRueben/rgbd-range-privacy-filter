#!/usr/bin/env python
# Applies a privacy filter (i.e., blur or redaction) to an image based on depth values.
# 1) Thresholds the depth image.
# 2) Applies it to the RGB image as a mask.
# 3) Applies a privacy filter. 

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv
from copy import deepcopy as copy
from scipy.ndimage.filters import gaussian_filter

def copy_Image_empty_data(image_old):
    image_new = Image()
    image_new.header = copy(image_old.header)
    image_new.height = copy(image_old.height)
    image_new.width = copy(image_old.width)
    image_new.encoding = copy(image_old.encoding)
    image_new.is_bigendian = copy(image_old.is_bigendian)
    image_new.step = copy(image_old.step)
    return image_new

class Filterer():
    def __init__(self):
        self.have_rgb = False
        self.have_depth = False

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/rgb/image_color', Image, self.callback_rgb)
        rospy.Subscriber('/camera/depth_registered/hw_registered/image_rect_raw', Image, self.callback_depth)
        self.pubber_filtered = rospy.Publisher('/camera/rgb/image_rect_color/filtered', Image)

    def run(self):
        while not rospy.is_shutdown():
            if self.have_rgb and self.have_depth:
                # convert image data to numpy arrays
                depth_array = numpy.asarray(self.bridge.imgmsg_to_cv(self.depth, self.depth.encoding))
                rgb_array = numpy.asarray(self.bridge.imgmsg_to_cv(self.rgb, self.rgb.encoding))
                
                # apply filter via mask
                mask = numpy.logical_or(depth_array > 2000,  # threshold (mm)
                                        depth_array == 0)  # bad values
                mask = numpy.tile(mask[:,:,numpy.newaxis], (1,1,3))  # make mask same size as rgb image
                rgb_array_blurry = numpy.zeros(rgb_array.shape)
                for i in range(3):  # apply filter to each color channel
                    rgb_array_blurry[:,:,i] = gaussian_filter(rgb_array[:,:,i], sigma=7)
                rgb_array[mask] = rgb_array_blurry[mask]  # make far-off things blurry
                
                # convert RGB array back to image msg and publish
                rgb_filtered = copy_Image_empty_data(self.rgb)  # note: timestamp unchanged
                rgb_filtered.data = self.bridge.cv_to_imgmsg(cv.fromarray(rgb_array), encoding=self.rgb.encoding).data
                self.pubber_filtered.publish(rgb_filtered)

    def callback_rgb(self, rgb):
        #rospy.loginfo('Got RGB!   {0}'.format(rgb.header.stamp))
        self.rgb = rgb
        if not self.have_rgb:
            self.have_rgb = True

    def callback_depth(self, depth):
        #rospy.loginfo('Got DEPTH! {0}'.format(depth.header.stamp))
        self.depth = depth
        if not self.have_depth:
            self.have_depth = True

if __name__ == "__main__":
    rospy.init_node('range_filter')
    filterer = Filterer()
    filterer.run()
